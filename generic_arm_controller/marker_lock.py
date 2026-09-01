"""Aggancio ("lock") one-shot della posa di un frame rilevato dalla camera.

------------------------------------------------------------------------------
PERCHE' UN LOCK E NON UNA RILETTURA CONTINUA
------------------------------------------------------------------------------
Con la camera montata sulla flangia (eye-in-hand) la posa del marker rispetto
alla base si ottiene componendo:

    ^B T_M  =  ^B T_E  ·  ^E T_C  ·  ^C T_M

dove ^E T_C e' la calibrazione mano-occhio. Un errore residuo su ^E T_C (che
c'e' sempre: e' la parte piu' difficile da calibrare) NON produce un offset
costante su ^B T_M, ma un errore che dipende da ^B T_E, cioe' DAL PUNTO DI
VISTA da cui si guarda il marker. Rileggere il marker a ogni waypoint significa
quindi inseguire un bersaglio che si sposta di qualche millimetro (o centimetro)
tra un waypoint e il successivo - e per giunta proprio durante l'avvicinamento
finale, dove il marker si vede da vicino e di taglio e la stima ArUco e' al suo
peggio.

Il comportamento corretto e' quello dell'operatore umano: guardo il marker UNA
VOLTA da una posizione comoda, mi segno dov'e' rispetto alla base, e da li' in
poi mi muovo su quella stima - non importa se il marker esce dall'inquadratura.
Tutti i waypoint della sequenza restano cosi' riferiti alla STESSA misura, e la
geometria relativa insegnata (pre-grasp -> grasp) viene riprodotta esattamente
anche in presenza di un offset globale.

Questo modulo implementa esattamente quello: campiona N rilevazioni buone, ne fa
la media, e restituisce un ^B T_M che il chiamante congela per tutta la sequenza
(esecuzione) o per tutta la sessione di insegnamento (salvataggio).

------------------------------------------------------------------------------
COSA RENDE "BUONA" UNA RILEVAZIONE
------------------------------------------------------------------------------
Tre filtri, tutti necessari:

1. ETA'. `lookup_transform(..., Time())` significa "ultimo istante disponibile"
   e il buffer tf2 conserva i dati per 10 s: per 10 secondi dopo che ArUco ha
   smesso di pubblicare, il lookup CONTINUA A RIUSCIRE restituendo la vecchia
   rilevazione, senza sollevare alcuna eccezione. Senza un controllo esplicito
   sul timestamp e' impossibile distinguere "marker inquadrato adesso" da
   "marker sparito 8 secondi fa".

2. DISTANZA. Da molto vicino il marker occupa quasi tutta l'immagine, esce
   parzialmente dall'inquadratura e l'ambiguita' di posa del singolo marker
   diventa dominante: la rilevazione "riesce" ma il numero e' sbagliato di
   centimetri.

3. DISPERSIONE tra i campioni. E' l'indice di qualita' piu' informativo che si
   possa avere gratis: se N rilevazioni consecutive dello stesso marker fermo
   differiscono di parecchi millimetri, quella vista non e' affidabile e va
   segnalato all'operatore prima che insegni o esegua qualcosa sopra.
"""

import math
import time

from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException


# Soglie di sola diagnostica: oltre queste il lock viene comunque acquisito (non
# vogliamo bloccare l'operativita'), ma con un warning ben visibile.
SPREAD_WARN_M = 0.010       # 10 mm
SPREAD_WARN_DEG = 5.0


def _norm3(t):
    return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)


def average_transforms(samples):
    """Media di N TransformStamped con lo stesso parent/child frame.

    Traslazione: media aritmetica. Rotazione: media dei quaternioni con
    allineamento di segno sul primo campione (q e -q sono la stessa rotazione:
    senza allineamento due campioni identici ma di segno opposto si
    cancellerebbero) e rinormalizzazione finale. E' l'approssimazione standard,
    valida finche' i campioni sono vicini tra loro - condizione garantita qui dal
    fatto che si mediano rilevazioni consecutive dello stesso marker fermo.

    Ritorna (TransformStamped, spread_m, spread_deg), dove lo spread e' la
    deviazione massima dei campioni rispetto alla media.
    """
    n = len(samples)
    tx = sum(s.transform.translation.x for s in samples) / n
    ty = sum(s.transform.translation.y for s in samples) / n
    tz = sum(s.transform.translation.z for s in samples) / n

    q0 = samples[0].transform.rotation
    qx = qy = qz = qw = 0.0
    for s in samples:
        q = s.transform.rotation
        if (q.x * q0.x + q.y * q0.y + q.z * q0.z + q.w * q0.w) < 0.0:
            qx -= q.x
            qy -= q.y
            qz -= q.z
            qw -= q.w
        else:
            qx += q.x
            qy += q.y
            qz += q.z
            qw += q.w
    nq = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if nq < 1e-9:
        qx, qy, qz, qw, nq = q0.x, q0.y, q0.z, q0.w, 1.0
    qx, qy, qz, qw = qx / nq, qy / nq, qz / nq, qw / nq

    spread_m = 0.0
    spread_deg = 0.0
    for s in samples:
        t = s.transform.translation
        spread_m = max(
            spread_m,
            math.sqrt((t.x - tx) ** 2 + (t.y - ty) ** 2 + (t.z - tz) ** 2)
        )
        q = s.transform.rotation
        dot = abs(q.x * qx + q.y * qy + q.z * qz + q.w * qw)
        dot = min(1.0, max(-1.0, dot))
        spread_deg = max(spread_deg, math.degrees(2.0 * math.acos(dot)))

    avg = TransformStamped()
    avg.header.frame_id = samples[0].header.frame_id
    avg.child_frame_id = samples[0].child_frame_id
    avg.transform.translation.x = tx
    avg.transform.translation.y = ty
    avg.transform.translation.z = tz
    avg.transform.rotation.x = qx
    avg.transform.rotation.y = qy
    avg.transform.rotation.z = qz
    avg.transform.rotation.w = qw
    return avg, spread_m, spread_deg


class MarkerLocker:
    """Aggancia e conserva ^B T_M per uno o piu' frame rilevati dalla camera.

    Il chiamante decide QUANDO rilasciare il lock (inizio sequenza per
    l'esecutore, servizio esplicito per il salvataggio): finche' il lock e'
    attivo, `get()` restituisce sempre la stessa stima e il marker puo'
    tranquillamente uscire dall'inquadratura.
    """

    def __init__(self, node, tf_buffer, base_frame, camera_optical_frame,
                 min_distance_m, max_age_s, samples, sample_period_s=0.1):
        self._node = node
        self._tf = tf_buffer
        self.base_frame = base_frame
        self.camera_optical_frame = camera_optical_frame
        self.min_distance_m = float(min_distance_m)
        self.max_age_s = float(max_age_s)
        self.samples = max(1, int(samples))
        self.sample_period_s = float(sample_period_s)

        self._locked = {}       # frame -> TransformStamped (base_frame <- frame)
        self._locked_info = {}  # frame -> descrizione leggibile del lock

    # ------------------------------------------------------------------
    # Stato del lock
    # ------------------------------------------------------------------

    def is_locked(self, frame):
        return frame in self._locked

    def get(self, frame):
        return self._locked.get(frame)

    def info(self, frame):
        return self._locked_info.get(frame, "nessun aggancio")

    def release(self, frame=None):
        if frame is None:
            self._locked.clear()
            self._locked_info.clear()
        else:
            self._locked.pop(frame, None)
            self._locked_info.pop(frame, None)

    # ------------------------------------------------------------------
    # Campionamento
    # ------------------------------------------------------------------

    def sample_once(self, frame):
        """Una singola rilevazione validata. Ritorna (TransformStamped, "") se
        buona, (None, motivo) altrimenti. Non modifica il lock: e' usabile anche
        come semplice sonda "il marker si vede adesso?"."""
        now = self._node.get_clock().now()

        try:
            cam = self._tf.lookup_transform(
                self.camera_optical_frame, frame, Time(),
                timeout=Duration(seconds=0.05)
            )
        except TransformException as ex:
            return None, (f"'{frame}' non rilevato da "
                          f"'{self.camera_optical_frame}' ({ex})")

        # Filtro 1: eta' della rilevazione (vedi nota in cima al modulo).
        stamp = cam.header.stamp
        if stamp.sec or stamp.nanosec:
            detected_at = Time(seconds=stamp.sec, nanoseconds=stamp.nanosec,
                               clock_type=now.clock_type)
            age = (now - detected_at).nanoseconds / 1e9
            if age > self.max_age_s:
                return None, (f"rilevazione di '{frame}' vecchia di {age:.2f} s "
                              f"(max {self.max_age_s:.2f} s): il marker non e' "
                              f"piu' inquadrato, la TF e' solo residuo del "
                              f"buffer tf2")

        # Filtro 2: distanza minima dalla camera.
        dist = _norm3(cam.transform.translation)
        if dist < self.min_distance_m:
            return None, (f"'{frame}' a soli {dist:.3f} m da "
                          f"'{self.camera_optical_frame}' (minimo "
                          f"{self.min_distance_m:.2f} m): troppo vicino, la "
                          f"stima ArUco e' inaffidabile")

        # Composizione fino alla base. Time() = ultimo istante comune a tutta la
        # catena: poiche' il link flangia->camera e' statico, l'istante comune
        # coincide con il timestamp della rilevazione, quindi ^B T_E e ^C T_M
        # sono presi allo stesso istante (composizione coerente).
        try:
            ts = self._tf.lookup_transform(
                self.base_frame, frame, Time(),
                timeout=Duration(seconds=0.05)
            )
        except TransformException as ex:
            return None, (f"TF '{self.base_frame}' <- '{frame}' non "
                          f"disponibile ({ex})")

        return ts, ""

    def acquire(self, frame):
        """Acquisisce (o ri-acquisisce) il lock mediando N campioni validi.
        Ritorna (TransformStamped, messaggio) oppure (None, motivo)."""
        collected = []
        last_reason = "nessun campione raccolto"
        for i in range(self.samples):
            if i:
                time.sleep(self.sample_period_s)
            ts, reason = self.sample_once(frame)
            if ts is None:
                last_reason = reason
            else:
                collected.append(ts)

        if not collected:
            return None, f"aggancio su '{frame}' NON riuscito: {last_reason}"

        avg, spread_m, spread_deg = average_transforms(collected)
        avg.header.stamp = self._node.get_clock().now().to_msg()

        t = avg.transform.translation
        msg = (f"Aggancio su '{frame}' acquisito da {len(collected)}/"
               f"{self.samples} campioni: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) m "
               f"in '{self.base_frame}', dispersione {spread_m * 1000:.1f} mm / "
               f"{spread_deg:.2f} deg")
        if spread_m > SPREAD_WARN_M or spread_deg > SPREAD_WARN_DEG:
            msg += (" - DISPERSIONE ALTA: rilevazione instabile, conviene "
                    "riagganciare da una posa piu' frontale e piu' vicina")

        self._locked[frame] = avg
        self._locked_info[frame] = msg
        return avg, msg

    def get_or_acquire(self, frame):
        """Ritorna il lock esistente, acquisendolo alla prima chiamata.
        Ritorna (TransformStamped|None, messaggio, appena_acquisito)."""
        existing = self._locked.get(frame)
        if existing is not None:
            return existing, self._locked_info.get(frame, ""), False
        ts, msg = self.acquire(frame)
        return ts, msg, ts is not None

import rclpy
from rclpy.node import Node
import json
import os
import datetime
import shutil  # <-- Aggiunto per gestire i backup dei file completi

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from generic_arm_interfaces.srv import SavePose
from generic_arm_interfaces.srv import GripperCommand as GripperSrv
from std_srvs.srv import Trigger

from generic_arm_controller.marker_lock import MarkerLocker


class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')
        self.get_logger().info('Pose Saver Node Started.')

        # Declare ROS2 parameters for flexibility across different robots
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'tool0')

        # Read parameters from ROS2 launch configuration
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value

        # ------------------------------------------------------------------
        # Aggancio ("lock") del marker: guardo una volta, poi vado alla cieca
        # ------------------------------------------------------------------
        # Tutte le pose insegnate relative al marker devono essere riferite alla
        # STESSA misura di ^B T_M, altrimenti la geometria relativa che si sta
        # insegnando (pre-grasp -> grasp -> sollevamento) risulta costruita su
        # stime diverse del marker, prese da punti di vista diversi durante il
        # jog: con la camera sulla flangia l'errore residuo di calibrazione
        # mano-occhio dipende dal punto di vista, quindi quelle stime NON
        # coincidono e i waypoint insegnati sono reciprocamente incoerenti.
        #
        # Percio': la posa del marker viene agganciata UNA VOLTA (appena e'
        # inquadrato in modo affidabile) e resta fissa per tutta la sessione di
        # insegnamento. Da quel momento il marker puo' uscire dall'inquadratura
        # quanto si vuole - anzi, e' proprio quello che succede avvicinandosi
        # all'oggetto. Se l'oggetto viene spostato, si riaggancia con
        # /relock_marker.
        #
        # Vedi marker_lock.py per i filtri applicati (eta' della rilevazione,
        # distanza minima, dispersione tra campioni).
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_min_reliable_distance_m', 0.20)
        self.declare_parameter('marker_max_tf_age_s', 0.5)
        self.declare_parameter('marker_lock_samples', 5)
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.marker_min_reliable_distance_m = self.get_parameter(
            'marker_min_reliable_distance_m').value
        self.marker_max_tf_age_s = self.get_parameter('marker_max_tf_age_s').value
        self.marker_lock_samples = self.get_parameter('marker_lock_samples').value

        self.tf_buffer = Buffer()
        # spin_thread=True e' necessario, non cosmetico: il TransformListener
        # sottoscrive /tf sul nodo, e questo nodo gira su un executor
        # mono-thread. Senza un thread dedicato, ogni callback che blocca in
        # attesa di una TF (save_pose aspetta fino a 3 s) impedisce al nodo di
        # processare i messaggi /tf proprio mentre li sta aspettando: il buffer
        # non si aggiorna e il timeout non puo' mai essere soddisfatto.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.all_poses          = []

        # Manteniamo la tua cartella e il tuo file originale (L'esecutore leggerà sempre questo!)
        workspace_dir    = os.path.expanduser('~/ros2_ws/src')
        self.task_result_dir  = os.path.join(workspace_dir, 'task_result')
        os.makedirs(self.task_result_dir, exist_ok=True)
        self.json_file_path = os.path.join(self.task_result_dir, 'robot_poses_ws.json')
        self.get_logger().info(f"Saving to: {self.json_file_path}")

        # Carica le pose accumulate in precedenza se il file esiste già
        self._load_poses()

        self.save_pose_service = self.create_service(
            SavePose, 'save_pose', self.save_pose_callback
        )

        self.save_gripper_service = self.create_service(
            GripperSrv, 'save_gripper_action', self.save_gripper_action_callback
        )

        self.clear_poses_service = self.create_service(
            Trigger, 'clear_saved_poses', self.clear_poses_callback
        )

        self.relock_marker_service = self.create_service(
            Trigger, 'relock_marker', self.relock_marker_callback
        )

        self.get_logger().info(
            'Services ready: /save_pose, /save_gripper_action, '
            '/clear_saved_poses, /relock_marker'
        )

        # Frame la cui posa va agganciata invece che riletta a ogni salvataggio.
        # Indicizzato per nome: piu' marker (es. "marker_26", "marker_28") hanno
        # ognuno il proprio lock indipendente, senza logica aggiuntiva.
        self._trackable_frames      = ("aruco_marker",)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._locker = MarkerLocker(
            node=self,
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            camera_optical_frame=self.camera_optical_frame,
            min_distance_m=self.marker_min_reliable_distance_m,
            max_age_s=self.marker_max_tf_age_s,
            samples=self.marker_lock_samples,
        )
        # Il timer si limita ad agganciare il marker la PRIMA volta che lo vede:
        # una volta agganciato non fa piu' nulla (niente ri-pubblicazioni
        # continue su /tf_static, che e' un topic latched e non va usato come
        # stream).
        self._auto_lock_timer = self.create_timer(1.0, self._auto_lock_tick)

    # ------------------------------------------------------------------
    # Aggancio marker
    # ------------------------------------------------------------------

    def _locked_frame_name(self, frame: str) -> str:
        # Nome distinto da quello dell'esecutore ("_locked_run"): i due nodi
        # agganciano il marker in momenti diversi e pubblicherebbero valori
        # diversi per lo stesso child_frame_id, creando un conflitto in TF.
        return f"{frame}_locked_teach"

    def _publish_locked_frame(self, frame: str, ts: TransformStamped):
        """Pubblica il lock come TF statica, cosi' e' visibile in RViz (utile
        per capire a occhio dove il sistema crede che sia il marker) ed e'
        utilizzabile direttamente come source_frame di una lookup."""
        out = TransformStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.child_frame_id  = self._locked_frame_name(frame)
        out.transform       = ts.transform
        self._static_tf_broadcaster.sendTransform(out)

    def _try_lock(self, frame: str):
        """Tenta l'aggancio di `frame`. Ritorna (ok, messaggio)."""
        ts, msg = self._locker.acquire(frame)
        if ts is None:
            return False, msg
        self._publish_locked_frame(frame, ts)
        return True, msg

    def _auto_lock_tick(self):
        for frame in self._trackable_frames:
            if self._locker.is_locked(frame):
                continue
            # Sonda leggera prima di impegnarsi nella raccolta degli N campioni:
            # se il marker non e' inquadrato non ha senso bloccare il nodo per
            # mezzo secondo a ogni tick.
            probe, _ = self._locker.sample_once(frame)
            if probe is None:
                continue
            ok, msg = self._try_lock(frame)
            if not ok:
                continue
            self.get_logger().info(msg)
            self.get_logger().info(
                f"Tutte le pose salvate relative a '{frame}' useranno questa "
                f"stima, anche quando il marker non sara' piu' inquadrato. "
                f"Se sposti l'oggetto, chiama /relock_marker."
            )

    def relock_marker_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Ri-acquisisce l'aggancio da dove si trova il braccio adesso. Da usare
        quando l'oggetto viene spostato, o per agganciare da una posa scelta
        apposta invece che da quella casuale in cui si trovava il braccio
        all'avvio del nodo.

        L'aggancio precedente NON viene rilasciato prima del tentativo: se il
        nuovo campionamento fallisce (tipicamente perche' ci si e' avvicinati
        sotto la distanza minima) si resterebbe SENZA aggancio, e il timer
        automatico ne riprenderebbe uno in silenzio dalla prima posa utile -
        cioe' esattamente la misura non ripetibile che si stava cercando di
        evitare. `MarkerLocker.acquire()` sovrascrive solo in caso di successo,
        quindi un relock fallito lascia intatto quello buono di prima."""
        messages = []
        all_ok = True
        for frame in self._trackable_frames:
            had_lock = self._locker.is_locked(frame)
            ok, msg = self._try_lock(frame)
            if not ok and had_lock:
                msg += (f" - aggancio precedente MANTENUTO: "
                        f"{self._locker.info(frame)}")
            all_ok = all_ok and ok
            messages.append(msg)
            (self.get_logger().info if ok else self.get_logger().warn)(msg)

        response.success = all_ok
        response.message = " | ".join(messages)
        return response

    # ------------------------------------------------------------------
    # I/O file JSON (Invariati per garantire l'accumulo nello stesso file)
    # ------------------------------------------------------------------

    def _load_poses(self):
        try:
            if os.path.exists(self.json_file_path):
                with open(self.json_file_path, 'r') as f:
                    self.all_poses = json.load(f)
                self.get_logger().info(
                    f"Loaded {len(self.all_poses)} entries. Nuovi salvataggi saranno accodati."
                )
        except Exception as e:
            self.get_logger().error(f"Failed to load poses: {e}")

    def _save_poses_to_file(self):
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
            with open(self.json_file_path, 'w') as f:
                json.dump(self.all_poses, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to save to file: {e}")

    # ------------------------------------------------------------------
    # Callback: salva posa robot (Accumula nel file principale)
    # ------------------------------------------------------------------

    def save_pose_callback(self, request: SavePose.Request, response: SavePose.Response):
        target_frame = self.end_effector_frame
        source_frame = ""
        pose_type    = ""

        if request.save_mode == 0:
            pose_type    = "absolute"
            source_frame = self.base_frame
        elif request.save_mode == 1:
            pose_type    = "relative"
            source_frame = "camera_color_optical_frame"
        elif request.save_mode == 2:
            pose_type    = "relative"
            source_frame = request.reference_frame
            if not source_frame:
                response.success = False
                response.message = "ERROR: save_mode=2 richiede reference_frame."
                self.get_logger().error(response.message)
                return response
        else:
            response.success = False
            response.message = f"ERROR: save_mode={request.save_mode} non valido."
            self.get_logger().error(response.message)
            return response

        try:
            now = rclpy.time.Time()
            lock_note = ""

            if source_frame in self._trackable_frames:
                # Il marker NON viene riletto qui: si usa sempre l'aggancio, in
                # modo che tutte le pose della sessione siano riferite alla
                # stessa misura di ^B T_M. Se il marker non e' mai stato
                # agganciato, l'errore e' esplicito - il salvataggio silenzioso
                # su una stima improvvisata e' esattamente cio' che vogliamo
                # evitare.
                if not self._locker.is_locked(source_frame):
                    ok, msg = self._try_lock(source_frame)
                    if not ok:
                        response.success = False
                        response.message = (
                            f"Impossibile salvare relativo a '{source_frame}': "
                            f"{msg}. Inquadra il marker da almeno "
                            f"{self.marker_min_reliable_distance_m:.2f} m "
                            f"(l'aggancio e' automatico) e riprova."
                        )
                        self.get_logger().error(response.message)
                        return response
                    self.get_logger().info(msg)

                ts = self.tf_buffer.lookup_transform(
                    self._locked_frame_name(source_frame), target_frame, now,
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )
                lock_note = f" [aggancio: {self._locker.info(source_frame)}]"
            else:
                ts = self.tf_buffer.lookup_transform(
                    source_frame, target_frame, now,
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )

            t = ts.transform.translation
            r = ts.transform.rotation

            pose_data = {
                "task_type":    "move",
                "task_name":    request.task_name,
                "type":         pose_type,
                "source_frame": source_frame,
                "target_frame": target_frame,
                "transform": {
                    "translation": {"x": t.x, "y": t.y, "z": t.z},
                    "rotation":    {"x": r.x, "y": r.y, "z": r.z, "w": r.w}
                }
            }

            self.all_poses.append(pose_data)
            self._save_poses_to_file()

            response.success = True
            response.message = (
                f"Saved '{request.task_name}' "
                f"({pose_type}: {target_frame} w.r.t. {source_frame}). "
                f"Total entries: {len(self.all_poses)}.{lock_note}"
            )
            self.get_logger().info(response.message)

        except TransformException as ex:
            response.success = False
            response.message = f"TF error: {ex}"
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # Callback: salva azione gripper (Accumula nel file principale)
    # ------------------------------------------------------------------

    def save_gripper_action_callback(
        self, request: GripperSrv.Request, response: GripperSrv.Response
    ):
        command      = request.command.strip().lower()
        gripper_type = request.gripper_type.strip().lower() \
                       if request.gripper_type else "auto"
        position     = float(request.position)

        valid_commands = ("open", "close", "move")
        if command not in valid_commands:
            response.success = False
            response.message = f"Comando '{command}' non valido. Usa: {valid_commands}."
            self.get_logger().error(response.message)
            return response

        if command == "move":
            if not (0.0 <= position <= 1.0):
                response.success = False
                response.message = f"position={position:.3f} fuori range [0.0, 1.0] per 'move'."
                self.get_logger().error(response.message)
                return response

        gripper_data = {
            "task_type":    "gripper",
            "command":      command,
            "position":     position,   
            "gripper_type": gripper_type
        }

        self.all_poses.append(gripper_data)
        self._save_poses_to_file()

        pos_info = f"position={position:.3f}" if command == "move" else command
        response.success = True
        response.message = (
            f"Saved gripper action: {pos_info} ({gripper_type}). "
            f"Total entries: {len(self.all_poses)}."
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Callback: azzera tutto MA salva un backup cronologico prima
    # ------------------------------------------------------------------

    def clear_poses_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        try:
            # Se ci sono delle pose accumulate, prima di cancellarle facciamo un backup storico
            if self.all_poses:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_filename = f"backup_poses_{timestamp}.json"
                backup_path = os.path.join(self.task_result_dir, backup_filename)
                
                # Copia il file pieno corrente nel file di backup storico
                if os.path.exists(self.json_file_path):
                    shutil.copyfile(self.json_file_path, backup_path)
                    msg_backup = f"Backup creato con successo: {backup_filename}. "
                else:
                    msg_backup = "Nessun file su disco da backuppare. "
            else:
                msg_backup = "Lista già vuota. "

            # Ora svuota l'array e azzera il file principale
            self.all_poses = []
            with open(self.json_file_path, 'w') as f:
                json.dump([], f)
                
            response.success = True
            response.message = f"{msg_backup}Tutte le voci correnti sono state azzerate in robot_poses_ws.json."
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to clear/backup: {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

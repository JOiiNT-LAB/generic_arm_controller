import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
from std_srvs.srv._trigger import Trigger_Response
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController

import json
import os
import threading
import time
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose_stamped

from generic_arm_interfaces.srv import GripperCommand as GripperSrv

from generic_arm_controller import teach_mode
from generic_arm_controller.marker_lock import MarkerLocker


class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started.')

        # Frame in cui ik_trajectory_node calcola l'IK (stesso significato di
        # base_frame per fk_node - riusa il campo del profilo robot invece di
        # duplicarlo come "target_frame").
        self.declare_parameter('base_frame', 'base_link')
        self.target_frame_for_ik = self.get_parameter('base_frame').value

        # ------------------------------------------------------------------
        # Aggancio ("lock") del marker: guardo una volta, poi vado alla cieca
        # ------------------------------------------------------------------
        # marker_lock_mode:
        #   'once' (default) - la posa del marker rispetto alla base viene
        #       misurata UNA VOLTA, al primo waypoint della sequenza che ne ha
        #       bisogno, e poi riusata identica per tutti i waypoint successivi.
        #       Il marker puo' uscire dall'inquadratura senza conseguenze: e'
        #       esattamente cio' che succede durante l'avvicinamento finale.
        #       E' il comportamento corretto per un pick: rileggere il marker a
        #       ogni waypoint sposta il bersaglio tra un passo e l'altro, perche'
        #       con la camera sulla flangia l'errore residuo di calibrazione
        #       mano-occhio si proietta su ^B T_M in modo dipendente dal punto di
        #       vista - e i punti di vista dei vari waypoint sono diversi.
        #       Con il lock, la geometria relativa insegnata (pre-grasp -> grasp)
        #       viene riprodotta esattamente anche in presenza di un offset
        #       globale sulla stima del marker.
        #   'live' - comportamento storico: rilettura a ogni waypoint con
        #       fallback sull'ultima lettura buona. Mantenuto per confronto/debug.
        #
        # Il lock viene rilasciato all'inizio di OGNI sequenza: tra un'esecuzione
        # e l'altra l'oggetto puo' essere stato spostato, quindi va sempre
        # ri-osservato almeno una volta.
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_min_reliable_distance_m', 0.20)
        self.declare_parameter('marker_max_tf_age_s', 0.5)
        self.declare_parameter('marker_lock_samples', 5)
        self.declare_parameter('marker_lock_mode', 'once')
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.marker_min_reliable_distance_m = self.get_parameter(
            'marker_min_reliable_distance_m').value
        self.marker_max_tf_age_s = self.get_parameter('marker_max_tf_age_s').value
        self.marker_lock_samples = self.get_parameter('marker_lock_samples').value
        self.marker_lock_mode = str(self.get_parameter('marker_lock_mode').value).lower()
        if self.marker_lock_mode not in ('once', 'live'):
            self.get_logger().warn(
                f"marker_lock_mode='{self.marker_lock_mode}' non valido: uso 'once'."
            )
            self.marker_lock_mode = 'once'

        self.reentrant_callback_group = ReentrantCallbackGroup()

        # execute_saved_tasks_callback gira su ReentrantCallbackGroup (necessario per gli
        # altri servizi di questo nodo), ma una sequenza che pilota il robot fisico NON va
        # mai eseguita due volte in parallelo (es. utente che ripete "execute tasks" in chat
        # prima che la sequenza precedente finisca) - i comandi si sovrapporrebbero sullo
        # stesso robot. Lock non bloccante: una seconda richiesta concorrente viene
        # rifiutata subito invece di partire in parallelo.
        self._exec_lock = threading.Lock()

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publisher IK
        self.target_pose_publisher = self.create_publisher(
            PoseStamped, '/target_cartesian_pose', 10
        )

        # Servizio di avvio sequenza
        self.execute_task_service = self.create_service(
            Trigger,
            'execute_saved_tasks',
            self.execute_saved_tasks_callback,
            callback_group=self.reentrant_callback_group
        )

        # QoS feedback IK
        self.qos_profile_ik_feedback = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscription feedback IK
        self.ik_result_subscription = self.create_subscription(
            Bool,
            '/ik_action_result',
            self.ik_result_callback,
            self.qos_profile_ik_feedback,
            callback_group=self.reentrant_callback_group
        )
        self.ik_reason_subscription = self.create_subscription(
            String,
            '/ik_action_result_reason',
            self.ik_reason_callback,
            self.qos_profile_ik_feedback,
            callback_group=self.reentrant_callback_group
        )

        # Client GripperManager
        self.gripper_client = self.create_client(
            GripperSrv,
            '/gripper/command',
            callback_group=self.reentrant_callback_group
        )

        # ------------------------------------------------------------------
        # Teach mode (hand-guiding): /set_teach_mode deactivates the profile's
        # teach_mode_controller so the arm can be guided by hand, and
        # reactivates it before replaying. The state is latched on /teach_mode
        # and execute_saved_tasks refuses to run while it is on. An empty
        # teach_mode_controller (default) means the robot has no teach mode.
        # Pure logic in teach_mode.py.
        # ------------------------------------------------------------------
        self.declare_parameter('teach_mode_controller', '')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.teach_mode_controller = str(self.get_parameter('teach_mode_controller').value)
        controller_manager = str(self.get_parameter('controller_manager').value).rstrip('/')
        self._list_controllers_client = self.create_client(
            ListControllers, f'{controller_manager}/list_controllers',
            callback_group=self.reentrant_callback_group
        )
        self._switch_controller_client = self.create_client(
            SwitchController, f'{controller_manager}/switch_controller',
            callback_group=self.reentrant_callback_group
        )
        self.controller_manager_timeout = 5.0
        self._teach_mode_active = False
        self._teach_mode_pub = self.create_publisher(
            Bool, 'teach_mode',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.teach_mode_service = self.create_service(
            SetBool, 'set_teach_mode', self.set_teach_mode_callback,
            callback_group=self.reentrant_callback_group
        )
        # The initial state is published once the controller manager answers.
        self._teach_mode_init_timer = self.create_timer(
            2.0, self._init_teach_mode_state,
            callback_group=self.reentrant_callback_group
        )

        # Stato interno
        self.ik_action_result_received = False
        self.ik_action_success         = False
        self.ik_action_reason          = ""
        self._last_task_failure_reason = ""
        self.saved_poses               = []
        self.ik_completion_timeout     = 20.0
        self.gripper_timeout           = 10.0

        # Frame per cui vale la logica di aggancio - non ha senso per un
        # source_frame "assoluto" tipo base_frame, solo per frame rilevati dalla
        # camera. Indicizzato per nome: piu' marker (es. "marker_26",
        # "marker_28") hanno ognuno il proprio lock indipendente.
        self._trackable_frames = ("aruco_marker",)
        self._locker = MarkerLocker(
            node=self,
            tf_buffer=self.tf_buffer,
            base_frame=self.target_frame_for_ik,
            camera_optical_frame=self.camera_optical_frame,
            min_distance_m=self.marker_min_reliable_distance_m,
            max_age_s=self.marker_max_tf_age_s,
            samples=self.marker_lock_samples,
        )
        # Cache usata solo in modalita' 'live' (comportamento storico).
        self._live_cache = {}

        # ------------------------------------------------------------------
        # CORREZIONE QUI: Puntiamo allo stesso identico file cumulativo
        # ------------------------------------------------------------------
        home_dir = os.path.expanduser("~")
        self.json_file_path_ws = os.path.join(
            home_dir, 'ros2_ws/src/task_result/robot_poses_ws.json'
        )
        self.get_logger().info(f"Target Pose file per esecuzione: {self.json_file_path_ws}")
        self.get_logger().info(f"marker_lock_mode = '{self.marker_lock_mode}'")

    # ------------------------------------------------------------------
    # Feedback IK
    # ------------------------------------------------------------------

    def ik_reason_callback(self, msg: String):
        self.ik_action_reason = msg.data

    def ik_result_callback(self, msg: Bool):
        self.ik_action_result_received = True
        self.ik_action_success         = msg.data
        if self.ik_action_success:
            self.get_logger().info("IK completed successfully.")
        else:
            self.get_logger().warn(f"IK failed: {self.ik_action_reason or 'unknown reason'}.")

    # ------------------------------------------------------------------
    # Caricamento JSON
    # ------------------------------------------------------------------

    def load_poses_from_file(self) -> bool:
        if not os.path.exists(self.json_file_path_ws):
            self.get_logger().error(f"File non trovato: {self.json_file_path_ws}")
            self.saved_poses = []
            return False
        try:
            with open(self.json_file_path_ws, 'r') as f:
                self.saved_poses = json.load(f)
            if not self.saved_poses:
                self.get_logger().warn("File vuoto.")
                return False
            self.get_logger().info(
                f"Caricati {len(self.saved_poses)} entries."
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Errore caricamento: {e}")
            self.saved_poses = []
            return False

    # ------------------------------------------------------------------
    # Esecuzione task gripper
    # ------------------------------------------------------------------

    def _execute_gripper_task(self, task_data: dict) -> bool:
        command      = task_data.get('command', '')
        position     = float(task_data.get('position', 0.0))
        gripper_type = task_data.get('gripper_type', 'auto')
        self._last_task_failure_reason = ""

        if not self.gripper_client.service_is_ready():
            self.get_logger().warn("GripperManager non pronto, attendo 5 s...")
            if not self.gripper_client.wait_for_service(timeout_sec=5.0):
                self._last_task_failure_reason = "GripperManager unreachable"
                self.get_logger().error("GripperManager non raggiungibile. Salto.")
                return False

        req          = GripperSrv.Request()
        req.command  = command
        req.position = position
        if hasattr(req, 'gripper_type'):
            req.gripper_type = gripper_type

        pos_info = (
            f"position={position:.3f}" if command == "move" else command
        )
        self.get_logger().info(
            f"Gripper → {pos_info} ({gripper_type})"
        )

        future = self.gripper_client.call_async(req)
        start  = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > self.gripper_timeout:
                self._last_task_failure_reason = f"Gripper timeout ({self.gripper_timeout:.1f}s)"
                self.get_logger().warn(
                    f"Timeout gripper ({self.gripper_timeout:.1f} s)."
                )
                return False

        result = future.result()
        if result is None:
            self._last_task_failure_reason = "Empty gripper response"
            self.get_logger().error("Risposta gripper nulla.")
            return False

        if result.success:
            self.get_logger().info(f"Gripper OK: {result.message}")
        else:
            self._last_task_failure_reason = result.message or "Gripper failed (unknown reason)"
            self.get_logger().warn(f"Gripper FAIL: {result.message}")
        return result.success

    # ------------------------------------------------------------------
    # Aggancio marker
    # ------------------------------------------------------------------

    def _locked_frame_name(self, frame: str) -> str:
        # Nome distinto da quello del nodo di salvataggio ("_locked_teach"): i
        # due nodi agganciano il marker in momenti diversi e pubblicherebbero
        # valori diversi per lo stesso child_frame_id, creando un conflitto TF.
        return f"{frame}_locked_run"

    def _publish_locked_frame(self, frame: str, ts: TransformStamped):
        """Pubblica il lock come TF statica: serve solo a poter vedere in RViz
        dove la sequenza in corso crede che sia il marker. La trasformazione dei
        waypoint usa direttamente `ts`, non questo frame."""
        out = TransformStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame_for_ik
        out.child_frame_id  = self._locked_frame_name(frame)
        out.transform       = ts.transform
        self._static_tf_broadcaster.sendTransform(out)

    def _resolve_marker_transform_once(self, source_frame: str, task_name: str):
        """Modalita' 'once': ritorna il lock, acquisendolo alla prima chiamata
        della sequenza. Dopodiche' il marker non viene piu' letto."""
        transform, msg, is_new = self._locker.get_or_acquire(source_frame)
        if transform is None:
            self.get_logger().error(
                f"{msg}. Impossibile eseguire '{task_name}': il marker deve "
                f"essere inquadrato almeno una volta, all'inizio della sequenza. "
                f"Metti come primo waypoint una posa assoluta da cui il marker "
                f"sia ben visibile. Salto."
            )
            return None
        if is_new:
            self.get_logger().info(msg)
            self._publish_locked_frame(source_frame, transform)
            self.get_logger().info(
                f"Da qui in poi tutti i waypoint relativi a '{source_frame}' "
                f"useranno questa stima: il marker puo' uscire "
                f"dall'inquadratura senza conseguenze."
            )
        else:
            self.get_logger().info(
                f"'{task_name}': uso l'aggancio su '{source_frame}' gia' "
                f"acquisito in questa sequenza (marker non riletto)."
            )
        return transform

    def _resolve_marker_transform_live(self, source_frame: str, task_name: str):
        """Modalita' 'live' (storica): rilettura a ogni waypoint, con fallback
        sull'ultima lettura buona di questa sequenza."""
        need_fallback = False
        fallback_reason = ""
        transform = None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame_for_ik, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            probe, reason = self._locker.sample_once(source_frame)
            if probe is None:
                need_fallback = True
                fallback_reason = reason
            else:
                self._live_cache[source_frame] = transform
        except TransformException as ex:
            need_fallback = True
            fallback_reason = (
                f"TF error ({source_frame}->{self.target_frame_for_ik}): {ex}"
            )

        if need_fallback:
            transform = self._live_cache.get(source_frame)
            if transform is None:
                self.get_logger().error(
                    f"{fallback_reason} per '{task_name}'. Nessuna lettura "
                    f"precedente disponibile per '{source_frame}'. Salto."
                )
                return None
            self.get_logger().warn(
                f"{fallback_reason} per '{task_name}'. Uso l'ultima lettura "
                f"buona di '{source_frame}' di questa sequenza."
            )
        return transform

    # ------------------------------------------------------------------
    # Esecuzione task movimento (IK)
    # ------------------------------------------------------------------

    def _execute_move_task(self, task_data: dict) -> bool:
        target_frame_for_ik = self.target_frame_for_ik
        task_name = task_data.get('task_name', 'Unnamed')
        self._last_task_failure_reason = ""

        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()

        try:
            source_frame = task_data['source_frame']
            pose.header.frame_id = source_frame
        except KeyError:
            self._last_task_failure_reason = "Missing source_frame"
            self.get_logger().error(f"Manca source_frame per '{task_name}'. Salto.")
            return False

        try:
            tr  = task_data['transform']['translation']
            rot = task_data['transform']['rotation']
            pose.pose.position.x    = tr['x']
            pose.pose.position.y    = tr['y']
            pose.pose.position.z    = tr['z']
            pose.pose.orientation.x = rot['x']
            pose.pose.orientation.y = rot['y']
            pose.pose.orientation.z = rot['z']
            pose.pose.orientation.w = rot['w']
        except KeyError as e:
            self._last_task_failure_reason = f"Incomplete transform: missing {e}"
            self.get_logger().error(
                f"Transform incompleto per '{task_name}': manca {e}. Salto."
            )
            return False

        # Trasformazione TF se necessaria
        if source_frame != target_frame_for_ik:
            if source_frame in self._trackable_frames:
                if self.marker_lock_mode == 'once':
                    transform = self._resolve_marker_transform_once(
                        source_frame, task_name)
                else:
                    transform = self._resolve_marker_transform_live(
                        source_frame, task_name)
            else:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame_for_ik, source_frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                except TransformException as ex:
                    self._last_task_failure_reason = f"TF error: {ex}"
                    self.get_logger().error(
                        f"TF error ({source_frame}->{target_frame_for_ik}) per "
                        f"'{task_name}': {ex}. Salto."
                    )
                    return False

            if transform is None:
                self._last_task_failure_reason = (
                    f"Could not resolve reference '{source_frame}' "
                    f"(marker not locked or TF unavailable)"
                )
                return False
            pose = do_transform_pose_stamped(pose, transform)

        # Publish + attesa feedback
        self.ik_action_result_received = False
        self.ik_action_success         = False
        self.ik_action_reason          = ""
        self.target_pose_publisher.publish(pose)
        self.get_logger().info(
            f"Pose '{task_name}' pubblicata. Attendo IK..."
        )

        start = self.get_clock().now()
        while not self.ik_action_result_received:
            time.sleep(0.01)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.ik_completion_timeout:
                self._last_task_failure_reason = (
                    f"IK timeout ({self.ik_completion_timeout:.1f}s) - "
                    f"ik_trajectory_node did not respond"
                )
                self.get_logger().warn(
                    f"Timeout IK ({self.ik_completion_timeout:.1f} s) per '{task_name}'."
                )
                return False

        if not self.ik_action_success:
            self._last_task_failure_reason = self.ik_action_reason or "IK failed (unknown reason)"
            self.get_logger().warn(f"IK fallito per '{task_name}': {self._last_task_failure_reason}")
            return False

        self.get_logger().info(f"IK OK per '{task_name}'.")
        return True

    # ------------------------------------------------------------------
    # Callback servizio: esegue la sequenza completa con pause
    # ------------------------------------------------------------------

    def execute_saved_tasks_callback(
        self, request: Trigger.Request, response: Trigger_Response
    ):
        if not self._exec_lock.acquire(blocking=False):
            response.success = False
            response.message = "Una sequenza è già in esecuzione - attendi che finisca."
            self.get_logger().warn(response.message)
            return response
        try:
            if self._teach_mode_active:
                response.success = False
                response.message = ("Teach mode is ON: turn it off (/set_teach_mode false) "
                                    "before executing the sequence.")
                self.get_logger().warn(response.message)
                return response
            return self._execute_saved_tasks(response)
        finally:
            self._exec_lock.release()

    # ------------------------------------------------------------------
    # Teach mode (hand-guiding)
    # ------------------------------------------------------------------

    def set_teach_mode_callback(self, request, response):
        enable = bool(request.data)
        # Holding the execution lock during the switch also keeps a sequence
        # from starting halfway through it.
        locked = self._exec_lock.acquire(blocking=False)
        try:
            error = teach_mode.check_request(self.teach_mode_controller, executing=not locked)
            if error is None:
                error = self._switch_teach_mode(enable)
        finally:
            if locked:
                self._exec_lock.release()
        response.success = error is None
        response.message = error or teach_mode.describe(enable)
        if error:
            self.get_logger().warn(f"Teach mode request refused: {error}")
        else:
            self.get_logger().info(response.message)
        return response

    def _switch_teach_mode(self, enable):
        """Switch the controller for teach mode `enable`; return an error or None."""
        state, error = self._read_teach_controller_state()
        if error:
            return error
        activate, deactivate, error = teach_mode.plan_switch(
            enable, self.teach_mode_controller, state)
        if error:
            return error
        if activate or deactivate:
            req = SwitchController.Request()
            req.activate_controllers = activate
            req.deactivate_controllers = deactivate
            req.strictness = SwitchController.Request.STRICT
            req.activate_asap = True
            req.timeout = Duration(sec=2)
            res = self._call_service(
                self._switch_controller_client, req, self.controller_manager_timeout)
            if res is None:
                return "The controller manager did not answer the switch request."
            if not res.ok:
                return (f"The controller manager refused to switch "
                        f"'{self.teach_mode_controller}'.")
        self._teach_mode_init_timer.cancel()
        self._set_teach_mode_state(enable)
        return None

    def _read_teach_controller_state(self):
        """Return (state, error) of teach_mode_controller; state None if not loaded."""
        res = self._call_service(
            self._list_controllers_client, ListControllers.Request(),
            self.controller_manager_timeout)
        if res is None:
            return None, "The controller manager is not reachable (list_controllers)."
        return teach_mode.controller_state(res.controller, self.teach_mode_controller), None

    def _init_teach_mode_state(self):
        """Publish the initial teach mode state; retried until it is known."""
        if self.teach_mode_controller:
            state, error = self._read_teach_controller_state()
            if error or state is None:
                return  # controller manager or controller not up yet
            self._set_teach_mode_state(state == teach_mode.INACTIVE)
        else:
            self._set_teach_mode_state(False)
        self._teach_mode_init_timer.cancel()

    def _set_teach_mode_state(self, active):
        self._teach_mode_active = active
        self._teach_mode_pub.publish(Bool(data=active))

    def _call_service(self, client, request, timeout):
        """Call a service from a reentrant callback; None if unavailable or on timeout."""
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                return None
        return future.result()

    def _execute_saved_tasks(self, response: Trigger_Response):
        self.get_logger().info("Avvio sequenza task.")

        # Ogni nuova sequenza riparte senza aggancio: il marker (o altro frame)
        # potrebbe essersi spostato dall'ultima esecuzione, quindi va ri-osservato
        # almeno una volta. Da li' in poi resta agganciato per tutta la sequenza.
        self._locker.release()
        self._live_cache = {}

        if not self.load_poses_from_file():
            response.success = False
            response.message = "Impossibile caricare le pose."
            return response

        success_count = 0

        for i, task_data in enumerate(self.saved_poses):
            task_type = task_data.get('task_type', 'move')
            label     = task_data.get(
                'task_name', task_data.get('command', f'Task {i+1}')
            )

            self.get_logger().info(
                f"--- [{i+1}/{len(self.saved_poses)}] "
                f"type={task_type}  label={label} ---"
            )

            self._last_task_failure_reason = ""
            if task_type == 'gripper':
                ok = self._execute_gripper_task(task_data)
                default_sleep = 1.5
            elif task_type == 'move':
                ok = self._execute_move_task(task_data)
                # L'action FollowJointTrajectory riporta "succeeded" alla fine
                # del tempo di traiettoria pianificato (trajectory_duration),
                # non quando il braccio reale si è davvero fermato: appena
                # finita l'inerzia residua/oscillazione può ancora essere in
                # movimento. Una piccola pausa qui evita di lanciare il target
                # successivo mentre il braccio non si è ancora stabilizzato.
                # Override per singolo waypoint con "sleep_time" nel json.
                default_sleep = 0.4
            else:
                self._last_task_failure_reason = f"Unrecognized task_type '{task_type}'"
                self.get_logger().warn(
                    f"task_type '{task_type}' non riconosciuto. Salto."
                )
                ok = False
                default_sleep = 0.0

            if ok:
                success_count += 1

                sleep_time = float(task_data.get('sleep_time', default_sleep))
                if sleep_time > 0.0:
                    self.get_logger().info(f"Attesa di stabilità per {sleep_time} secondi...")
                    time.sleep(sleep_time)
            else:
                # Stop at the first failure: the next steps were taught assuming this one
                # succeeded (e.g. a gripper close after a move that never happened).
                reason = self._last_task_failure_reason or "unknown reason"
                skipped = len(self.saved_poses) - (i + 1)
                self.get_logger().error(
                    f"Sequence stopped at step {i+1}/{len(self.saved_poses)} "
                    f"('{label}'): {reason}. {skipped} remaining steps not executed."
                )
                response.success = False
                response.message = (
                    f"Stopped at step {i+1}/{len(self.saved_poses)} ('{label}'): {reason}. "
                    f"{success_count} steps OK before it, {skipped} not executed."
                )
                return response

        self.get_logger().info(f"Sequence completed: {success_count} steps OK.")
        response.success = True
        response.message = (
            f"Executed {len(self.saved_poses)} tasks: {success_count} OK, 0 failed."
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown.')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

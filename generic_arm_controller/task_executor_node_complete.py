"""
Replay the saved sequence on /execute_saved_tasks, and serve /set_teach_mode.

The sequence is robot_poses_ws.json. The replay rules (order, stop at the first failure,
pauses, result messages) live in sequence.py; this node performs each step: a 'move' is
published on /target_cartesian_pose and waits for ik_trajectory_node's /ik_action_result,
a 'gripper' step calls /gripper/command. Marker-relative poses go through marker_lock.py,
teach mode through teach_mode.py. See the package README.
"""

import os
import threading
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from generic_arm_controller import pose_store
from generic_arm_controller import sequence
from generic_arm_controller import teach_mode
from generic_arm_controller.marker_lock import MarkerLocker
from generic_arm_interfaces.srv import GripperCommand as GripperSrv
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from std_srvs.srv._trigger import Trigger_Response
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Frames detected by the camera, which go through the marker lock. Indexed by name, so
# several markers would each get their own lock.
TRACKABLE_FRAMES = ('aruco_marker',)
IK_COMPLETION_TIMEOUT_S = 20.0
GRIPPER_TIMEOUT_S = 10.0
CONTROLLER_MANAGER_TIMEOUT_S = 5.0


class TaskExecutorNode(Node):

    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started.')
        self._cb_group = ReentrantCallbackGroup()
        # The services run on a reentrant group, but a sequence driving the robot must never
        # run twice at once (e.g. "execute tasks" repeated in the chat before the previous
        # run ends): a second request is refused, not queued.
        self._exec_lock = threading.Lock()

        self._read_parameters()
        self._create_ik_and_gripper_interfaces()
        self._create_marker_lock()
        self._create_teach_mode_interfaces()

        self.json_file_path_ws = pose_store.DEFAULT_PATH
        self.get_logger().info(f"Target Pose file per esecuzione: {self.json_file_path_ws}")
        self.get_logger().info(f"marker_lock_mode = '{self.marker_lock_mode}'")

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _read_parameters(self):
        # The frame ik_trajectory_node computes the IK in (the profile's base_frame).
        self.declare_parameter('base_frame', 'base_link')
        self.target_frame_for_ik = self.get_parameter('base_frame').value

        # Marker lock, see README "Marker lock": 'once' (default) measures the marker once
        # per sequence and reuses it, so the taught pre-grasp -> grasp geometry is replayed
        # exactly even with a hand-eye calibration offset; 'live' re-reads it at every
        # waypoint (old behaviour, kept for comparison).
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

        # Teach mode: an empty teach_mode_controller (default) means no teach mode.
        self.declare_parameter('teach_mode_controller', '')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.teach_mode_controller = str(self.get_parameter('teach_mode_controller').value)
        self._controller_manager = str(
            self.get_parameter('controller_manager').value).rstrip('/')

    def _create_ik_and_gripper_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.target_pose_publisher = self.create_publisher(
            PoseStamped, '/target_cartesian_pose', 10)
        self.execute_task_service = self.create_service(
            Trigger, 'execute_saved_tasks', self.execute_saved_tasks_callback,
            callback_group=self._cb_group)

        ik_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=1,
                            durability=DurabilityPolicy.VOLATILE)
        self.ik_result_subscription = self.create_subscription(
            Bool, '/ik_action_result', self.ik_result_callback, ik_qos,
            callback_group=self._cb_group)
        self.ik_reason_subscription = self.create_subscription(
            String, '/ik_action_result_reason', self.ik_reason_callback, ik_qos,
            callback_group=self._cb_group)
        self.ik_action_result_received = False
        self.ik_action_success = False
        self.ik_action_reason = ""

        self.gripper_client = self.create_client(
            GripperSrv, '/gripper/command', callback_group=self._cb_group)
        # Why the last step failed, for the sequence result message.
        self._last_task_failure_reason = ""

    def _create_marker_lock(self):
        self._locker = MarkerLocker(
            node=self,
            tf_buffer=self.tf_buffer,
            base_frame=self.target_frame_for_ik,
            camera_optical_frame=self.camera_optical_frame,
            min_distance_m=self.marker_min_reliable_distance_m,
            max_age_s=self.marker_max_tf_age_s,
            samples=self.marker_lock_samples,
        )
        self._live_cache = {}  # last good marker reading, 'live' mode only

    def _create_teach_mode_interfaces(self):
        # /set_teach_mode deactivates teach_mode_controller so the arm can be guided by hand,
        # and reactivates it; the state is latched on /teach_mode and execute_saved_tasks
        # refuses to run while it is on. Pure logic in teach_mode.py.
        self._list_controllers_client = self.create_client(
            ListControllers, f'{self._controller_manager}/list_controllers',
            callback_group=self._cb_group)
        self._switch_controller_client = self.create_client(
            SwitchController, f'{self._controller_manager}/switch_controller',
            callback_group=self._cb_group)
        self._teach_mode_active = False
        self._teach_mode_pub = self.create_publisher(
            Bool, 'teach_mode',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.teach_mode_service = self.create_service(
            SetBool, 'set_teach_mode', self.set_teach_mode_callback,
            callback_group=self._cb_group)
        # The initial state is published once the controller manager answers.
        self._teach_mode_init_timer = self.create_timer(
            2.0, self._init_teach_mode_state, callback_group=self._cb_group)

    # ------------------------------------------------------------------
    # Feedback IK
    # ------------------------------------------------------------------

    def ik_reason_callback(self, msg: String):
        self.ik_action_reason = msg.data

    def ik_result_callback(self, msg: Bool):
        self.ik_action_result_received = True
        self.ik_action_success = msg.data
        if self.ik_action_success:
            self.get_logger().info("IK completed successfully.")
        else:
            self.get_logger().warn(f"IK failed: {self.ik_action_reason or 'unknown reason'}.")

    # ------------------------------------------------------------------
    # Sequence
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

    def _execute_saved_tasks(self, response: Trigger_Response):
        self.get_logger().info("Avvio sequenza task.")
        # Every sequence starts without a lock: the object may have moved since the last
        # run, so the marker is observed again at least once.
        self._locker.release()
        self._live_cache = {}

        entries = self._load_sequence()
        if entries is None:
            response.success = False
            response.message = "Impossibile caricare le pose."
            return response

        response.success, response.message = sequence.run(
            entries, self._run_step, self.get_logger())
        return response

    def _load_sequence(self):
        """Return the saved entries, or None (logged) if there is nothing to run."""
        path = self.json_file_path_ws
        if not os.path.exists(path):
            self.get_logger().error(f"File non trovato: {path}")
            return None
        try:
            entries = pose_store.load(path)
        except (ValueError, OSError) as e:
            self.get_logger().error(f"Errore caricamento: {e}")
            return None
        if not entries:
            self.get_logger().warn("File vuoto.")
            return None
        self.get_logger().info(f"Caricati {len(entries)} entries.")
        return entries

    def _run_step(self, task_type, entry):
        """One step for sequence.run: return (ok, failure_reason)."""
        self._last_task_failure_reason = ""
        if task_type == 'gripper':
            ok = self._execute_gripper_task(entry)
        else:
            ok = self._execute_move_task(entry)
        return ok, self._last_task_failure_reason

    # ------------------------------------------------------------------
    # Gripper step
    # ------------------------------------------------------------------

    def _execute_gripper_task(self, task_data: dict) -> bool:
        command = task_data.get('command', '')
        position = float(task_data.get('position', 0.0))
        gripper_type = task_data.get('gripper_type', 'auto')

        if not self.gripper_client.service_is_ready():
            self.get_logger().warn("GripperManager non pronto, attendo 5 s...")
            if not self.gripper_client.wait_for_service(timeout_sec=5.0):
                self._last_task_failure_reason = "GripperManager unreachable"
                self.get_logger().error("GripperManager non raggiungibile. Salto.")
                return False

        req = GripperSrv.Request()
        req.command = command
        req.position = position
        req.gripper_type = gripper_type
        pos_info = f"position={position:.3f}" if command == "move" else command
        self.get_logger().info(f"Gripper → {pos_info} ({gripper_type})")

        future = self.gripper_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > GRIPPER_TIMEOUT_S:
                self._last_task_failure_reason = f"Gripper timeout ({GRIPPER_TIMEOUT_S:.1f}s)"
                self.get_logger().warn(f"Timeout gripper ({GRIPPER_TIMEOUT_S:.1f} s).")
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
    # Move step
    # ------------------------------------------------------------------

    def _execute_move_task(self, task_data: dict) -> bool:
        task_name = task_data.get('task_name', 'Unnamed')
        try:
            source_frame, position, quaternion = sequence.pose_from_entry(task_data)
        except ValueError as e:
            self._last_task_failure_reason = str(e)
            self.get_logger().error(f"'{task_name}': {e}. Salto.")
            return False

        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.header.frame_id = source_frame
        p, q = pose.pose.position, pose.pose.orientation
        p.x, p.y, p.z = position
        q.x, q.y, q.z, q.w = quaternion

        if source_frame != self.target_frame_for_ik:
            pose = self._to_ik_frame(pose, source_frame, task_name)
            if pose is None:
                return False
        return self._send_and_wait_for_ik(pose, task_name)

    def _to_ik_frame(self, pose, source_frame, task_name):
        """Express pose in the IK frame; None (failure reason set) if it can't be."""
        if source_frame in TRACKABLE_FRAMES:
            if self.marker_lock_mode == 'once':
                transform = self._resolve_marker_transform_once(source_frame, task_name)
            else:
                transform = self._resolve_marker_transform_live(source_frame, task_name)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame_for_ik, source_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
            except TransformException as ex:
                self._last_task_failure_reason = f"TF error: {ex}"
                self.get_logger().error(
                    f"TF error ({source_frame}->{self.target_frame_for_ik}) per "
                    f"'{task_name}': {ex}. Salto.")
                return None

        if transform is None:
            self._last_task_failure_reason = (
                f"Could not resolve reference '{source_frame}' "
                f"(marker not locked or TF unavailable)")
            return None
        return do_transform_pose_stamped(pose, transform)

    def _send_and_wait_for_ik(self, pose, task_name):
        """Publish the target and wait for ik_trajectory_node's result."""
        self.ik_action_result_received = False
        self.ik_action_success = False
        self.ik_action_reason = ""
        self.target_pose_publisher.publish(pose)
        self.get_logger().info(f"Pose '{task_name}' pubblicata. Attendo IK...")

        start = self.get_clock().now()
        while not self.ik_action_result_received:
            time.sleep(0.01)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > IK_COMPLETION_TIMEOUT_S:
                self._last_task_failure_reason = (
                    f"IK timeout ({IK_COMPLETION_TIMEOUT_S:.1f}s) - "
                    f"ik_trajectory_node did not respond")
                self.get_logger().warn(
                    f"Timeout IK ({IK_COMPLETION_TIMEOUT_S:.1f} s) per '{task_name}'.")
                return False

        if not self.ik_action_success:
            self._last_task_failure_reason = self.ik_action_reason or "IK failed (unknown reason)"
            self.get_logger().warn(
                f"IK fallito per '{task_name}': {self._last_task_failure_reason}")
            return False
        self.get_logger().info(f"IK OK per '{task_name}'.")
        return True

    # ------------------------------------------------------------------
    # Marker lock
    # ------------------------------------------------------------------

    def _locked_frame_name(self, frame: str) -> str:
        # Distinct from the saving node's "_locked_teach": the two nodes lock the marker at
        # different times, and the same child_frame_id with two values would conflict in TF.
        return f"{frame}_locked_run"

    def _publish_locked_frame(self, frame: str, ts: TransformStamped):
        """
        Publish the lock as a static TF, only for RViz.

        It shows where the running sequence thinks the marker is: the waypoints are
        transformed with `ts` directly.
        """
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame_for_ik
        out.child_frame_id = self._locked_frame_name(frame)
        out.transform = ts.transform
        self._static_tf_broadcaster.sendTransform(out)

    def _resolve_marker_transform_once(self, source_frame: str, task_name: str):
        """'once' mode: the lock, taken at the first call of the sequence, then reused."""
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
        """'live' mode: re-read at every waypoint, falling back on the last good reading."""
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
                self._switch_controller_client, req, CONTROLLER_MANAGER_TIMEOUT_S)
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
            CONTROLLER_MANAGER_TIMEOUT_S)
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

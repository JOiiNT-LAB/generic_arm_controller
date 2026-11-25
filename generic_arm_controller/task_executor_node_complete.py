"""
Replay the saved sequence on /execute_saved_tasks, and serve /set_teach_mode.

The sequence is robot_poses_ws.json (pose_store.py). The replay rules (order, stop at the
first failure, pauses, result messages) are in sequence.py; this node performs each step:
a 'move' is published on /target_cartesian_pose and waits for ik_trajectory_node's
/ik_action_result, a 'gripper' step calls /gripper/command. Marker-relative waypoints are
resolved by marker_resolution.py, teach mode is teach_mode_service.py. See the README.
"""

import os
import threading
import time

from generic_arm_controller import pose_store
from generic_arm_controller import sequence
from generic_arm_controller.marker_resolution import MarkerResolver, MODES
from generic_arm_controller.service_call import call_with_timeout
from generic_arm_controller.teach_mode_service import TeachModeService
from generic_arm_interfaces.srv import GripperCommand as GripperSrv
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Frames detected by the camera, resolved through the marker lock.
TRACKABLE_FRAMES = ('aruco_marker',)
IK_COMPLETION_TIMEOUT_S = 20.0
GRIPPER_TIMEOUT_S = 10.0


class TaskExecutorNode(Node):

    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started.')
        cb_group = ReentrantCallbackGroup()
        # The services run on a reentrant group, but a sequence driving the robot must never
        # run twice at once (e.g. "execute tasks" repeated in the chat before the previous
        # run ends): a second request is refused, not queued.
        self._exec_lock = threading.Lock()

        # The frame ik_trajectory_node computes the IK in (the profile's base_frame).
        self.target_frame_for_ik = self._param('base_frame', 'base_link')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        marker_lock_mode = str(self._param('marker_lock_mode', 'once')).lower()
        if marker_lock_mode not in MODES:
            self.get_logger().warn(
                f"marker_lock_mode='{marker_lock_mode}' non valido: uso 'once'.")
            marker_lock_mode = 'once'
        self._markers = MarkerResolver(
            self, self.tf_buffer, self.target_frame_for_ik, marker_lock_mode,
            camera_optical_frame=self._param(
                'camera_optical_frame', 'camera_color_optical_frame'),
            min_distance_m=self._param('marker_min_reliable_distance_m', 0.20),
            max_age_s=self._param('marker_max_tf_age_s', 0.5),
            samples=self._param('marker_lock_samples', 5))

        self._teach_mode = TeachModeService(
            self,
            controller=str(self._param('teach_mode_controller', '')),
            controller_manager=str(self._param(
                'controller_manager', '/controller_manager')).rstrip('/'),
            callback_group=cb_group,
            exec_lock=self._exec_lock)

        self.target_pose_publisher = self.create_publisher(
            PoseStamped, '/target_cartesian_pose', 10)
        ik_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=1,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Bool, '/ik_action_result', self.ik_result_callback,
                                 ik_qos, callback_group=cb_group)
        self.create_subscription(String, '/ik_action_result_reason', self.ik_reason_callback,
                                 ik_qos, callback_group=cb_group)
        self.ik_action_result_received = False
        self.ik_action_success = False
        self.ik_action_reason = ""

        self.gripper_client = self.create_client(
            GripperSrv, '/gripper/command', callback_group=cb_group)
        self.create_service(Trigger, 'execute_saved_tasks', self.execute_saved_tasks_callback,
                            callback_group=cb_group)
        # Why the last step failed, for the sequence result message.
        self._last_task_failure_reason = ""

        self.json_file_path_ws = pose_store.DEFAULT_PATH
        self.get_logger().info(f"Target Pose file per esecuzione: {self.json_file_path_ws}")
        self.get_logger().info(f"marker_lock_mode = '{marker_lock_mode}'")

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

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
            if self._teach_mode.active:
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
        self._markers.reset()

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

        done, result = call_with_timeout(self.gripper_client, req, GRIPPER_TIMEOUT_S)
        if not done:
            self._last_task_failure_reason = f"Gripper timeout ({GRIPPER_TIMEOUT_S:.1f}s)"
            self.get_logger().warn(f"Timeout gripper ({GRIPPER_TIMEOUT_S:.1f} s).")
            return False
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
            transform = self._markers.resolve(source_frame, task_name)
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

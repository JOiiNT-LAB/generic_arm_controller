import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response

import json
import os
import time
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from ur_msgs.srv import GripperCommand as GripperSrv


class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started.')

        self.reentrant_callback_group = ReentrantCallbackGroup()

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # Client GripperManager
        self.gripper_client = self.create_client(
            GripperSrv,
            '/gripper/command',
            callback_group=self.reentrant_callback_group
        )

        # Stato interno
        self.ik_action_result_received = False
        self.ik_action_success         = False
        self.saved_poses               = []
        self.ik_completion_timeout     = 20.0
        self.gripper_timeout           = 10.0

        # ------------------------------------------------------------------
        # CORREZIONE QUI: Puntiamo allo stesso identico file cumulativo
        # ------------------------------------------------------------------
        home_dir = os.path.expanduser("~")
        self.json_file_path_ws = os.path.join(
            home_dir, 'ros2_ws/src/task_result/robot_poses_ws.json'
        )
        self.get_logger().info(f"Target Pose file per esecuzione: {self.json_file_path_ws}")

    # ------------------------------------------------------------------
    # Feedback IK
    # ------------------------------------------------------------------

    def ik_result_callback(self, msg: Bool):
        self.ik_action_result_received = True
        self.ik_action_success         = msg.data
        if self.ik_action_success:
            self.get_logger().info("IK completed successfully.")
        else:
            self.get_logger().warn("IK failed.")

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

        if not self.gripper_client.service_is_ready():
            self.get_logger().warn("GripperManager non pronto, attendo 5 s...")
            if not self.gripper_client.wait_for_service(timeout_sec=5.0):
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
                self.get_logger().warn(
                    f"Timeout gripper ({self.gripper_timeout:.1f} s)."
                )
                return False

        result = future.result()
        if result is None:
            self.get_logger().error("Risposta gripper nulla.")
            return False

        if result.success:
            self.get_logger().info(f"Gripper OK: {result.message}")
        else:
            self.get_logger().warn(f"Gripper FAIL: {result.message}")
        return result.success

    # ------------------------------------------------------------------
    # Esecuzione task movimento (IK)
    # ------------------------------------------------------------------

    def _execute_move_task(self, task_data: dict, target_frame_for_ik: str) -> bool:
        task_name = task_data.get('task_name', 'Unnamed')

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        try:
            source_frame = task_data['source_frame']
            pose.header.frame_id = source_frame
        except KeyError:
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
            self.get_logger().error(
                f"Transform incompleto per '{task_name}': manca {e}. Salto."
            )
            return False

        # Trasformazione TF se necessaria
        if source_frame != target_frame_for_ik:
            try:
                pose = self.tf_buffer.transform(
                    pose, target_frame_for_ik,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except TransformException as ex:
                self.get_logger().error(
                    f"TF error per '{task_name}': {ex}. Salto."
                )
                return False

        # Publish + attesa feedback
        self.ik_action_result_received = False
        self.ik_action_success         = False
        self.target_pose_publisher.publish(pose)
        self.get_logger().info(
            f"Pose '{task_name}' pubblicata. Attendo IK..."
        )

        start = self.get_clock().now()
        while not self.ik_action_result_received:
            time.sleep(0.01)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.ik_completion_timeout:
                self.get_logger().warn(
                    f"Timeout IK ({self.ik_completion_timeout:.1f} s) per '{task_name}'."
                )
                return False

        if not self.ik_action_success:
            self.get_logger().warn(f"IK fallito per '{task_name}'.")
            return False

        self.get_logger().info(f"IK OK per '{task_name}'.")
        return True

    # ------------------------------------------------------------------
    # Callback servizio: esegue la sequenza completa con pause
    # ------------------------------------------------------------------

    def execute_saved_tasks_callback(
        self, request: Trigger.Request, response: Trigger_Response
    ):
        self.get_logger().info("Avvio sequenza task.")

        if not self.load_poses_from_file():
            response.success = False
            response.message = "Impossibile caricare le pose."
            return response

        target_frame_for_ik = 'base_link'
        success_count = 0
        fail_count    = 0

        for i, task_data in enumerate(self.saved_poses):
            task_type = task_data.get('task_type', 'move')
            label     = task_data.get(
                'task_name', task_data.get('command', f'Task {i+1}')
            )

            self.get_logger().info(
                f"--- [{i+1}/{len(self.saved_poses)}] "
                f"type={task_type}  label={label} ---"
            )

            if task_type == 'gripper':
                ok = self._execute_gripper_task(task_data)
                default_sleep = 1.5 
            elif task_type == 'move':
                ok = self._execute_move_task(task_data, target_frame_for_ik)
                default_sleep = 0.0
            else:
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
                fail_count += 1

        self.get_logger().info(
            f"Sequenza completata: OK={success_count} FAIL={fail_count}"
        )
        response.success = (fail_count == 0)
        response.message = (
            f"Executed {len(self.saved_poses)} tasks: "
            f"{success_count} OK, {fail_count} failed."
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
    
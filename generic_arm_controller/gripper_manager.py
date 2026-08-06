#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import GripperCommand as GripperSrv
from control_msgs.action import GripperCommand as GripperAction

# FIX 3: protected QB import
try:
    from qb_softhand_industry_srvs.srv import SetCommand
    QB_AVAILABLE = True
except ImportError:
    QB_AVAILABLE = False


RG2_MAX_WIDTH_M  = 0.085
SOFTHAND_MIN_POS = 0
SOFTHAND_MAX_POS = 3500

# Kinematic limits Robotiq 2F-85 in Gazebo/ROS 2 Control
ROBOTIQ_MIN_POS  = 0.0  # Open
ROBOTIQ_MAX_POS  = 0.8  # Fully Closed

# Franka Hand: apertura pinza in metri (a differenza del Robotiq, qui la convenzione
# non è invertita: 0.0 = chiuso, larghezza massima = aperto)
FRANKA_HAND_MIN_WIDTH = 0.0    # Closed
FRANKA_HAND_MAX_WIDTH = 0.08   # Open


class GripperManager(Node):

    def __init__(self):
        super().__init__('gripper_manager')

        # FIX DEADLOCK: ReentrantCallbackGroup + MultiThreadedExecutor
        self.cb_group = ReentrantCallbackGroup()

        # --- RG2 publisher ---
        self.rg2_pub = self.create_publisher(
            JointTrajectory,
            '/finger_width_trajectory_controller/joint_trajectory',
            10,
        )

        # --- SoftHand client (only if package available) ---
        self.softhand_client = None
        if QB_AVAILABLE:
            self.softhand_client = self.create_client(
                SetCommand,
                '/qb_softhand_industry_communication_handler/set_command',
                callback_group=self.cb_group,
            )
            self.get_logger().info('SoftHand client created.')
        else:
            self.get_logger().warn(
                'qb_softhand_industry_srvs not found. SoftHand disabled.'
            )

        # --- Robotiq Action Client ---
        self.robotiq_client = ActionClient(
            self,
            GripperAction,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb_group
        )
        self.get_logger().info('Robotiq Action Client initialized.')

        # --- Franka Hand Action Client (stesso action type del Robotiq) ---
        self.franka_hand_client = ActionClient(
            self,
            GripperAction,
            '/franka_gripper/gripper_action',
            callback_group=self.cb_group
        )
        self.get_logger().info('Franka Hand Action Client initialized.')

        # --- ROS Service ---
        self.service = self.create_service(
            GripperSrv,
            '/gripper/command',
            self.gripper_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('GripperManager READY')

    # =========================================================
    # MAIN CALLBACK
    # =========================================================

    def gripper_callback(self, request, response):
        cmd          = request.command.strip().lower()
        gripper_type = request.gripper_type.strip().lower() \
                       if request.gripper_type else 'auto'

        position = self._resolve_position(cmd, request.position)

        if position is None:
            response.success = False
            response.message = (
                f"Invalid command '{cmd}'. "
                "Use 'open', 'close', or 'move' with position in [0.0, 1.0]."
            )
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f"Gripper → cmd={cmd}, position={position:.3f}, type={gripper_type}"
        )

        if gripper_type == 'auto':
            gripper_type = self._auto_select()

        if gripper_type == 'rg2':
            return self._handle_rg2(position, response)
        elif gripper_type == 'softhand':
            return self._handle_softhand(position, response)
        elif gripper_type == 'robotiq':
            return self._handle_robotiq(position, response)
        elif gripper_type == 'franka_hand':
            return self._handle_franka_hand(position, response)
        else:
            response.success = False
            response.message = f"Unknown gripper type: '{gripper_type}'"
            self.get_logger().error(response.message)
            return response

    # =========================================================
    # POSITION RESOLUTION
    # =========================================================

    def _resolve_position(self, cmd: str, raw_position: float):
        if cmd == 'open':
            return 1.0  # Logic convention: 1.0 = Fully Open
        elif cmd == 'close':
            return 0.0  # Logic convention: 0.0 = Fully Closed
        elif cmd == 'move':
            return max(0.0, min(1.0, float(raw_position)))
        return None

    # =========================================================
    # RG2
    # =========================================================

    def _handle_rg2(self, position: float, response):
        target_width = RG2_MAX_WIDTH_M * position

        msg = JointTrajectory()
        msg.joint_names = ['finger_width']

        point = JointTrajectoryPoint()
        point.positions = [target_width]
        point.time_from_start.sec = 1
        msg.points = [point]

        self.rg2_pub.publish(msg)

        self.get_logger().info(
            f"[RG2] position={position:.3f} → {target_width*1000:.1f} mm"
        )
        response.success = True
        response.message = (
            f"RG2: position={position:.3f} ({target_width*1000:.1f} mm)"
        )
        return response

    # =========================================================
    # SOFTHAND
    # =========================================================

    def _handle_softhand(self, position: float, response):
        if not QB_AVAILABLE or self.softhand_client is None:
            response.success = False
            response.message = 'SoftHand not available (QB package not installed).'
            self.get_logger().error(response.message)
            return response

        if not self.softhand_client.service_is_ready():
            response.success = False
            response.message = 'SoftHand service not available.'
            self.get_logger().error(response.message)
            return response

        softhand_cmd = int(
            SOFTHAND_MAX_POS - position * (SOFTHAND_MAX_POS - SOFTHAND_MIN_POS)
        )

        req = self.softhand_client.srv_type.Request()
        req.max_repeats      = 1
        req.set_commands     = True
        req.position_command = softhand_cmd

        future = self.softhand_client.call_async(req)

        import time
        timeout = 5.0
        start   = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = f'SoftHand timeout ({timeout:.1f} s).'
                self.get_logger().warn(response.message)
                return response

        result = future.result()

        self.get_logger().info(
            f'[SoftHand] position={position:.3f} → cmd={softhand_cmd}'
        )
        response.success = True
        response.message = (
            result.message
            if result
            else f'SoftHand: position={position:.3f} (cmd={softhand_cmd})'
        )
        return response

    # =========================================================
    # ROBOTIQ
    # =========================================================

    def _handle_robotiq(self, position: float, response):
        if not self.robotiq_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'Robotiq action server not available!'
            self.get_logger().error(response.message)
            return response

        robotiq_pos = ROBOTIQ_MAX_POS - position * (ROBOTIQ_MAX_POS - ROBOTIQ_MIN_POS)

        goal_msg = GripperAction.Goal()
        goal_msg.command.position = robotiq_pos
        goal_msg.command.max_effort = 100.0  # Grasping force

        self.get_logger().info(f'[Robotiq] Sending goal position: {robotiq_pos:.3f}')
        
        send_goal_future = self.robotiq_client.send_goal_async(goal_msg)

        import time
        timeout = 5.0
        start = time.time()

        while not send_goal_future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = 'Robotiq goal acceptance timeout.'
                return response

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            response.success = False
            response.message = 'Robotiq goal rejected by action server.'
            return response

        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = 'Robotiq movement completion timeout.'
                return response

        self.get_logger().info(f'[Robotiq] Movement completed successfully.')
        response.success = True
        response.message = f'Robotiq set to {robotiq_pos:.2f} (Logical input: {position:.2f})'
        return response

    # =========================================================
    # FRANKA HAND
    # =========================================================

    def _handle_franka_hand(self, position: float, response):
        if not self.franka_hand_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'Franka Hand action server not available!'
            self.get_logger().error(response.message)
            return response

        franka_width = FRANKA_HAND_MIN_WIDTH + position * (
            FRANKA_HAND_MAX_WIDTH - FRANKA_HAND_MIN_WIDTH
        )

        goal_msg = GripperAction.Goal()
        goal_msg.command.position = franka_width
        goal_msg.command.max_effort = 20.0  # Grasping force (N)

        self.get_logger().info(f'[FrankaHand] Sending goal width: {franka_width:.3f} m')

        send_goal_future = self.franka_hand_client.send_goal_async(goal_msg)

        import time
        timeout = 5.0
        start = time.time()

        while not send_goal_future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = 'Franka Hand goal acceptance timeout.'
                return response

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            response.success = False
            response.message = 'Franka Hand goal rejected by action server.'
            return response

        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = 'Franka Hand movement completion timeout.'
                return response

        self.get_logger().info('[FrankaHand] Movement completed successfully.')
        response.success = True
        response.message = f'FrankaHand set to {franka_width:.3f} m (Logical input: {position:.2f})'
        return response

    # =========================================================
    # AUTO SELECT
    # =========================================================

    def _auto_select(self) -> str:
        if self.robotiq_client.server_is_ready():
            return 'robotiq'
        if self.franka_hand_client.server_is_ready():
            return 'franka_hand'
        return 'rg2'


def main():
    rclpy.init()
    node = GripperManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
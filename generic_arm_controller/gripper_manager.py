#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from generic_arm_interfaces.srv import GripperCommand as GripperSrv
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

# Il command_interface "position" di ign_ros2_control (Fortress) non applica alcuna
# coppia per questo giunto: la conversione interna position->velocity risulta un
# no-op nella fisica, mentre comandare la velocity interface funziona correttamente.
# Guidiamo quindi il giunto in Gazebo con un ciclo di controllo in velocità.
ROBOTIQ_JOINT_NAME       = 'robotiq_85_left_knuckle_joint'
ROBOTIQ_COMMANDS_TOPIC   = '/robotiq_gripper_controller/commands'
ROBOTIQ_MOVE_VELOCITY    = 0.4   # rad/s (< limite giunto 0.5 rad/s)
ROBOTIQ_POS_TOLERANCE    = 0.01  # rad
ROBOTIQ_MOVE_TIMEOUT     = 5.0   # s

# Franka Hand: apertura pinza in metri (a differenza del Robotiq, qui la convenzione
# non è invertita: 0.0 = chiuso, larghezza massima = aperto)
FRANKA_HAND_MIN_WIDTH = 0.0    # Closed
FRANKA_HAND_MAX_WIDTH = 0.08   # Open


class GripperManager(Node):

    def __init__(self):
        super().__init__('gripper_manager')

        # Gripper di default per il robot corrente (dal profilo robot), usato quando la
        # richiesta di servizio non specifica gripper_type (o lo lascia 'auto').
        self.declare_parameter('default_gripper_type', 'auto')
        self.default_gripper_type = self.get_parameter('default_gripper_type').value

        # FIX DEADLOCK: ReentrantCallbackGroup + MultiThreadedExecutor
        self.cb_group = ReentrantCallbackGroup()

        # --- RG2 publisher ---
        self.rg2_pub = self.create_publisher(
            JointTrajectory,
            '/finger_width_trajectory_controller/joint_trajectory',
            10,
        )

        # --- Robotiq velocity command publisher + joint state feedback ---
        self.robotiq_cmd_pub = self.create_publisher(
            Float64MultiArray,
            ROBOTIQ_COMMANDS_TOPIC,
            10,
        )
        self.robotiq_current_pos = 0.0
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.cb_group,
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
    # JOINT STATE FEEDBACK (usato dal controllo in velocità del Robotiq)
    # =========================================================

    def _joint_state_callback(self, msg):
        if ROBOTIQ_JOINT_NAME in msg.name:
            self.robotiq_current_pos = msg.position[msg.name.index(ROBOTIQ_JOINT_NAME)]

    # =========================================================
    # MAIN CALLBACK
    # =========================================================

    def gripper_callback(self, request, response):
        cmd = request.command.strip().lower()

        position = self._resolve_position(cmd)

        if position is None:
            response.success = False
            response.message = (
                f"Invalid command '{cmd}'. Use 'open' or 'close'."
            )
            self.get_logger().error(response.message)
            return response

        gripper_type = (
            self.default_gripper_type
            if self.default_gripper_type != 'auto'
            else self._auto_select()
        )

        self.get_logger().info(
            f"Gripper → cmd={cmd}, position={position:.3f}, type={gripper_type}"
        )

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

    def _resolve_position(self, cmd: str):
        if cmd == 'open':
            return 1.0  # Logic convention: 1.0 = Fully Open
        elif cmd == 'close':
            return 0.0  # Logic convention: 0.0 = Fully Closed
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
        target_pos = ROBOTIQ_MAX_POS - position * (ROBOTIQ_MAX_POS - ROBOTIQ_MIN_POS)

        self.get_logger().info(f'[Robotiq] Driving to position: {target_pos:.3f}')

        start = time.time()
        while time.time() - start < ROBOTIQ_MOVE_TIMEOUT:
            error = target_pos - self.robotiq_current_pos
            if abs(error) <= ROBOTIQ_POS_TOLERANCE:
                break
            vel = ROBOTIQ_MOVE_VELOCITY if error > 0 else -ROBOTIQ_MOVE_VELOCITY
            self.robotiq_cmd_pub.publish(Float64MultiArray(data=[vel]))
            time.sleep(0.02)

        self.robotiq_cmd_pub.publish(Float64MultiArray(data=[0.0]))

        error = target_pos - self.robotiq_current_pos
        response.success = abs(error) <= ROBOTIQ_POS_TOLERANCE
        response.message = (
            f'Robotiq set to {target_pos:.2f} (Logical input: {position:.2f}), '
            f'reached {self.robotiq_current_pos:.3f}'
        )
        if not response.success:
            self.get_logger().warn(f'[Robotiq] Timeout, {response.message}')
        else:
            self.get_logger().info(f'[Robotiq] Movement completed: {response.message}')
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
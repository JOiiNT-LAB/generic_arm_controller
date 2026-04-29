import rclpy
from rclpy.node import Node

from ur_msgs.srv import GripperCommand as GripperSrv
from qb_softhand_industry_srvs.srv import SetCommand

from control_msgs.msg import GripperCommand as RG2Msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperManager(Node):

    def __init__(self):
        super().__init__('gripper_manager')

        # =========================
        # RG2 publisher
        # =========================
        self.rg2_pub = self.create_publisher(
            JointTrajectory,
            '/finger_width_trajectory_controller/joint_trajectory',
            10
        )
        # =========================
        # SoftHand client
        # =========================

        self.softhand_client = self.create_client(
            SetCommand,
            '/qb_softhand_industry_communication_handler/set_command'
        )

        # =========================
        # UNICO SERVICE ROS
        # =========================
        self.service = self.create_service(
            GripperSrv,
            '/gripper/command',
            self.gripper_callback
        )

        self.get_logger().info("GripperManager READY")

    # =========================================================
    # MAIN CALLBACK
    # =========================================================
    def gripper_callback(self, request, response):

        cmd = request.command.lower()

        # opzionale: puoi estendere il srv con gripper_type
        gripper_type = getattr(request, "gripper_type", "auto").lower()

        self.get_logger().info(
            f"Gripper request → cmd={cmd}, type={gripper_type}"
        )

        # =========================
        # AUTO ROUTING
        # =========================
        if gripper_type == "auto":
            gripper_type = self._auto_select()

        # =========================
        # RG2
        # =========================
        if gripper_type == "rg2":
            return self._handle_rg2(cmd, response)

        # =========================
        # SOFTHAND
        # =========================
        elif gripper_type == "softhand":
            return self._handle_softhand(cmd, response)

        else:
            response.success = False
            response.message = f"Unknown gripper type: {gripper_type}"
            return response
        

    def _handle_rg2(self, cmd, response):

        msg = JointTrajectory()
        msg.joint_names = ['finger_width']

        point = JointTrajectoryPoint()

        # -------------------------
        # NORMALIZED POSITION
        # -------------------------
        position = None

        cmd = cmd.lower()

        if cmd == "open":
            position = 1.0
        elif cmd == "close":
            position = 0.0
        else:
            response.success = False
            response.message = f"Invalid command: {cmd}"
            return response

        # clamp
        position = max(0.0, min(1.0, position))

        # mapping reale RG2
        point.positions = [0.085 * position]

        point.time_from_start.sec = 1
        msg.points = [point]

        self.rg2_pub.publish(msg)

        self.get_logger().info(f"[RG2] position={position:.2f}")

        response.success = True
        response.message = f"RG2 executed at {position:.2f}"
        return response
    
    def _handle_softhand(self, cmd, response):

        if not self.softhand_client.service_is_ready():
            response.success = False
            response.message = "SoftHand service not available"
            return response

        POSITION_MAP = {
            "open": 0,
            "close": 3500
        }

        cmd = cmd.lower()

        if cmd not in POSITION_MAP:
            response.success = False
            response.message = f"Invalid command: {cmd}"
            return response

        position = POSITION_MAP[cmd]

        req = self.softhand_client.srv_type.Request()
        req.max_repeats = 1
        req.set_commands = True
        req.position_command = position

        future = self.softhand_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()

        response.success = True
        response.message = result.message if result else f"SoftHand {cmd} executed"
        return response
    # =========================================================
    # SIMPLE POLICY
    # =========================================================
    def _auto_select(self):
        """
        Default policy:
        - prefer RG2 if system is active
        - fallback SoftHand
        """
        return "rg2"


def main():
    rclpy.init()
    node = GripperManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import pinocchio as pin

from generic_arm_controller.urdf_loader import build_pinocchio_urdf


class FKNode(Node):
    """
    ROS2 node for Forward Kinematics using Pinocchio.
    Fully configurable via ROS2 parameters — no hardcoded robot-specific values.

    Parameters
    ----------
    urdf_package       : str  — ROS package containing the URDF/xacro (default: 'ur_description')
    robot_type         : str  — Robot type passed to xacro, e.g. 'ur10e', 'ur5e' (default: 'ur10e')
    end_effector_frame : str  — Pinocchio frame name for the end-effector (default: 'tool0')
    base_frame         : str  — Frame ID used in the published PoseStamped header (default: 'base_link')
    joint_states_topic : str  — Input JointState topic (default: '/joint_states')
    fk_pose_topic      : str  — Output PoseStamped topic (default: '/fk_pose')
    """

    def __init__(self):
        super().__init__('fk_node')

        # ------------------------------------------------------------------ #
        # Declare parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('urdf_package',       'ur_description')
        self.declare_parameter('robot_type',         'ur10e')
        self.declare_parameter('end_effector_frame', 'tool0')
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('fk_pose_topic',      '/fk_pose')

        # ------------------------------------------------------------------ #
        # Read parameters
        # ------------------------------------------------------------------ #
        urdf_package       = self.get_parameter('urdf_package').value
        robot_type         = self.get_parameter('robot_type').value
        self.ee_frame_name = self.get_parameter('end_effector_frame').value
        self.base_frame    = self.get_parameter('base_frame').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        fk_pose_topic      = self.get_parameter('fk_pose_topic').value

        self.get_logger().info(
            f'Starting FK node | robot: {robot_type} | package: {urdf_package} | '
            f'EE frame: {self.ee_frame_name} | base frame: {self.base_frame}'
        )

        # ------------------------------------------------------------------ #
        # Build URDF and load Pinocchio model (shared utility)
        # ------------------------------------------------------------------ #
        urdf_path = build_pinocchio_urdf(
            urdf_package=urdf_package,
            robot_type=robot_type,
            logger=self.get_logger(),
        )

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.get_logger().info(f'Pinocchio model loaded: {self.model.nq} DOF')

        # ------------------------------------------------------------------ #
        # Identify controllable joints
        # ------------------------------------------------------------------ #
        self.joint_names = [
            self.model.names[self.model.joints[i].id]
            for i in range(1, self.model.nq + 1)
            if self.model.joints[i].nq > 0
        ]
        self.get_logger().info(f'Controllable joints: {self.joint_names}')

        # ------------------------------------------------------------------ #
        # Validate end-effector frame
        # ------------------------------------------------------------------ #
        if not self.model.existFrame(self.ee_frame_name):
            raise ValueError(
                f"End-effector frame '{self.ee_frame_name}' not found in model. "
                f"Check the 'end_effector_frame' parameter."
            )
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
        self.get_logger().info(
            f'End-effector frame: {self.ee_frame_name} (id={self.ee_frame_id})'
        )

        # ------------------------------------------------------------------ #
        # Joint configuration state
        # ------------------------------------------------------------------ #
        self.q = pin.neutral(self.model)

        # ------------------------------------------------------------------ #
        # ROS2 publisher and subscriber
        # ------------------------------------------------------------------ #
        self.fk_pose_publisher = self.create_publisher(PoseStamped, fk_pose_topic, 10)
        self.create_subscription(JointState, joint_states_topic, self.joint_callback, 10)

        self.get_logger().info(
            f'Subscribed to: {joint_states_topic} | Publishing to: {fk_pose_topic}'
        )

    # ---------------------------------------------------------------------- #
    # Callback
    # ---------------------------------------------------------------------- #

    def joint_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        for joint_name in self.joint_names:
            if joint_name in name_to_pos:
                jid   = self.model.getJointId(joint_name)
                q_idx = self.model.joints[jid].idx_q
                if self.model.joints[jid].nq == 1:
                    self.q[q_idx] = name_to_pos[joint_name]

        # Forward kinematics
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        try:
            ee_pose    = self.data.oMf[self.ee_frame_id]
            pos        = ee_pose.translation
            quaternion = pin.Quaternion(ee_pose.rotation).coeffs()  # [x, y, z, w]

            pose_msg                    = PoseStamped()
            pose_msg.header.stamp       = self.get_clock().now().to_msg()
            pose_msg.header.frame_id    = self.base_frame
            pose_msg.pose.position.x    = float(pos[0])
            pose_msg.pose.position.y    = float(pos[1])
            pose_msg.pose.position.z    = float(pos[2])
            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])

            self.fk_pose_publisher.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'Error computing FK: {e}')


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FKNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'FK node failed: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
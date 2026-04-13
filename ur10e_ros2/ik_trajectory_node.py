import math

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint

import pinocchio as pin

from ur10e_ros2.urdf_loader import build_pinocchio_urdf


class IKTrajectoryNode(Node):
    """
    ROS2 Node for Inverse Kinematics using Pinocchio.
    Fully configurable via ROS2 parameters — no hardcoded robot-specific values.

    Parameters
    ----------
    urdf_package        : str   — ROS package containing the URDF/xacro (default: 'ur_description')
    robot_type          : str   — Robot type passed to xacro, e.g. 'ur10e', 'ur5e' (default: 'ur10e')
    end_effector_frame  : str   — Pinocchio frame name for the end-effector (default: 'tool0')
    joint_names         : list  — Ordered joint names for the controller
    action_server_name  : str   — FollowJointTrajectory action server topic
    pose_topic          : str   — Input PoseStamped topic (default: '/target_cartesian_pose')
    joint_states_topic  : str   — JointState topic (default: '/joint_states')
    ik_result_topic     : str   — Output Bool topic for IK result (default: '/ik_action_result')
    ik_tolerance        : float — IK convergence tolerance (default: 0.001)
    ik_max_iter         : int   — IK max iterations (default: 4000)
    ik_dt               : float — IK integration time step (default: 0.1)
    ik_damping          : float — Damped least-squares damping factor (default: 0.05)
    trajectory_duration : float — Trajectory execution duration in seconds (default: 3.0)
    """

    def __init__(self):
        super().__init__('ik_trajectory_node')

        # ------------------------------------------------------------------ #
        # Declare parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('urdf_package', 'ur_description')
        self.declare_parameter('robot_type', 'ur10e')
        self.declare_parameter('end_effector_frame', 'tool0')
        self.declare_parameter('joint_names', [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ])
        self.declare_parameter(
            'action_server_name',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter('pose_topic',          '/target_cartesian_pose')
        self.declare_parameter('joint_states_topic',  '/joint_states')
        self.declare_parameter('ik_result_topic',     '/ik_action_result')
        self.declare_parameter('ik_tolerance',        0.001)
        self.declare_parameter('ik_max_iter',         4000)
        self.declare_parameter('ik_dt',               0.1)
        self.declare_parameter('ik_damping',          0.05)
        self.declare_parameter('trajectory_duration', 3.0)

        # ------------------------------------------------------------------ #
        # Read parameters
        # ------------------------------------------------------------------ #
        urdf_package       = self.get_parameter('urdf_package').value
        robot_type         = self.get_parameter('robot_type').value
        self.ee_frame_name = self.get_parameter('end_effector_frame').value
        self.joint_names   = list(self.get_parameter('joint_names').value)
        action_server_name = self.get_parameter('action_server_name').value
        pose_topic         = self.get_parameter('pose_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        ik_result_topic    = self.get_parameter('ik_result_topic').value

        self.tolerance           = self.get_parameter('ik_tolerance').value
        self.max_iter            = self.get_parameter('ik_max_iter').value
        self.dt                  = self.get_parameter('ik_dt').value
        self.damping             = self.get_parameter('ik_damping').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value

        self.get_logger().info(
            f'Starting IK node | robot: {robot_type} | package: {urdf_package} | '
            f'EE frame: {self.ee_frame_name} | action: {action_server_name}'
        )

        # ------------------------------------------------------------------ #
        # URDF — delegato al modulo condiviso urdf_loader
        # ------------------------------------------------------------------ #
        urdf_path = build_pinocchio_urdf(
            urdf_package=urdf_package,
            robot_type=robot_type,
            logger=self.get_logger(),
        )

        # ------------------------------------------------------------------ #
        # Pinocchio model
        # ------------------------------------------------------------------ #
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.get_logger().info(f'Pinocchio model loaded: {self.model.nq} DOF')

        self._validate_joints()

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
        # State
        # ------------------------------------------------------------------ #
        self.current_q          = pin.neutral(self.model)
        self.initial_q_received = False

        # ------------------------------------------------------------------ #
        # ROS2 interfaces
        # ------------------------------------------------------------------ #
        self.create_subscription(PoseStamped, pose_topic,         self.pose_callback,  10)
        self.create_subscription(JointState,  joint_states_topic, self.joint_callback, 10)
        self.get_logger().info(f'Subscribed to: {pose_topic}, {joint_states_topic}')

        self._action_client = ActionClient(self, FollowJointTrajectory, action_server_name)
        self.get_logger().info(f'Waiting for action server: {action_server_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready.')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.ik_result_publisher = self.create_publisher(Bool, ik_result_topic, qos)
        self.get_logger().info(f'Publishing IK results on: {ik_result_topic}')

    # ---------------------------------------------------------------------- #
    # Helpers
    # ---------------------------------------------------------------------- #

    def _validate_joints(self):
        model_joints = [
            self.model.names[self.model.joints[i].id]
            for i in range(1, self.model.nq + 1)
            if self.model.joints[i].nq > 0
        ]
        missing = [n for n in self.joint_names if n not in model_joints]
        if missing:
            raise ValueError(
                f"Joints not found in model: {missing}. "
                f"Available: {model_joints}. Fix the 'joint_names' parameter."
            )
        self.get_logger().info(f'Joint order confirmed: {self.joint_names}')

    def _get_joint_positions(self, q: np.ndarray):
        positions = []
        for name in self.joint_names:
            jid   = self.model.getJointId(name)
            q_idx = self.model.joints[jid].idx_q
            if q_idx == -1:
                self.get_logger().error(f'Invalid q index for joint {name}.')
                return None
            positions.append(q[q_idx])
        return positions

    # ---------------------------------------------------------------------- #
    # Callbacks
    # ---------------------------------------------------------------------- #

    def joint_callback(self, msg: JointState):
        if not self.initial_q_received:
            self.get_logger().info('Initial joint states received.')
            self.initial_q_received = True

        pos_map   = dict(zip(msg.name, msg.position))
        updated_q = pin.neutral(self.model)

        for name in self.joint_names:
            if name in pos_map:
                jid   = self.model.getJointId(name)
                q_idx = self.model.joints[jid].idx_q
                if q_idx != -1 and self.model.joints[jid].nq > 0:
                    updated_q[q_idx] = pos_map[name]

        self.current_q = updated_q

    def pose_callback(self, msg: PoseStamped):
        if not self.initial_q_received:
            self.get_logger().warn('Waiting for initial joint states.')
            return

        p = msg.pose
        self.get_logger().info(
            f'Target pose: pos=({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f})'
        )

        q_norm = math.sqrt(
            p.orientation.w ** 2 + p.orientation.x ** 2 +
            p.orientation.y ** 2 + p.orientation.z ** 2
        )
        if q_norm == 0:
            self.get_logger().error('Zero-norm quaternion — ignoring pose.')
            return

        rotation   = pin.Quaternion(
            p.orientation.w / q_norm,
            p.orientation.x / q_norm,
            p.orientation.y / q_norm,
            p.orientation.z / q_norm,
        ).matrix()
        translation = np.array([p.position.x, p.position.y, p.position.z])
        target_se3  = pin.SE3(rotation, translation)

        q_guess    = self.current_q.copy()
        found      = False
        error_norm = float('inf')

        for i in range(self.max_iter):
            pin.computeJointJacobians(self.model, self.data, q_guess)
            pin.updateFramePlacements(self.model, self.data)

            error      = pin.log6(self.data.oMf[self.ee_frame_id].inverse() * target_se3).vector
            error_norm = np.linalg.norm(error)

            if i % 100 == 0:
                self.get_logger().debug(f'IK iter {i}: error={error_norm:.6f}')

            if error_norm < self.tolerance:
                found = True
                self.get_logger().info(f'IK converged at iter {i}, error={error_norm:.6f}')
                break

            J = pin.getFrameJacobian(
                self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL
            )[:, :self.model.nq]

            try:
                U, S, Vt = np.linalg.svd(J, full_matrices=False)
                J_pinv   = Vt.T @ np.diag(S / (S ** 2 + self.damping)) @ U.T
            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f'SVD failed at iter {i}: {e}')
                continue

            q_guess = pin.integrate(self.model, q_guess, self.dt * (J_pinv @ error))
            q_guess = np.clip(q_guess, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        if found:
            self._send_trajectory(q_guess)
        else:
            self.get_logger().warn(
                f'IK failed after {self.max_iter} iterations, error={error_norm:.4f}'
            )
            out      = Bool()
            out.data = False
            self.ik_result_publisher.publish(out)

    # ---------------------------------------------------------------------- #
    # Trajectory
    # ---------------------------------------------------------------------- #

    def _send_trajectory(self, q_solution: np.ndarray):
        current_pos  = self._get_joint_positions(self.current_q)
        solution_pos = self._get_joint_positions(q_solution)
        if current_pos is None or solution_pos is None:
            return

        goal                        = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        p0                          = JointTrajectoryPoint()
        p0.positions                = current_pos
        p0.time_from_start.sec     = 0
        p0.time_from_start.nanosec = 0
        goal.trajectory.points.append(p0)

        dur_sec                     = int(self.trajectory_duration)
        p1                          = JointTrajectoryPoint()
        p1.positions                = solution_pos
        p1.time_from_start.sec     = dur_sec
        p1.time_from_start.nanosec = int((self.trajectory_duration - dur_sec) * 1e9)
        goal.trajectory.points.append(p1)

        self.get_logger().info('Sending trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    # ---------------------------------------------------------------------- #
    # Action callbacks
    # ---------------------------------------------------------------------- #

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return
        self.get_logger().info('Goal accepted.')
        self._get_result_future = handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        msg    = Bool()

        if status == rclpy.action.client.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Trajectory succeeded.')
            msg.data = True
        else:
            self.get_logger().warn(
                f'Trajectory failed — status={status}, error_code={result.error_code}'
            )
            msg.data = False

        self.ik_result_publisher.publish(msg)

    def _feedback_callback(self, feedback_msg):
        pass


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = IKTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
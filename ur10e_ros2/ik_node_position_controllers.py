import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray 
import pinocchio as pin
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import subprocess
import re
import math
from sensor_msgs.msg import JointState

class UR10eIKNode(Node):
    """
    ROS2 Node for performing Inverse Kinematics (IK) for a UR10e robot
    using the Pinocchio library. It subscribes to a target Cartesian pose,
    calculates the corresponding joint angles, and publishes them.
    """
    def __init__(self):
        super().__init__('ur10e_ik_node')

        self.get_logger().info('Initializing UR10e Inverse Kinematics Node...')

        # --- URDF Loading and Processing ---
        # This section handles finding the URDF, processing it with xacro,
        # and adjusting mesh paths for Pinocchio.
        try:
            ur_description_share_dir = get_package_share_directory('ur_description')
            urdf_xacro_path = os.path.join(ur_description_share_dir, 'urdf', 'ur.urdf.xacro')
            mesh_dir_path = os.path.join(ur_description_share_dir, 'meshes')
            temp_urdf_file_raw = "/tmp/ur10e_pinocchio_generated_raw.urdf"
            urdf_path_for_pinocchio = "/tmp/ur10e_pinocchio_final.urdf"

            # Command to run xacro to generate a clean URDF
            xacro_command = [
                'ros2', 'run', 'xacro', 'xacro', urdf_xacro_path,
                'ur_type:=ur10e',
                'name:=ur10e', 
                'transmission_hw_interface:=""',
                'sim_gazebo:=false',
                'sim_ignition:=false',
                'use_fake_hardware:=false',
                'headless_mode:=false'
            ]

            self.get_logger().info(f"Generating clean URDF using xacro: {' '.join(xacro_command)} > {temp_urdf_file_raw}")

            try:
                # Execute xacro command and save output to a temporary file
                with open(temp_urdf_file_raw, 'w') as f:
                    subprocess.run(xacro_command, check=True, stdout=f, stderr=subprocess.PIPE)
                self.get_logger().info(f"Raw URDF generated at: {temp_urdf_file_raw}")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to generate raw URDF with xacro. Stderr: {e.stderr.decode()}")
                raise
            except Exception as e:
                self.get_logger().error(f"Error during raw xacro generation: {e}")
                raise

            # Read the raw URDF content
            with open(temp_urdf_file_raw, 'r') as f_in:
                urdf_content = f_in.read()

            # Replace relative mesh paths with absolute paths for Pinocchio
            urdf_content = re.sub(
                r'filename="package://ur_description/meshes/ur10e/visual/([^"]+)"',
                lambda m: f'filename="{os.path.join(mesh_dir_path, "ur10e", "visual", m.group(1))}"',
                urdf_content
            )
            urdf_content = re.sub(
                r'filename="package://ur_description/meshes/ur10e/collision/([^"]+)"',
                lambda m: f'filename="{os.path.join(mesh_dir_path, "ur10e", "collision", m.group(1))}"',
                urdf_content
            )

            # Save the processed URDF to a final temporary file
            with open(urdf_path_for_pinocchio, 'w') as f_out:
                f_out.write(urdf_content)
            self.get_logger().info(f"URDF with absolute mesh paths saved to: {urdf_path_for_pinocchio}")

        except Exception as e:
            self.get_logger().error(f"Failed to get URDF/mesh paths or generate/process URDF: {e}")
            raise 

        # Ensure the final URDF file exists before proceeding
        if not os.path.exists(urdf_path_for_pinocchio):
            self.get_logger().error(f"Final URDF file not found at: {urdf_path_for_pinocchio}. Something went wrong with processing.")
            raise FileNotFoundError(f"Final URDF file missing: {urdf_path_for_pinocchio}")

        # --- Pinocchio Model Loading ---
        try:
            self.model = pin.buildModelFromUrdf(urdf_path_for_pinocchio)
            self.data = self.model.createData()
            self.get_logger().info('Robot model loaded successfully with Pinocchio.')
            self.get_logger().info(f"Model has {self.model.nq} degrees of freedom.")

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio model from {urdf_path_for_pinocchio}.")
            self.get_logger().error(f"Specific Pinocchio error: {e}")
            raise 

        # --- Joint Identification and Ordering ---
        self.controllable_joint_ids = []
        self.joint_names = []
        # Iterate through model joints to identify controllable (movable) joints
        for i in range(1, self.model.nq + 1): # Start from 1 as 0 is the universe joint
            joint_name = self.model.names[self.model.joints[i].id]
            if self.model.joints[i].nq > 0: # Check if the joint has degrees of freedom
                 self.controllable_joint_ids.append(self.model.joints[i].id)
                 self.joint_names.append(joint_name)

        # Define the expected order of UR10e joints for publishing
        # This matches the order you provided.
        expected_ur_joint_order = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
        # Check if the joints identified by Pinocchio match the expected UR order.
        # If not, reorder them to ensure consistency for publishing.
        if set(self.joint_names) != set(expected_ur_joint_order):
            self.get_logger().warn("Discrepancy in joint names or order from Pinocchio vs. expected UR joints.")
            self.get_logger().warn(f"Pinocchio joints: {self.joint_names}")
            self.get_logger().warn(f"Expected UR joints: {expected_ur_joint_order}")
            
            reordered_joint_ids = []
            reordered_joint_names = []
            # Create a map from joint name to its Pinocchio ID for efficient lookup
            joint_name_to_id_map = {name: self.model.getJointId(name) for name in self.joint_names} 
            
            # Populate reordered lists based on the expected order
            for name in expected_ur_joint_order:
                if name in joint_name_to_id_map:
                    reordered_joint_ids.append(joint_name_to_id_map[name])
                    reordered_joint_names.append(name)
                else:
                    self.get_logger().error(f"Expected joint '{name}' not found in Pinocchio model. IK may fail.")
            
            self.controllable_joint_ids = reordered_joint_ids
            self.joint_names = reordered_joint_names
            self.get_logger().info(f"Reordered controllable joints: {self.joint_names}")
        else:
             self.get_logger().info(f"Pinocchio joint order matches expected UR order: {self.joint_names}")
        
        # Initialize robot configuration to neutral position
        self.q = pin.neutral(self.model)
        self.initial_q_received = False # Flag to track if initial joint states have been received

        # --- End-effector Frame Setup ---
        self.end_effector_frame_name = "tool0" 
        if not self.model.existFrame(self.end_effector_frame_name):
            self.get_logger().error(f"End effector frame '{self.end_effector_frame_name}' not found in the Pinocchio model.")
            self.get_logger().error("Please check the correct end effector frame name for your UR10e URDF.")
            raise ValueError(f"End effector frame '{self.end_effector_frame_name}' not found.")
        self.end_effector_frame_id = self.model.getFrameId(self.end_effector_frame_name)
        self.get_logger().info(f"Using end effector frame: {self.end_effector_frame_name} (ID: {self.end_effector_frame_id})")

        # --- IK Solver Parameters ---
        self.tolerance = 1e-4  # Convergence tolerance for IK
        self.max_iter = 4000   # Maximum iterations for IK solver
        self.dt = 0.1          # Time step for numerical integration in IK
        self.damping = 5e-2    # Damping factor for damped least squares

        # --- ROS2 Subscriptions and Publishers ---
        self.pose_subscription = self.create_subscription(PoseStamped, '/target_cartesian_pose', self.pose_callback, 10)
        self.get_logger().info('Subscribed to /target_cartesian_pose topic.')

        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info('Subscribed to /joint_states topic for initial configuration.')

        # Use the controller name from the YAML configuration (as per your update)
        self.position_controller_topic = '/forward_position_controller/commands' 
        self.joint_position_publisher = self.create_publisher(Float64MultiArray, self.position_controller_topic, 10)
        self.get_logger().info(f'Publishing to {self.position_controller_topic} topic (Float64MultiArray).')

        self.last_q = self.q.copy() # Store the last known good joint configuration

    def joint_callback(self, msg: JointState):
        """
        Callback for /joint_states topic.
        Initializes the robot's current joint configuration from actual sensor data.
        """
        if not self.initial_q_received:
            self.get_logger().info("Received initial joint states.")
            self.initial_q_received = True

        joint_positions_map = {name: position for name, position in zip(msg.name, msg.position)}
        updated_q = pin.neutral(self.model) # Start with a neutral configuration

        # Update 'updated_q' with received joint positions for the controllable joints
        for joint_name in self.joint_names: 
            if joint_name in joint_positions_map:
                joint_id_in_model = self.model.getJointId(joint_name)
                q_idx = self.model.joints[joint_id_in_model].idx_q
                if q_idx != -1: # Check if it's a movable joint
                    updated_q[q_idx] = joint_positions_map[joint_name]
                else:
                    self.get_logger().warn(f"Joint '{joint_name}' found in JointState but not a movable joint in Pinocchio model (idx_q is -1).")
            else:
                self.get_logger().warn(f"Joint '{joint_name}' (expected controllable joint) not found in incoming /joint_states message. Using its default neutral value.")

        self.last_q = updated_q # Update the last known configuration

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for /target_cartesian_pose topic.
        Performs inverse kinematics to find joint angles for the target pose.
        """
        if not self.initial_q_received:
            self.get_logger().warn("Waiting for initial joint states before processing poses.")
            return

        target_pose = msg.pose
        self.get_logger().info(
            f"Received target pose: Position({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f}), "
            f"Orientation({target_pose.orientation.x:.3f}, {target_pose.orientation.y:.3f}, {target_pose.orientation.z:.3f}, {target_pose.orientation.w:.3f})"
        )

        # Normalize quaternion to prevent numerical instability
        q_norm = math.sqrt(
            target_pose.orientation.w**2 +
            target_pose.orientation.x**2 +
            target_pose.orientation.y**2 +
            target_pose.orientation.z**2
        )
        # Create Pinocchio SE3 object from ROS PoseStamped data
        rotation = pin.Quaternion(
            target_pose.orientation.w / q_norm,
            target_pose.orientation.x / q_norm,
            target_pose.orientation.y / q_norm,
            target_pose.orientation.z / q_norm
        ).matrix()

        translation = np.array([
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ])
        target_se3 = pin.SE3(rotation, translation)

        # Start IK from the last known good configuration
        q_current_guess = self.last_q.copy()
        found_solution = False

        self.get_logger().info("Starting IK iterations...")
        for i in range(self.max_iter):
            # Compute forward kinematics and Jacobians
            pin.computeJointJacobians(self.model, self.data, q_current_guess)
            pin.updateFramePlacements(self.model, self.data)

            # Get current end-effector pose and calculate error
            current_se3 = self.data.oMf[self.end_effector_frame_id]
            error = pin.log6(current_se3.inverse() * target_se3).vector # Error in tangent space
            error_norm = np.linalg.norm(error)

            # Log progress
            if i % 100 == 0 or error_norm < self.tolerance:
                self.get_logger().info(f"Iter: {i}, Error norm: {error_norm:.6f}")

            # Check for convergence
            if error_norm < self.tolerance:
                found_solution = True
                self.get_logger().info(f"Convergence achieved at iteration {i} with error {error_norm:.6f}")
                break

            # Get Jacobian in LOCAL frame and select only relevant columns (for movable joints)
            J = pin.getFrameJacobian(self.model, self.data, self.end_effector_frame_id, pin.ReferenceFrame.LOCAL)
            J = J[:, :self.model.nq] # Ensure Jacobian matches the number of joint DOFs

            try:
                # Compute damped pseudo-inverse using SVD
                U, S, Vt = np.linalg.svd(J, full_matrices=False)
                S_damped = S / (S**2 + self.damping)
                J_pinv = Vt.T @ np.diag(S_damped) @ U.T
            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f"SVD failed at iteration {i}: {e}")
                continue

            # Calculate joint velocity update
            delta_q = J_pinv @ error
            # Integrate joint velocities to update joint positions
            q_current_guess = pin.integrate(self.model, q_current_guess, self.dt * delta_q)
            # Clip joint positions to stay within joint limits
            q_current_guess = np.clip(q_current_guess, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        if found_solution:
            self.get_logger().info("IK solution found. Publishing joint positions.")
            self.last_q = q_current_guess # Update last_q with the new solution

            joint_positions_msg = Float64MultiArray()
            joint_positions_to_publish = []
            
            # Populate the message with joint positions in the correct order
            for joint_name in self.joint_names:
                joint_id = self.model.getJointId(joint_name)
                q_idx = self.model.joints[joint_id].idx_q
                if q_idx != -1:
                    joint_positions_to_publish.append(q_current_guess[q_idx])
                else:
                    self.get_logger().error(f"Joint {joint_name} has invalid q index. Cannot publish.")
                    return # Exit if a critical joint index is missing

            joint_positions_msg.data = joint_positions_to_publish
            self.joint_position_publisher.publish(joint_positions_msg)
        else:
            self.get_logger().warn(f"IK failed after {self.max_iter} iterations. Final error: {error_norm:.4f}")

def main(args=None):
    rclpy.init(args=args)
    ur10e_ik_node = UR10eIKNode()
    try:
        # spin() will block until the node is shut down,
        # either by an external request or by Ctrl+C.
        rclpy.spin(ur10e_ik_node)
    except KeyboardInterrupt:
        # This block is optional if you have no specific message to print on exit.
        # The spin() function will exit gracefully on Ctrl+C.
        ur10e_ik_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Cleanly destroy the node.
        ur10e_ik_node.destroy_node()
        # Shut down rclpy. This should happen once at the very end.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

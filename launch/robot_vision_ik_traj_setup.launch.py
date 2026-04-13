from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml  # Load YAML library


# Function to read calibration YAML file and start static_transform_publisher
def load_calibration_from_yaml(context):
    # Build the complete path to the calibration YAML file
    # Make sure 'ur10e_ros2' is the correct package name
    calibration_file_path = PathJoinSubstitution([
        FindPackageShare('ur10e_ros2'),
        'calibration_results',
        'hand_eye_transform.yaml'
    ]).perform(context)  # .perform(context) needed to resolve substitutions at runtime

    # Verify that the file exists before attempting to load it
    if not os.path.exists(calibration_file_path):
        # If file is not found, raise a meaningful error
        raise FileNotFoundError(f"Error: Calibration file not found at path: {calibration_file_path}")

    try:
        # Open and load YAML data
        with open(calibration_file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        # Extract translation and rotation values
        trans = calib_data['translation']
        rot = calib_data['rotation']

        # Prepare arguments for static_transform_publisher node
        # Note the order: x, y, z, roll, pitch, yaw (or quaternion x,y,z,w), parent_frame, child_frame
        # Here we use quaternions (x, y, z, w)
        static_tf_args = [
            str(trans['x']), str(trans['y']), str(trans['z']),
            str(rot['x']), str(rot['y']), str(rot['z']), str(rot['w']),
            calib_data['parent_frame_id'],
            calib_data['child_frame_id']
        ]
        
        # Return a list of actions, in this case a single node
        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_hand_eye_tf_publisher',
                output='screen',  # Show node output on console
                arguments=static_tf_args
            )
        ]
    except Exception as e:
        # Handle any errors during YAML loading or parsing
        raise RuntimeError(f"Error during calibration loading from YAML file: {e}")

def generate_launch_description():
    return LaunchDescription([
        # --- Launch Arguments ---
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='false',
            description='Enable Realsense camera'
        ),
        DeclareLaunchArgument(
            'enable_qb',
            default_value='false',
            description='Enable QBSofthand gripper'
        ),

        # --- 0. Start LLM App ---
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('llm_app'),
                'launch',
                'llm_app.launch.py'
            ])
        ),

        # --- Realsense (if enabled) ---
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ]),
            condition=IfCondition(LaunchConfiguration('enable_realsense'))
        ),

        # --- QB SoftHand (if enabled) ---
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('qb_softhand_industry_driver'),
                'launch',
                'softhand_industry_communication_handler.launch.py'
            ]),
            condition=IfCondition(LaunchConfiguration('enable_qb'))
        ),

        # --- 1. Start Static Transform Publisher Node for calibration ---
        # OpaqueFunction executes the Python function 'load_calibration_from_yaml'
        # and includes the actions (Nodes) that it returns.
        OpaqueFunction(function=load_calibration_from_yaml),

        # --- 2. Start Forward Kinematics (FK) Node ---
        Node(
            package='ur10e_ros2',  # Make sure this is the correct package name
            executable='fk_node',  # Executable name defined in setup.py
            name='fk_node',        # ROS 2 node name
            output='screen',       # Show node output on console
        ),
        Node(
            package='ur10e_ros2',
            executable='task_executor_node_complete',
            name='task_executor_node_complete',  # Descriptive node name
            output='screen',
        ),

        Node(
            package='ur10e_ros2',
            executable='task_saving_node_complete',
            name='task_saving_node_complete',  # Descriptive node name
            output='screen',
        ),
        Node(
            package='ur10e_ros2',
            executable='ik_trajectory_node',
            name='ik_trajectory_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ur10e_ros2'),
                    'config',
                    'ik_trajectory_node_params.yaml'
                ])
            ]
        ),
        # Node(
        #     package='ur10e_ros2',  # Make sure this is the correct package name
        #     executable='ur10e_ik_trajectory_node',  # Executable name defined in setup.py
        #     name='ur10e_ik_trajectory_node',        # ROS 2 node name
        #     output='screen',                        # Show node output on console
        # ),

    ])



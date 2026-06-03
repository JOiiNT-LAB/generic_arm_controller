from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
import yaml


def load_calibration_from_yaml(context):
    """Load hand-eye calibration and create static transform publisher"""
    calibration_file_path = PathJoinSubstitution([
        FindPackageShare('generic_arm_controller'),
        'calibration_results',
        'hand_eye_transform.yaml'
    ]).perform(context)

    if not os.path.exists(calibration_file_path):
        raise FileNotFoundError(
            f"Calibration file not found: {calibration_file_path}"
        )

    try:
        with open(calibration_file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        trans = calib_data['translation']
        rot   = calib_data['rotation']

        static_tf_args = [
            str(trans['x']), str(trans['y']), str(trans['z']),
            str(rot['x']),   str(rot['y']),   str(rot['z']), str(rot['w']),
            calib_data['parent_frame_id'],
            calib_data['child_frame_id'],
        ]

        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_hand_eye_tf_publisher',
                output='screen',
                arguments=static_tf_args,
            )
        ]
    except Exception as e:
        raise RuntimeError(f"Error loading calibration YAML: {e}")


def generate_launch_description():
    """
    Secondary launcher for UR10e robot.
    Launches FK, IK trajectory node, and calibration TF.
    """

    # Load hand-eye calibration TF
    static_tf_node = OpaqueFunction(function=load_calibration_from_yaml)

    # Forward Kinematics node
    fk_node = Node(
        package='generic_arm_controller',
        executable='fk_node',
        name='fk_node',
        output='screen',
        respawn=False,
    )

    # Inverse Kinematics trajectory node
    ik_node = Node(
        package='generic_arm_controller',
        executable='ik_trajectory_node',
        name='ik_trajectory_node',
        output='screen',
        respawn=False,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('generic_arm_controller'),
                'config',
                'ik_trajectory_node_params.yaml',
            ])
        ],
    )

    return LaunchDescription([
        static_tf_node,
        fk_node,
        ik_node,
    ])

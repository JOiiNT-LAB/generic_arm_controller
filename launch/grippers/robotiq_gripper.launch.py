from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Secondary launcher for Robotiq gripper.
    Launches the gripper manager node for RG2/RG6 control.
    """

    gripper_node = Node(
        package='generic_arm_controller',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        respawn=False,
    )

    return LaunchDescription([
        gripper_node,
    ])

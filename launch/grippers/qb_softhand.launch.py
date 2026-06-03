from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Secondary launcher for QB Softhand Industry gripper.
    Includes the QB Softhand communication handler launch file.
    """

    qb_softhand_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('qb_softhand_industry_driver'),
            'launch',
            'softhand_industry_communication_handler.launch.py',
        ])
    )

    return LaunchDescription([
        qb_softhand_launch,
    ])

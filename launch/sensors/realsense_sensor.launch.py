from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Secondary launcher for RealSense RGB-D camera.
    Launches the RealSense2 camera driver.
    """

    realsense_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py',
        ])
    )

    return LaunchDescription([
        realsense_launch,
    ])

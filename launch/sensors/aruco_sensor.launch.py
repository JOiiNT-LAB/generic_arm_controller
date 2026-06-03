from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Secondary launcher for ArUco marker detection.
    Launches the ArUco single marker detector node.
    """

    aruco_detect_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_size': 0.1778,
            'marker_id': 0,
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': 'aruco_marker',
            'reference_frame': 'camera_color_optical_frame',
            'dictionary': 10,
        }],
        remappings=[
            ('/image', '/camera_sensor/realsense_camera/image_raw'),
            ('/camera_info', '/camera_sensor/realsense_camera/camera_info')
        ]
    )

    return LaunchDescription([
        aruco_detect_node,
    ])

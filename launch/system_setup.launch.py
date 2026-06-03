"""
Modular System Setup Launcher for Generic Arm Controller

This launcher composes secondary launchers for robots, grippers, and sensors.
It provides a cleaner, more modular alternative to robot_vision_ik_traj_setup.launch.py

Usage:
    # UR10e + Robotiq gripper with RealSense
    ros2 launch generic_arm_controller system_setup.launch.py \
        robot:=ur10e gripper:=robotiq enable_realsense:=true

    # UR10e + QB Softhand with RealSense and ArUco
    ros2 launch generic_arm_controller system_setup.launch.py \
        robot:=ur10e gripper:=qb_softhand enable_realsense:=true enable_aruco:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -----------------------------------------------------------------------
    # Launch Arguments
    # -----------------------------------------------------------------------
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='ur10e',
        description='Robot type (ur10e, ur5e, franka, etc.)'
    )

    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value='robotiq',
        description='Gripper type (robotiq, qb_softhand, rg2, etc.)'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='Target TF frame for IK trajectory execution'
    )

    enable_realsense_arg = DeclareLaunchArgument(
        'enable_realsense',
        default_value='false',
        description='Enable RealSense RGB-D camera'
    )

    enable_aruco_arg = DeclareLaunchArgument(
        'enable_aruco',
        default_value='false',
        description='Enable ArUco marker detection'
    )

    # -----------------------------------------------------------------------
    # Robot launchers (conditional based on robot argument)
    # -----------------------------------------------------------------------
    ur10e_launcher = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('generic_arm_controller'),
            'launch/robots',
            'ur10e.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('robot') == 'ur10e')
    )

    # -----------------------------------------------------------------------
    # Gripper launchers (conditional based on gripper argument)
    # -----------------------------------------------------------------------
    robotiq_launcher = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('generic_arm_controller'),
            'launch/grippers',
            'robotiq_gripper.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('gripper') == 'robotiq')
    )

    qb_softhand_launcher = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('generic_arm_controller'),
            'launch/grippers',
            'qb_softhand.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('gripper') == 'qb_softhand')
    )

    # -----------------------------------------------------------------------
    # Sensor launchers (conditional)
    # -----------------------------------------------------------------------
    realsense_launcher = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('generic_arm_controller'),
            'launch/sensors',
            'realsense_sensor.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('enable_realsense'))
    )

    aruco_launcher = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('generic_arm_controller'),
            'launch/sensors',
            'aruco_sensor.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('enable_aruco'))
    )

    # -----------------------------------------------------------------------
    # High-level task management nodes
    # -----------------------------------------------------------------------
    task_executor_node = Node(
        package='generic_arm_controller',
        executable='task_executor_node_complete',
        name='task_executor_node_complete',
        output='screen',
        respawn=False,
        parameters=[
            {
                'target_frame': LaunchConfiguration('target_frame'),
            }
        ],
    )

    task_saving_node = Node(
        package='generic_arm_controller',
        executable='task_saving_node_complete',
        name='task_saving_node_complete',
        output='screen',
        respawn=False,
    )

    # -----------------------------------------------------------------------
    # LLM app with delay (ensures all services are ready)
    # -----------------------------------------------------------------------
    llm_app_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('llm_app'),
            'launch',
            'llm_app.launch.py',
        ])
    )

    llm_app_delayed = TimerAction(
        period=3.0,
        actions=[llm_app_launch],
    )

    # -----------------------------------------------------------------------
    # Compose all launchers
    # -----------------------------------------------------------------------
    return LaunchDescription([
        # Arguments
        robot_arg,
        gripper_arg,
        target_frame_arg,
        enable_realsense_arg,
        enable_aruco_arg,

        # Robot and gripper launchers
        ur10e_launcher,
        robotiq_launcher,
        qb_softhand_launcher,

        # Sensor launchers
        realsense_launcher,
        aruco_launcher,

        # Task management (high-level logic)
        task_executor_node,
        task_saving_node,

        # LLM interface (delayed start)
        llm_app_delayed,
    ])

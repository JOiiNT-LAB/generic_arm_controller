import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'generic_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line is CRITICAL for service generation and installation
        # Other data files for your package
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'calibration_results'), glob(os.path.join('calibration_results', '*'))),
    ],
    install_requires=[
        'setuptools',
        'pinocchio', # Keep if this is a pip-installable Python package
        'numpy',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Generic ROS2 package for controlling and manipulating robotic arms (UR, Franka, KUKA, etc.).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'fk_node = {package_name}.fk_node:main',
            f'marker_pose_transformer_node = {package_name}.marker_pose_transformer_node:main',
            f'ur10e_ik_trajectory_node = {package_name}.ur10e_ik_trajectory_node:main',
            f'task_saving_node_complete = {package_name}.task_saving_node_complete:main',
            f'task_executor_node_complete = {package_name}.task_executor_node_complete:main',
            f'ik_trajectory_node = {package_name}.ik_trajectory_node:main',  # ← aggiungi questa
            f'gripper_node = {package_name}.gripper_manager:main',

        ],
    },
)


























# import os
# import glob # Make sure this is imported at the top of your setup.py
# from setuptools import find_packages, setup

# package_name = 'ur10e_ros2' # Ensure this is correctly defined

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # --- ENSURE THESE LINES ARE PRESENT AND CORRECT ---
#         # Include all launch files from the 'launch' directory
#         (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py'))),
#         # Include all files from the 'config' directory
#         (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
#         # Include all files from the 'calibration_results' directory
#         (os.path.join('share', package_name, 'calibration_results'), glob.glob('calibration_results/*')),
#         (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')), # <-- NEW

#     ],
#     install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'std_srvs', 
#                       'control_msgs', 'trajectory_msgs', 'pinocchio', 'numpy', 
#                       'sensor_msgs', 'pyyaml'], # Ho aggiunto qui le dipendenze comuni che ti serviranno
#                                                  # Assicurati che siano tutte elencate    zip_safe=True,
#     maintainer='Your Name',
#     maintainer_email='your_email@example.com',
#     description='ROS2 package for controlling and interacting with the UR10e robot.',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             f'fk_node = {package_name}.fk_node:main',
#             f'ik_node_position_controllers = {package_name}.ik_node_position_controllers:main',
#             f'marker_pose_transformer_node = {package_name}.marker_pose_transformer_node:main',
#             f'object_grapping_node = {package_name}.object_grapping_node:main',
#             f'pose_publisher_node = {package_name}.PosePublisher_node:main',
#             f'ur10e_ik_trajectory_node = {package_name}.ur10e_ik_trajectory_node:main',
#             f'task_saving_node = {package_name}.task_saving_node:main',
#             f'task_executor_node = {package_name}.task_executor_node:main',],
#     },
# )
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'generic_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Inserisci TUTTI i tuoi data_files QUI, dentro una singola lista
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch/robots'), glob(os.path.join('launch/robots', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch/grippers'), glob(os.path.join('launch/grippers', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch/sensors'), glob(os.path.join('launch/sensors', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config/robots'), glob(os.path.join('config/robots', '*.yaml'))),
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
            f'ik_trajectory_node = {package_name}.ik_trajectory_node:main',
            f'task_saving_node_complete = {package_name}.task_saving_node_complete:main',
            f'task_executor_node_complete = {package_name}.task_executor_node_complete:main',
            f'gripper_node = {package_name}.gripper_manager:main',
            f'validate_pipeline = {package_name}.validate_pipeline:main',
        ],
    },
)
















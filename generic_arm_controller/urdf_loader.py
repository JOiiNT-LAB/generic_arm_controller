"""
urdf_loader.py
==============
Shared utility to generate a Pinocchio-ready URDF from a xacro file.
Used by both the FK and IK nodes so URDF logic is not duplicated.
"""

import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory


def build_pinocchio_urdf(
    urdf_package: str,
    robot_type: str,
    logger=None,
    extra_xacro_args: dict = None,
) -> str:
    """
    Generate a Pinocchio-compatible URDF from a xacro file by:
      1. Running xacro to produce a raw URDF
      2. Replacing all package:// mesh URIs with absolute paths

    Parameters
    ----------
    urdf_package     : ROS package name containing the URDF/xacro
    robot_type       : Robot type string passed to xacro (e.g. 'ur10e', 'ur5e')
    logger           : Optional rclpy logger for status messages
    extra_xacro_args : Optional dict of additional xacro arguments

    Returns
    -------
    Path to the final URDF file ready for Pinocchio
    """
    def log(msg):
        if logger:
            logger.info(msg)

    def log_err(msg):
        if logger:
            logger.error(msg)

    try:
        pkg_share  = get_package_share_directory(urdf_package)
    except Exception as e:
        log_err(f"Package '{urdf_package}' not found: {e}")
        raise

    raw_urdf   = f'/tmp/{robot_type}_pinocchio_raw.urdf'
    final_urdf = f'/tmp/{robot_type}_pinocchio_final.urdf'
    is_franka  = urdf_package == 'franka_description'

    if is_franka:
        xacro_path = os.path.join(pkg_share, 'robots', robot_type, f'{robot_type}.urdf.xacro')
        xacro_cmd = [
            'ros2', 'run', 'xacro', 'xacro', xacro_path,
            f'robot_type:={robot_type}',
            'hand:=true',
            'ee_id:=franka_hand',
        ]
    else:
        xacro_path = os.path.join(pkg_share, 'urdf', 'ur.urdf.xacro')
        xacro_cmd = [
            'ros2', 'run', 'xacro', 'xacro', xacro_path,
            f'ur_type:={robot_type}',
            f'name:={robot_type}',
            'transmission_hw_interface:=""',
            'sim_gazebo:=false',
            'sim_ignition:=false',
            'use_fake_hardware:=false',
            'headless_mode:=false',
        ]

    if extra_xacro_args:
        for k, v in extra_xacro_args.items():
            xacro_cmd.append(f'{k}:={v}')

    log(f'Running xacro for {robot_type} from {urdf_package}...')
    try:
        with open(raw_urdf, 'w') as f:
            subprocess.run(xacro_cmd, check=True, stdout=f, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        log_err(f'xacro failed: {e.stderr.decode()}')
        raise

    with open(raw_urdf, 'r') as f:
        content = f.read()

    # pin.buildModelFromUrdf() (usato da fk/ik node) non carica le mesh: la sostituzione
    # package://->path assoluto serve solo per il layout ur_description.
    if not is_franka:
        mesh_dir = os.path.join(pkg_share, 'meshes')
        for subfolder in ('visual', 'collision'):
            content = re.sub(
                rf'filename="package://{urdf_package}/meshes/{robot_type}/{subfolder}/([^"]+)"',
                lambda m, sf=subfolder: (
                    f'filename="{os.path.join(mesh_dir, robot_type, sf, m.group(1))}"'
                ),
                content,
            )

    with open(final_urdf, 'w') as f:
        f.write(content)

    log(f'URDF ready: {final_urdf}')
    return final_urdf
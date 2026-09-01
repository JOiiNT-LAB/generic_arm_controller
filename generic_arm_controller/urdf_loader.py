"""
urdf_loader.py
==============
Shared utility to generate a Pinocchio-ready URDF from a xacro file.
Used by both the FK and IK nodes so URDF logic is not duplicated.

Robot-specific values (xacro file path, xacro arguments) are supplied by the
caller (read from a per-robot profile YAML, see config/robots/) - this module
has no per-robot branching.
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory


def build_pinocchio_urdf(
    urdf_package: str,
    robot_type: str,
    xacro_relative_path: str,
    xacro_args: list = None,
    logger=None,
) -> str:
    """
    Generate a Pinocchio-compatible URDF by running xacro on the given file.

    pin.buildModelFromUrdf() (used by fk_node/ik_trajectory_node) only reads
    joints/links/inertials, not <visual>/<collision> mesh tags, so no mesh
    path post-processing is needed here - just the raw xacro output.

    Parameters
    ----------
    urdf_package        : ROS package name containing the URDF/xacro
    robot_type          : Robot type string, used for the temp file name and logging
    xacro_relative_path : Path to the xacro file, relative to the package share dir
    xacro_args          : xacro arguments as a list of "name:=value" strings
    logger              : Optional rclpy logger for status messages

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
        pkg_share = get_package_share_directory(urdf_package)
    except Exception as e:
        log_err(f"Package '{urdf_package}' not found: {e}")
        raise

    xacro_path = os.path.join(pkg_share, xacro_relative_path)
    # PID in the name: fk_node and ik_trajectory_node call this function with the
    # same robot_type in separate processes started in parallel by the launcher -
    # a shared path caused a race on open('w')/subprocess.run between the two (one
    # would truncate the other's file mid-write -> corrupted URDF -> both crash
    # immediately).
    final_urdf = f'/tmp/{robot_type}_pinocchio_final_{os.getpid()}.urdf'

    xacro_cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_path] + list(xacro_args or [])

    log(f'Running xacro for {robot_type} from {urdf_package}...')
    try:
        with open(final_urdf, 'w') as f:
            subprocess.run(xacro_cmd, check=True, stdout=f, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        log_err(f'xacro failed: {e.stderr.decode()}')
        raise

    log(f'URDF ready: {final_urdf}')
    return final_urdf

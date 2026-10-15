"""
Read, check, compare and write hand-eye calibration files (no ROS needed).

A calibration file is what handeye_ros2 writes and what the launch files read:

    parent_frame_id: panda_link8      # the flange
    child_frame_id: camera_link
    translation: {x, y, z}            # metres
    rotation: {x, y, z, w}            # quaternion

The same file feeds the real robot (static TF published by
robot_vision_ik_traj_setup.launch.py) and the simulation (the camera pose in
arm_gz_bringup/urdf/fr3_camera.gazebo.xacro), so what is written here must be sane.
"""

import math
import os
import tempfile

import yaml

REQUIRED_KEYS = ('parent_frame_id', 'child_frame_id', 'translation', 'rotation')
QUATERNION_NORM_TOLERANCE = 1e-3


class CalibrationError(ValueError):
    """The calibration file is missing data, not finite, or inconsistent."""


def _finite_floats(section, keys, name):
    try:
        values = [float(section[k]) for k in keys]
    except (KeyError, TypeError, ValueError) as ex:
        raise CalibrationError(f"'{name}' must contain numeric {', '.join(keys)}: {ex}")
    if not all(math.isfinite(v) for v in values):
        raise CalibrationError(f"'{name}' contains NaN or infinite values")
    return values


def validate(data):
    """Raise CalibrationError unless `data` is a usable calibration."""
    if not isinstance(data, dict):
        raise CalibrationError('the file does not contain a YAML mapping')
    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise CalibrationError(f'missing keys: {", ".join(missing)}')
    for key in ('parent_frame_id', 'child_frame_id'):
        if not isinstance(data[key], str) or not data[key]:
            raise CalibrationError(f"'{key}' must be a non-empty string")
    _finite_floats(data['translation'], 'xyz', 'translation')
    q = _finite_floats(data['rotation'], 'xyzw', 'rotation')
    if math.sqrt(sum(v * v for v in q)) < 1e-6:
        raise CalibrationError("'rotation' is a zero quaternion")


def translation(data):
    return tuple(_finite_floats(data['translation'], 'xyz', 'translation'))


def quaternion(data):
    """Quaternion as (x, y, z, w), exactly as stored (not normalised)."""
    return tuple(_finite_floats(data['rotation'], 'xyzw', 'rotation'))


def quaternion_norm(data):
    return math.sqrt(sum(v * v for v in quaternion(data)))


def normalized(data):
    """Return a copy of `data` whose quaternion has unit norm."""
    validate(data)
    x, y, z, w = quaternion(data)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    out = {
        'parent_frame_id': data['parent_frame_id'],
        'child_frame_id': data['child_frame_id'],
        'translation': dict(zip('xyz', translation(data))),
        'rotation': {'x': x / n, 'y': y / n, 'z': z / n, 'w': w / n},
    }
    return out


def load_calibration(path):
    """Load and validate a calibration file."""
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except OSError as ex:
        raise CalibrationError(f'cannot read {path}: {ex}')
    except yaml.YAMLError as ex:
        raise CalibrationError(f'{path} is not valid YAML: {ex}')
    try:
        validate(data)
    except CalibrationError as ex:
        raise CalibrationError(f'{path}: {ex}')
    return data


def same_frames(a, b):
    return (a['parent_frame_id'] == b['parent_frame_id']
            and a['child_frame_id'] == b['child_frame_id'])


def delta(a, b):
    """Return (translation distance in mm, rotation angle in degrees) between two calibrations."""
    ta, tb = translation(a), translation(b)
    dist_mm = 1000.0 * math.sqrt(sum((x - y) ** 2 for x, y in zip(ta, tb)))
    qa, qb = quaternion(a), quaternion(b)
    na = math.sqrt(sum(v * v for v in qa))
    nb = math.sqrt(sum(v * v for v in qb))
    dot = abs(sum(x * y for x, y in zip(qa, qb))) / (na * nb)
    angle_deg = math.degrees(2.0 * math.acos(min(1.0, dot)))
    return dist_mm, angle_deg


def save_calibration(path, data):
    """
    Write `data` to `path` without breaking a symlink at `path`.

    colcon --symlink-install makes the installed calibration file a symlink to the
    source one: the real file is replaced (atomically) and the link stays valid, so
    no rebuild is needed. Returns (resolved path, whether `path` was a symlink).
    """
    validate(data)
    is_link = os.path.islink(path)
    target = os.path.realpath(path)
    directory = os.path.dirname(target)
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=directory, suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False)
        os.replace(tmp_path, target)
    except BaseException:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    return target, is_link


# ---------------------------------------------------------------------------
# Locations (need ament_index_python, i.e. a sourced ROS environment)
# ---------------------------------------------------------------------------

def package_share_dir():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('generic_arm_controller')


def workspace_dir():
    """
    Workspace root, derived from the install layout.

    <ws>/install/generic_arm_controller/share/generic_arm_controller -> <ws>
    """
    return os.path.abspath(os.path.join(package_share_dir(), '..', '..', '..', '..'))


def load_robot_profile(robot, share_dir=None):
    """Parameters of config/robots/<robot>.yaml, the profile shared by all the nodes."""
    share_dir = share_dir or package_share_dir()
    path = os.path.join(share_dir, 'config', 'robots', f'{robot}.yaml')
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)['/**']['ros__parameters']
    except (OSError, KeyError, TypeError) as ex:
        raise CalibrationError(f"cannot read the profile of robot '{robot}' ({path}): {ex}")


def calibration_file(robot, calibration_dir=None):
    directory = calibration_dir or os.path.join(package_share_dir(), 'calibration_results')
    return os.path.join(directory, f'hand_eye_transform_{robot}.yaml')

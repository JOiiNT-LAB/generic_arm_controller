"""
Offline evaluation of hand-eye calibrations (no ROS needed).

The marker is physically fixed, so a good calibration makes its position in the robot
base frame stay put while the arm moves; a wrong one makes it "slide" with the point of
view. Given a capture (flange pose at the detection timestamp + marker pose in the
camera optical frame, see capture_hand_eye) any candidate calibration can be scored
without installing it or relaunching the pipeline.

Frames used below: G = base -> flange, X = flange -> camera optical frame (the thing being
estimated), M = optical frame -> marker. The marker in the base frame is G @ X @ M.
Calibration files describe flange -> camera_link, so X = calibration @ camera_offset, where
camera_offset is the fixed camera_link -> optical frame transform stored in the capture.
"""

import json
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from generic_arm_controller.hand_eye import calibration_io as cio

MIN_SAMPLES = 10


def mat(t, q):
    """4x4 transform from a translation (x, y, z) and a quaternion (x, y, z, w)."""
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(q).as_matrix()
    m[:3, 3] = t
    return m


def calibration_matrix(cal):
    return mat(cio.translation(cal), cio.quaternion(cal))


def load_capture(path):
    """Return (G, M, camera_offset) as 4x4 matrices from a capture JSON file."""
    with open(path, 'r') as f:
        data = json.load(f)
    samples = data['samples']
    if len(samples) < MIN_SAMPLES:
        raise cio.CalibrationError(
            f'{path} has only {len(samples)} samples (at least {MIN_SAMPLES} needed)')
    offset = mat(data['camera_offset']['t'], data['camera_offset']['q'])
    g = np.array([mat(s['ee_t'], s['ee_q']) for s in samples])
    m = np.array([mat(s['marker_t'], s['marker_q']) for s in samples])
    return g, m, offset


def marker_in_base(g, x, m):
    """Marker position (n x 3) in the base frame for the calibration X."""
    return np.einsum('nij,jk,nkl->nil', g, x, m)[:, :3, 3]


def stats(points):
    """Return (max-min per axis in mm, RMS distance from the mean in mm)."""
    spread = (points.max(axis=0) - points.min(axis=0)) * 1000.0
    rms = math.sqrt(((points - points.mean(axis=0)) ** 2).sum(axis=1).mean()) * 1000.0
    return spread, rms


def evaluate(g, m, offset, cal):
    """Score one calibration (flange -> camera_link): (spread per axis mm, RMS mm)."""
    return stats(marker_in_base(g, calibration_matrix(cal) @ offset, m))


def _params_to_x(p):
    x = np.eye(4)
    x[:3, :3] = Rotation.from_rotvec(p[:3]).as_matrix()
    x[:3, 3] = p[3:]
    return x


def _residual(p, g, m):
    points = marker_in_base(g, _params_to_x(p), m)
    return (points - points.mean(axis=0)).ravel()


def _fit_x(g, m, x0, max_nfev=20000):
    p0 = np.concatenate([Rotation.from_matrix(x0[:3, :3]).as_rotvec(), x0[:3, 3]])
    return least_squares(_residual, p0, args=(g, m), loss='soft_l1', f_scale=0.005,
                         max_nfev=max_nfev)


def fit(g, m, offset, start_cal):
    """
    Estimate the calibration that keeps the marker most still, starting from `start_cal`.

    Returns a dict with the calibration (same frames as `start_cal`, unit quaternion),
    its spread/RMS on all the data, and train/test RMS for an alternating and a temporal
    split (the temporal one is the meaningful one: adjacent samples are nearly identical).
    """
    x0 = calibration_matrix(start_cal) @ offset
    result = _fit_x(g, m, x0)
    spread, rms = stats(marker_in_base(g, _params_to_x(result.x), m))

    n = len(g)
    validation = []
    for name, train in (('alternating', np.arange(n) % 2 == 0),
                        ('temporal', np.arange(n) < n // 2)):
        test = ~train
        part = _fit_x(g[train], m[train], x0)
        x = _params_to_x(part.x)
        validation.append({
            'split': name,
            'train_rms_mm': stats(marker_in_base(g[train], x, m[train]))[1],
            'test_rms_mm': stats(marker_in_base(g[test], x, m[test]))[1],
        })

    flange_to_camera_link = _params_to_x(result.x) @ np.linalg.inv(offset)
    q = Rotation.from_matrix(flange_to_camera_link[:3, :3]).as_quat()
    t = flange_to_camera_link[:3, 3]
    calibration = {
        'parent_frame_id': start_cal['parent_frame_id'],
        'child_frame_id': start_cal['child_frame_id'],
        'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
        'rotation': {'x': float(q[0]), 'y': float(q[1]), 'z': float(q[2]), 'w': float(q[3])},
    }
    return {'calibration': calibration, 'spread_mm': spread, 'rms_mm': rms,
            'validation': validation}

"""Tests for the hand-eye calibration tools (no ROS needed)."""

import json
import math
import os

from generic_arm_controller import install_hand_eye
from generic_arm_controller.hand_eye import calibration_io as cio
from generic_arm_controller.hand_eye import evaluate as ev
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml


def make_cal(t=(0.045, -0.059, 0.019), q=(0.656, -0.265, 0.659, 0.253),
             parent='panda_link8', child='camera_link'):
    return {
        'parent_frame_id': parent,
        'child_frame_id': child,
        'translation': dict(zip('xyz', t)),
        'rotation': dict(zip('xyzw', q)),
    }


def write_yaml(path, data):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)


# ---------------------------------------------------------------------------
# calibration_io
# ---------------------------------------------------------------------------

def test_valid_calibration_passes():
    cio.validate(make_cal())


@pytest.mark.parametrize('broken', [
    None,
    [],
    {'parent_frame_id': 'a'},
    {**make_cal(), 'parent_frame_id': ''},
    {**make_cal(), 'translation': {'x': 0.0, 'y': 0.0}},
    {**make_cal(), 'translation': {'x': math.nan, 'y': 0.0, 'z': 0.0}},
    {**make_cal(), 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}},
    {**make_cal(), 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': math.inf}},
    {**make_cal(), 'translation': {'x': 'a', 'y': 0.0, 'z': 0.0}},
])
def test_broken_calibration_is_rejected(broken):
    with pytest.raises(cio.CalibrationError):
        cio.validate(broken)


def test_normalized_gives_unit_quaternion_and_keeps_the_rest():
    cal = make_cal(q=(0.0, 0.0, 0.0, 2.0))
    out = cio.normalized(cal)
    assert cio.quaternion_norm(out) == pytest.approx(1.0)
    assert out['rotation']['w'] == pytest.approx(1.0)
    assert out['translation'] == cal['translation']
    assert out['parent_frame_id'] == 'panda_link8'


def test_delta_measures_millimetres_and_degrees():
    a = make_cal(t=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))
    b = make_cal(t=(0.010, 0.0, 0.0),
                 q=tuple(Rotation.from_euler('z', 5, degrees=True).as_quat()))
    dist, angle = cio.delta(a, b)
    assert dist == pytest.approx(10.0)
    assert angle == pytest.approx(5.0)


def test_delta_ignores_quaternion_sign_and_scale():
    a = make_cal(q=(0.0, 0.0, 0.0, 1.0))
    b = make_cal(q=(0.0, 0.0, 0.0, -3.0))
    assert cio.delta(a, b)[1] == pytest.approx(0.0, abs=1e-6)


def test_same_frames():
    assert cio.same_frames(make_cal(), make_cal())
    assert not cio.same_frames(make_cal(), make_cal(child='camera_color_optical_frame'))


def test_save_then_load_round_trip(tmp_path):
    path = tmp_path / 'cal.yaml'
    cal = cio.normalized(make_cal())
    cio.save_calibration(str(path), cal)
    loaded = cio.load_calibration(str(path))
    assert loaded == cal
    assert sorted(yaml.safe_load(path.read_text())) == sorted(cio.REQUIRED_KEYS)


def test_save_keeps_a_symlink_valid(tmp_path):
    real_dir = tmp_path / 'src'
    real_dir.mkdir()
    real = real_dir / 'cal.yaml'
    write_yaml(real, make_cal())
    link = tmp_path / 'cal_link.yaml'
    os.symlink(real, link)

    written, was_link = cio.save_calibration(str(link), cio.normalized(make_cal(t=(1, 2, 3))))
    assert was_link
    assert os.path.islink(link)
    assert written == str(real)
    assert cio.translation(cio.load_calibration(str(real))) == (1.0, 2.0, 3.0)


def test_load_reports_unreadable_files(tmp_path):
    with pytest.raises(cio.CalibrationError):
        cio.load_calibration(str(tmp_path / 'missing.yaml'))
    bad = tmp_path / 'bad.yaml'
    bad.write_text('translation: [unclosed')
    with pytest.raises(cio.CalibrationError):
        cio.load_calibration(str(bad))


# ---------------------------------------------------------------------------
# evaluate (synthetic data with a known calibration)
# ---------------------------------------------------------------------------

OFFSET = ev.mat([0.0, 0.0, 0.0], Rotation.from_euler('xyz', [-90, 0, -90], degrees=True).as_quat())


def true_calibration():
    """Flange -> camera_link of the synthetic scene."""
    x_true = ev.mat([0.05, -0.06, 0.02],
                    Rotation.from_euler('xyz', [10, -20, 30], degrees=True).as_quat())
    flange_to_link = x_true @ np.linalg.inv(OFFSET)
    q = Rotation.from_matrix(flange_to_link[:3, :3]).as_quat()
    return make_cal(t=tuple(flange_to_link[:3, 3]), q=tuple(q))


def make_scene(n=60, seed=1):
    """Flange poses G and marker poses M (in the optical frame) for a fixed marker."""
    rng = np.random.default_rng(seed)
    x_true = ev.calibration_matrix(true_calibration()) @ OFFSET
    marker_in_base = ev.mat([0.5, 0.1, 0.0], [0.0, 0.0, 0.0, 1.0])
    rotations = Rotation.random(n, random_state=seed)
    g = np.array([ev.mat(rng.uniform(-0.3, 0.3, 3) + [0.3, 0.0, 0.5], r.as_quat())
                  for r in rotations])
    m = np.array([np.linalg.inv(gi @ x_true) @ marker_in_base for gi in g])
    return g, m


def perturbed(cal, dt=0.005, deg=3.0):
    t = np.array(cio.translation(cal)) + [dt, -dt, dt / 2]
    tilt = Rotation.from_euler('xyz', [deg, -deg, deg], degrees=True)
    r = Rotation.from_quat(cio.quaternion(cal)) * tilt
    return make_cal(t=tuple(t), q=tuple(r.as_quat()))


def test_true_calibration_keeps_the_marker_still():
    g, m = make_scene()
    spread, rms = ev.evaluate(g, m, OFFSET, true_calibration())
    assert max(spread) < 1e-6
    assert rms < 1e-6


def test_wrong_calibration_makes_the_marker_slide():
    g, m = make_scene()
    spread, rms = ev.evaluate(g, m, OFFSET, perturbed(true_calibration()))
    assert max(spread) > 5.0
    assert rms > 1.0


def test_fit_recovers_the_true_calibration():
    g, m = make_scene()
    start = perturbed(true_calibration())
    estimate = ev.fit(g, m, OFFSET, start)
    dist_mm, angle_deg = cio.delta(estimate['calibration'], true_calibration())
    assert dist_mm < 0.5
    assert angle_deg < 0.1
    assert estimate['rms_mm'] < 0.5
    assert {v['split'] for v in estimate['validation']} == {'alternating', 'temporal'}
    assert estimate['calibration']['parent_frame_id'] == 'panda_link8'


def test_load_capture_round_trip(tmp_path):
    g, m = make_scene(n=12)

    def lists(matrix):
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        return list(matrix[:3, 3]), list(q)

    samples = []
    for gi, mi in zip(g, m):
        (ee_t, ee_q), (mk_t, mk_q) = lists(gi), lists(mi)
        samples.append({'ee_t': ee_t, 'ee_q': ee_q, 'marker_t': mk_t, 'marker_q': mk_q})
    off_t, off_q = lists(OFFSET)
    path = tmp_path / 'capture.json'
    path.write_text(json.dumps({'camera_offset': {'t': off_t, 'q': off_q}, 'samples': samples}))

    g2, m2, offset2 = ev.load_capture(str(path))
    assert np.allclose(g, g2) and np.allclose(m, m2) and np.allclose(OFFSET, offset2)


def test_capture_with_too_few_samples_is_refused(tmp_path):
    path = tmp_path / 'capture.json'
    sample = {'ee_t': [0, 0, 0], 'ee_q': [0, 0, 0, 1],
              'marker_t': [0, 0, 1], 'marker_q': [0, 0, 0, 1]}
    path.write_text(json.dumps({'camera_offset': {'t': [0, 0, 0], 'q': [0, 0, 0, 1]},
                                'samples': [sample] * 3}))
    with pytest.raises(cio.CalibrationError):
        ev.load_capture(str(path))


# ---------------------------------------------------------------------------
# install_hand_eye
# ---------------------------------------------------------------------------

PROFILE = {'end_effector_frame': 'panda_link8'}


class Setup:
    def __init__(self, tmp_path, current=True, new=None):
        self.target = str(tmp_path / 'installed' / 'hand_eye_transform_fr3_real.yaml')
        self.source = str(tmp_path / 'result.yaml')
        self.history = str(tmp_path / 'history')
        os.makedirs(os.path.dirname(self.target))
        if current:
            write_yaml(self.target, make_cal())
        write_yaml(self.source, new if new is not None else make_cal(t=(0.050, -0.059, 0.019)))
        self.log = []

    def run(self, profile=PROFILE, robot='fr3_real', **kwargs):
        return install_hand_eye.install(self.source, self.target, self.history, profile, robot,
                                        out=self.log.append, **kwargs)

    def text(self):
        return '\n'.join(self.log)


def test_dry_run_writes_nothing(tmp_path):
    s = Setup(tmp_path)
    before = open(s.target).read()
    assert s.run(dry_run=True) == 0
    assert open(s.target).read() == before
    assert not os.path.exists(s.history)
    assert '5.0 mm' in s.text()


def test_install_replaces_the_file_and_keeps_a_backup(tmp_path):
    s = Setup(tmp_path)
    old = cio.load_calibration(s.target)
    assert s.run() == 0
    assert cio.translation(cio.load_calibration(s.target))[0] == pytest.approx(0.050)
    assert cio.quaternion_norm(cio.load_calibration(s.target)) == pytest.approx(1.0)
    backups = os.listdir(s.history)
    assert len(backups) == 1 and backups[0].startswith('hand_eye_transform_fr3_real_')
    assert cio.load_calibration(os.path.join(s.history, backups[0])) == old


def test_first_installation_needs_no_backup(tmp_path):
    s = Setup(tmp_path, current=False)
    assert s.run() == 0
    assert os.path.exists(s.target)
    assert not os.path.exists(s.history)


def test_install_goes_through_a_symlink(tmp_path):
    s = Setup(tmp_path, current=False)
    real = tmp_path / 'source_tree' / 'cal.yaml'
    real.parent.mkdir()
    write_yaml(real, make_cal())
    os.symlink(real, s.target)
    assert s.run() == 0
    assert os.path.islink(s.target)
    assert cio.translation(cio.load_calibration(str(real)))[0] == pytest.approx(0.050)
    assert 'not a symlink' not in s.text()


def test_different_frames_are_refused_unless_forced(tmp_path):
    s = Setup(tmp_path, new=make_cal(child='camera_color_optical_frame'))
    before = open(s.target).read()
    assert s.run() == 1
    assert open(s.target).read() == before
    assert s.run(force=True) == 0


def test_parent_frame_must_match_the_profile(tmp_path):
    s = Setup(tmp_path, new=make_cal(parent='tool0'))
    assert s.run() == 1
    assert 'panda_link8' in s.text()


def test_huge_change_is_refused_unless_forced(tmp_path):
    s = Setup(tmp_path, new=make_cal(t=(0.5, 0.0, 0.0)))
    assert s.run() == 1
    assert s.run(force=True) == 0


def test_simulated_robot_points_to_the_real_one(tmp_path):
    s = Setup(tmp_path)
    assert s.run(profile={'hand_eye_tf_from_urdf': True}, robot='fr3') == 1
    assert 'fr3_real' in s.text()


@pytest.mark.parametrize('bad', [
    make_cal(t=(math.nan, 0.0, 0.0)),
    make_cal(q=(0.0, 0.0, 0.0, 2.0)),        # norm 2: corrupted, not just rounded
    {'translation': {'x': 0, 'y': 0, 'z': 0}},
])
def test_bad_source_is_refused(tmp_path, bad):
    s = Setup(tmp_path, new=bad)
    before = open(s.target).read()
    assert s.run() == 1
    assert open(s.target).read() == before


def test_rounded_quaternion_is_accepted_and_normalised(tmp_path):
    s = Setup(tmp_path, new=make_cal(q=(0.656298, -0.265000, 0.659423, 0.253395)))
    assert s.run() == 0
    assert cio.quaternion_norm(cio.load_calibration(s.target)) == pytest.approx(1.0, abs=1e-12)

"""Tests for the pure IK safety checks (no ROS, no Pinocchio needed)."""

import math

from generic_arm_controller.ik_safety import check_target_pose, plan_trajectory_duration
import pytest

POS = (0.4, 0.1, 0.5)
QUAT = (0.0, 1.0, 0.0, 0.0)


def test_pose_in_base_frame_is_accepted():
    assert check_target_pose(POS, QUAT, 'base_link', 'base_link') is None


@pytest.mark.parametrize('frame_id, base_frame', [
    ('', 'base_link'),          # unstamped pose: accepted, the node only warns
    ('base_link', ''),          # frame check disabled
    ('anything', ''),
])
def test_frame_check_can_be_skipped(frame_id, base_frame):
    assert check_target_pose(POS, QUAT, frame_id, base_frame) is None


@pytest.mark.parametrize('frame_id', ['map', 'camera_color_optical_frame', 'fr3_link0'])
def test_pose_in_another_frame_is_refused(frame_id):
    error = check_target_pose(POS, QUAT, frame_id, 'base_link')
    assert error and frame_id in error and 'base_link' in error


@pytest.mark.parametrize('position, quaternion', [
    ((math.nan, 0.0, 0.0), QUAT),
    ((0.0, math.inf, 0.0), QUAT),
    (POS, (0.0, 1.0, 0.0, math.nan)),
    (POS, (-math.inf, 0.0, 0.0, 1.0)),
])
def test_non_finite_pose_is_refused(position, quaternion):
    assert check_target_pose(position, quaternion, 'base_link', 'base_link')


def test_small_move_keeps_the_base_duration():
    duration, error = plan_trajectory_duration(
        [0.0] * 6, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0, max_joint_speed=1.0)
    assert error is None
    assert duration == 3.0


def test_fast_move_is_stretched_to_the_speed_limit():
    duration, error = plan_trajectory_duration(
        [0.0] * 6, [0.0, 2.5, -1.0, 0.0, 0.0, 0.0], 3.0, max_joint_speed=0.5)
    assert error is None
    assert duration == pytest.approx(5.0)  # 2.5 rad at 0.5 rad/s


def test_move_above_the_step_limit_is_refused():
    duration, error = plan_trajectory_duration(
        [0.0] * 6, [0.0, 0.0, 3.5, 0.0, 0.0, 0.0], 3.0, max_joint_step=3.14)
    assert duration is None
    assert '3.500' in error and '3.14' in error


def test_step_limit_uses_the_largest_joint_and_both_directions():
    # joint deltas are 0.1 and |-2.5 - 1.0| = 3.5 (a move in the negative direction)
    _, error = plan_trajectory_duration([0.0, 1.0], [0.1, -2.5], 3.0, max_joint_step=3.0)
    assert error
    _, error = plan_trajectory_duration([0.0, 1.0], [0.1, -2.5], 3.0, max_joint_step=3.6)
    assert error is None


@pytest.mark.parametrize('solution', [[math.nan], [math.inf]])
def test_non_finite_solution_is_refused(solution):
    duration, error = plan_trajectory_duration([0.0], solution, 3.0)
    assert duration is None and error


def test_limits_disabled_by_default():
    duration, error = plan_trajectory_duration([0.0], [100.0], 3.0)
    assert (duration, error) == (3.0, None)


def test_empty_joint_lists():
    assert plan_trajectory_duration([], [], 3.0, 1.0, 1.0) == (3.0, None)

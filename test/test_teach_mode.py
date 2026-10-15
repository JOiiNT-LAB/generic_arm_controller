"""Tests for the pure teach mode logic (no ROS needed)."""

from types import SimpleNamespace

from generic_arm_controller import teach_mode
import pytest

JTC = 'joint_trajectory_controller'


def test_robot_without_teach_mode_is_refused():
    error = teach_mode.check_request('', executing=False)
    assert error and 'not supported' in error


def test_refused_while_a_sequence_runs():
    error = teach_mode.check_request(JTC, executing=True)
    assert error and 'sequence is running' in error


def test_accepted_when_idle():
    assert teach_mode.check_request(JTC, executing=False) is None


def test_controller_state_lookup():
    controllers = [SimpleNamespace(name='franka_robot_state_broadcaster', state='active'),
                   SimpleNamespace(name=JTC, state='inactive')]
    assert teach_mode.controller_state(controllers, JTC) == 'inactive'
    assert teach_mode.controller_state(controllers, 'missing') is None


@pytest.mark.parametrize('enable, state, activate, deactivate', [
    (True, 'active', [], [JTC]),     # teach mode on: deactivate the controller
    (False, 'inactive', [JTC], []),  # teach mode off: activate it again
    (True, 'inactive', [], []),      # already on: nothing to switch
    (False, 'active', [], []),       # already off: nothing to switch
])
def test_plan_switch(enable, state, activate, deactivate):
    assert teach_mode.plan_switch(enable, JTC, state) == (activate, deactivate, None)


def test_plan_switch_with_controller_not_loaded():
    activate, deactivate, error = teach_mode.plan_switch(True, JTC, None)
    assert (activate, deactivate) == ([], []) and 'not loaded' in error


@pytest.mark.parametrize('state', ['unconfigured', 'finalized'])
def test_plan_switch_with_unexpected_state(state):
    activate, deactivate, error = teach_mode.plan_switch(False, JTC, state)
    assert (activate, deactivate) == ([], []) and state in error


def test_describe():
    assert 'ON' in teach_mode.describe(True)
    assert 'OFF' in teach_mode.describe(False)

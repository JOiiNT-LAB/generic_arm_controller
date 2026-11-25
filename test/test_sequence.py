"""Tests for the sequence replay rules (no ROS needed)."""

from generic_arm_controller import sequence
import pytest

MOVE = {'task_type': 'move', 'task_name': 'approach', 'source_frame': 'fr3_link0',
        'transform': {'translation': {'x': 0.1, 'y': 0.2, 'z': 0.3},
                      'rotation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}}
CLOSE = {'task_type': 'gripper', 'command': 'close'}


class FakeLogger:

    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


def replay(entries, failing=None, reason='boom'):
    """Run entries; the step whose label is `failing` fails. Return (result, calls, sleeps)."""
    calls, sleeps = [], []

    def run_step(task_type, entry):
        calls.append((task_type, sequence.step_label(len(calls), entry)))
        if sequence.step_label(0, entry) == failing:
            return False, reason
        return True, ''

    result = sequence.run(entries, run_step, FakeLogger(), sleep=sleeps.append)
    return result, calls, sleeps


def test_all_steps_ok():
    (ok, msg), calls, sleeps = replay([MOVE, CLOSE])
    assert ok and msg == 'Executed 2 tasks: 2 OK, 0 failed.'
    assert calls == [('move', 'approach'), ('gripper', 'close')]
    assert sleeps == [0.4, 1.5]   # default pauses after a move / a gripper step


def test_stops_at_the_first_failure():
    entries = [MOVE, CLOSE, dict(MOVE, task_name='lift')]
    (ok, msg), calls, _ = replay(entries, failing='close', reason='Gripper timeout (10.0s)')
    assert not ok
    assert msg == ("Stopped at step 2/3 ('close'): Gripper timeout (10.0s). "
                   "1 steps OK before it, 1 not executed.")
    assert [c[1] for c in calls] == ['approach', 'close']   # 'lift' never runs


def test_empty_failure_reason_is_reported_as_unknown():
    (ok, msg), _, _ = replay([MOVE], failing='approach', reason='')
    assert not ok and "('approach'): unknown reason." in msg


def test_unrecognized_task_type_fails_without_calling_the_robot():
    (ok, msg), calls, _ = replay([{'task_type': 'dance', 'task_name': 'x'}])
    assert not ok and "Unrecognized task_type 'dance'" in msg
    assert calls == []


def test_sleep_time_overrides_the_default_and_zero_skips_the_pause():
    entries = [dict(MOVE, sleep_time=2), dict(CLOSE, sleep_time=0)]
    (ok, _), _, sleeps = replay(entries)
    assert ok and sleeps == [2.0]


def test_missing_task_type_means_move():
    entry = {k: v for k, v in MOVE.items() if k != 'task_type'}
    (ok, _), calls, _ = replay([entry])
    assert ok and calls == [('move', 'approach')]


@pytest.mark.parametrize('entry, label', [
    (MOVE, 'approach'), (CLOSE, 'close'), ({'task_type': 'move'}, 'Task 4')])
def test_step_label(entry, label):
    assert sequence.step_label(3, entry) == label


def test_pose_from_entry():
    frame, position, quaternion = sequence.pose_from_entry(MOVE)
    assert frame == 'fr3_link0'
    assert position == (0.1, 0.2, 0.3)
    assert quaternion == (1.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize('entry, reason', [
    ({'transform': MOVE['transform']}, 'Missing source_frame'),
    ({'source_frame': 'b', 'transform': {'translation': {'x': 0, 'y': 0, 'z': 0}}},
     "Incomplete transform: missing 'rotation'"),
    ({'source_frame': 'b', 'transform': {'translation': {'x': 0, 'y': 0},
                                         'rotation': MOVE['transform']['rotation']}},
     "Incomplete transform: missing 'z'"),
])
def test_pose_from_entry_reports_what_is_missing(entry, reason):
    with pytest.raises(ValueError, match=reason):
        sequence.pose_from_entry(entry)

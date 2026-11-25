"""Tests for the saved sequence file helpers (no ROS needed)."""

import json
import os

from generic_arm_controller import pose_store
import pytest

MOVE = {'task_type': 'move', 'task_name': 'a'}
GRIP = {'task_type': 'gripper', 'command': 'close'}


@pytest.fixture
def path(tmp_path):
    return str(tmp_path / 'task_result' / 'robot_poses_ws.json')


def test_missing_or_empty_file_is_an_empty_sequence(path):
    assert pose_store.load(path) == []
    os.makedirs(os.path.dirname(path))
    open(path, 'w').close()
    assert pose_store.load(path) == []


def test_append_creates_the_file_and_counts(path):
    assert pose_store.append(path, MOVE) == 1
    assert pose_store.append(path, GRIP) == 2
    assert pose_store.load(path) == [MOVE, GRIP]


def test_append_keeps_changes_made_to_the_file_meanwhile(path):
    # The bug: entries removed by hand came back at the next save.
    pose_store.append(path, MOVE)
    pose_store.append(path, GRIP)
    pose_store.write(path, [GRIP])          # edited by hand: first entry removed
    assert pose_store.append(path, MOVE) == 2
    assert pose_store.load(path) == [GRIP, MOVE]


@pytest.mark.parametrize('content', ['{"not": "a list"}', '[{"task_type": "move"', 'garbage'])
def test_unreadable_file_is_refused_and_left_untouched(path, content):
    os.makedirs(os.path.dirname(path))
    with open(path, 'w') as f:
        f.write(content)
    with pytest.raises(ValueError):
        pose_store.append(path, MOVE)
    with open(path) as f:
        assert f.read() == content


def test_write_keeps_the_file_permissions_and_leaves_no_temp_file(path):
    pose_store.write(path, [MOVE])
    os.chmod(path, 0o644)
    pose_store.write(path, [MOVE, GRIP])
    assert os.stat(path).st_mode & 0o777 == 0o644
    assert os.listdir(os.path.dirname(path)) == ['robot_poses_ws.json']
    with open(path) as f:
        assert json.load(f) == [MOVE, GRIP]

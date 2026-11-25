"""
The saved sequence file (robot_poses_ws.json): read from disk on every change (no ROS).

task_saving_node_complete used to read the file once at startup and rewrite it from that
in-memory copy at every save. An entry edited or removed by hand while the node was running
came back at the next save. Every change now starts from what is on disk. Kept apart from
the node so it can be unit tested without ROS.
"""

import json
import os
import tempfile

# The one sequence file: written by task_saving_node_complete, replayed by
# task_executor_node_complete, checked by check_saved_deltas.
DEFAULT_PATH = os.path.expanduser('~/ros2_ws/src/task_result/robot_poses_ws.json')


def load(path):
    """
    Return the entries saved in path ([] if the file doesn't exist or is empty).

    Raise ValueError if the file isn't a JSON list: writing over it would destroy what is
    there, so the caller must refuse to save instead.
    """
    if not os.path.exists(path):
        return []
    with open(path) as f:
        text = f.read()
    if not text.strip():
        return []
    try:
        entries = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f'{path} is not valid JSON ({e}): fix or move it.') from e
    if not isinstance(entries, list):
        raise ValueError(f'{path} does not contain a list of entries: fix or move it.')
    return entries


def write(path, entries):
    """Replace the content of path; an interruption never leaves a half-written file."""
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    mode = os.stat(path).st_mode & 0o777 if os.path.exists(path) else 0o664
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.' + os.path.basename(path) + '.')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(entries, f, indent=4)
        os.chmod(tmp, mode)  # mkstemp creates it 0600
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def append(path, entry):
    """Append entry to the entries on disk and return how many there are now."""
    entries = load(path)
    entries.append(entry)
    write(path, entries)
    return len(entries)

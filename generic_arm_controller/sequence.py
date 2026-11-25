"""
Pure logic of replaying a saved sequence (no ROS).

task_executor_node_complete runs the steps of robot_poses_ws.json in order: this module holds
the rules (stop at the first failure, pause after each step, result messages) and the parsing
of a saved pose, so they can be unit tested without a robot. The node supplies the function
that actually moves the arm or the gripper.
"""

import time

# Pause after a successful step, overridable per step with "sleep_time" in the JSON.
# After a move: FollowJointTrajectory reports success when the planned trajectory time is
# over, not when the real arm has stopped oscillating; a short pause keeps the next target
# from being sent while it is still settling.
DEFAULT_SLEEP_S = {'move': 0.4, 'gripper': 1.5}


def step_label(index, entry):
    """Name shown in logs and results: task_name, else the gripper command, else 'Task N'."""
    return entry.get('task_name', entry.get('command', f'Task {index + 1}'))


def run(entries, run_step, logger, sleep=time.sleep):
    """
    Run entries in order; return (success, message).

    run_step(task_type, entry) performs one 'move' or 'gripper' step and returns
    (ok, failure_reason). The sequence stops at the first failed step: the following
    steps were taught assuming it succeeded (e.g. closing the gripper after a move that
    never happened). logger needs info() and error() (a ROS logger does).
    """
    total = len(entries)
    done = 0
    for i, entry in enumerate(entries):
        task_type = entry.get('task_type', 'move')
        label = step_label(i, entry)
        logger.info(f"--- [{i + 1}/{total}] type={task_type}  label={label} ---")

        if task_type in DEFAULT_SLEEP_S:
            ok, reason = run_step(task_type, entry)
        else:
            ok, reason = False, f"Unrecognized task_type '{task_type}'"

        if not ok:
            reason = reason or 'unknown reason'
            skipped = total - (i + 1)
            logger.error(f"Sequence stopped at step {i + 1}/{total} ('{label}'): {reason}. "
                         f"{skipped} remaining steps not executed.")
            return False, (f"Stopped at step {i + 1}/{total} ('{label}'): {reason}. "
                           f"{done} steps OK before it, {skipped} not executed.")

        done += 1
        pause = float(entry.get('sleep_time', DEFAULT_SLEEP_S[task_type]))
        if pause > 0.0:
            logger.info(f"Attesa di stabilità per {pause} secondi...")
            sleep(pause)

    logger.info(f"Sequence completed: {done} steps OK.")
    return True, f"Executed {total} tasks: {done} OK, 0 failed."


def pose_from_entry(entry):
    """
    Return (source_frame, (x, y, z), (qx, qy, qz, qw)) of a saved 'move' step.

    Raise ValueError with the failure reason shown to the user if a field is missing.
    """
    if 'source_frame' not in entry:
        raise ValueError('Missing source_frame')
    try:
        t = entry['transform']['translation']
        r = entry['transform']['rotation']
        return (entry['source_frame'], (t['x'], t['y'], t['z']),
                (r['x'], r['y'], r['z'], r['w']))
    except KeyError as e:
        raise ValueError(f'Incomplete transform: missing {e}') from None

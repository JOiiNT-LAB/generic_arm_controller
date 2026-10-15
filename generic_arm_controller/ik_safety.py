"""
Pure safety checks for IK targets and trajectories (no ROS, no Pinocchio).

Kept apart from ik_trajectory_node so they can be unit tested without a robot.
"""

import math


def check_target_pose(position, quaternion, frame_id, base_frame):
    """
    Return an error message if the target pose must be refused, else None.

    position is (x, y, z), quaternion is (x, y, z, w). The node interprets every
    target in base_frame and does no TF lookup, so a pose stamped in another
    frame would silently be read as base-frame coordinates: refuse it instead.
    An empty frame_id or an empty base_frame disables the frame check.
    """
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        return 'Target pose contains NaN or infinite values.'
    if base_frame and frame_id and frame_id != base_frame:
        return (f"Target pose is in frame '{frame_id}' but the IK works in "
                f"'{base_frame}': transform it to '{base_frame}' first.")
    return None


def plan_trajectory_duration(current, solution, base_duration,
                             max_joint_speed=0.0, max_joint_step=0.0):
    """
    Return (duration_s, error) for a joint-space move from current to solution.

    The duration is stretched so that no joint exceeds max_joint_speed (rad/s).
    If a joint has to move more than max_joint_step (rad) the move is refused
    and the error message is returned instead. A limit <= 0 disables that check.
    """
    deltas = [abs(s - c) for c, s in zip(current, solution)]
    if not all(math.isfinite(d) for d in deltas):
        return None, 'Joint solution contains NaN or infinite values.'

    max_delta = max(deltas, default=0.0)
    if max_joint_step > 0.0 and max_delta > max_joint_step:
        return None, (f'Largest joint move is {max_delta:.3f} rad, above the '
                      f'{max_joint_step:g} rad limit: refusing the trajectory.')

    duration = base_duration
    if max_joint_speed > 0.0:
        duration = max(duration, max_delta / max_joint_speed)
    return duration, None

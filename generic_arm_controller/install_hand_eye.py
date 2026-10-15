#!/usr/bin/env python3
"""
Install a hand-eye calibration for a robot, after checking it.

The calibration node (handeye_ros2) writes its result to
<workspace>/calibration_results/hand_eye_calibration_ws_results.yaml. This tool checks it,
shows how far it is from the calibration in use, keeps a backup of the old one and writes
the new one where the launch files read it. That single file feeds the real robot and the
simulation (see CLAUDE.md), so recalibrating once updates both.

Usage:
    ros2 run generic_arm_controller install_hand_eye --robot fr3_real --dry-run
    ros2 run generic_arm_controller install_hand_eye --robot fr3_real
    ros2 run generic_arm_controller install_hand_eye --robot fr3_real --from my_result.yaml

Exit code is 0 on success (or a clean dry run), 1 if the installation was refused.
"""
import argparse
import datetime
import os
import shutil
import sys

from generic_arm_controller.hand_eye import calibration_io as cio

MAX_DELTA_MM = 100.0
MAX_DELTA_DEG = 20.0


def _describe(label, cal):
    t = cio.translation(cal)
    q = cio.quaternion(cal)
    return (f"  {label}: {cal['parent_frame_id']} -> {cal['child_frame_id']}\n"
            f"      translation x {t[0]:.6f}  y {t[1]:.6f}  z {t[2]:.6f}  "
            f"(|t| {1000 * sum(v * v for v in t) ** 0.5:.1f} mm)\n"
            f"      rotation    x {q[0]:.6f}  y {q[1]:.6f}  z {q[2]:.6f}  w {q[3]:.6f}")


def install(source, target, history_dir, profile, robot, force=False, dry_run=False,
            out=print):
    """Install `source` over `target`. Return 0 on success, 1 if refused."""
    if profile.get('hand_eye_tf_from_urdf'):
        out(f"ERROR: robot '{robot}' takes its camera from the robot model, which reads the "
            "calibration of the real robot. Install it for the real robot instead "
            "(e.g. --robot fr3_real): the simulation follows automatically.")
        return 1

    try:
        new = cio.load_calibration(source)
    except cio.CalibrationError as ex:
        out(f'ERROR: {ex}')
        return 1

    norm = cio.quaternion_norm(new)
    if abs(norm - 1.0) > cio.QUATERNION_NORM_TOLERANCE:
        out(f'ERROR: the quaternion has norm {norm:.4f}, too far from 1: the file looks '
            'corrupted rather than just rounded.')
        if not force:
            return 1
    new = cio.normalized(new)

    current = None
    if os.path.exists(target):
        try:
            current = cio.load_calibration(target)
        except cio.CalibrationError as ex:
            out(f'WARNING: the calibration in use is not readable ({ex}): no comparison.')

    ee_frame = profile.get('end_effector_frame')
    problems = []
    if ee_frame and new['parent_frame_id'] != ee_frame:
        problems.append(f"parent_frame_id is '{new['parent_frame_id']}' but the profile of "
                        f"'{robot}' takes poses at '{ee_frame}'")
    if current is not None and not cio.same_frames(new, current):
        problems.append(
            f"frames differ from the calibration in use "
            f"({new['parent_frame_id']} -> {new['child_frame_id']} instead of "
            f"{current['parent_frame_id']} -> {current['child_frame_id']})")
    if current is not None:
        dist_mm, angle_deg = cio.delta(new, current)
        if dist_mm > MAX_DELTA_MM or angle_deg > MAX_DELTA_DEG:
            problems.append(f'the change is very large ({dist_mm:.1f} mm, {angle_deg:.1f} deg; '
                            f'limits {MAX_DELTA_MM:.0f} mm, {MAX_DELTA_DEG:.0f} deg)')

    out(f"Robot '{robot}'")
    if current is not None:
        out(_describe('in use ', current))
    out(_describe('new    ', new))
    if current is not None:
        dist_mm, angle_deg = cio.delta(new, current)
        out(f'  change: {dist_mm:.1f} mm translation, {angle_deg:.2f} deg rotation')

    if problems:
        for p in problems:
            out(('WARNING (forced): ' if force else 'ERROR: ') + p)
        if not force:
            out('Nothing was written. Use --force if this is really what you want.')
            return 1

    if dry_run:
        out('Dry run: nothing was written.')
        return 0

    if current is not None:
        os.makedirs(history_dir, exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        backup = os.path.join(history_dir, f'hand_eye_transform_{robot}_{stamp}.yaml')
        shutil.copyfile(os.path.realpath(target), backup)
        out(f'Backup of the old calibration: {backup}')

    written, is_link = cio.save_calibration(target, new)
    out(f'Installed: {written}')
    if not is_link:
        out('NOTE: the installed file is not a symlink to the source (not a --symlink-install '
            'build): rebuild with "colcon build --packages-select generic_arm_controller".')
    out('Next: relaunch (real: robot:=fr3_real, simulation: robot:=fr3) and check the result '
        'with evaluate_hand_eye on a fresh capture.')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robot', required=True, help='profile name, e.g. fr3_real or ur10e')
    parser.add_argument('--from', dest='source', default=None,
                        help='calibration to install (default: the handeye_ros2 result '
                             'in <workspace>/calibration_results)')
    parser.add_argument('--calibration-dir', default=None,
                        help='directory holding hand_eye_transform_<robot>.yaml '
                             '(default: the installed package, i.e. what the launch reads)')
    parser.add_argument('--workspace-dir', default=None, help='workspace root (default: auto)')
    parser.add_argument('--dry-run', action='store_true', help='show the comparison only')
    parser.add_argument('--force', action='store_true', help='accept frame/size warnings')
    args = parser.parse_args(argv)

    try:
        workspace = args.workspace_dir or cio.workspace_dir()
        profile = cio.load_robot_profile(args.robot)
        source = args.source or os.path.join(
            workspace, 'calibration_results', 'hand_eye_calibration_ws_results.yaml')
        target = cio.calibration_file(args.robot, args.calibration_dir)
        history = os.path.join(workspace, 'calibration_results', 'history')
    except cio.CalibrationError as ex:
        print(f'ERROR: {ex}')
        return 1

    return install(source, target, history, profile, args.robot, args.force, args.dry_run)


if __name__ == '__main__':
    sys.exit(main())

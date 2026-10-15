#!/usr/bin/env python3
"""
Record the data needed to score hand-eye calibrations offline.

For every sample it stores:
  - the flange pose AT THE TIMESTAMP of the marker detection (not "now": the latency of the
    image pipeline would otherwise bias everything),
  - the marker pose in the camera optical frame.
With these any calibration can be evaluated later (evaluate_hand_eye) without installing it.

Base and flange frames come from the robot profile, so it works for the real FR3, the
simulated one and the UR10e. The control stack (with the ArUco node) must be running.

Usage (move the arm by hand or with /target_cartesian_pose, marker always in view):
    ros2 run generic_arm_controller capture_hand_eye --robot fr3_real
"""
import argparse
import datetime
import json
import os
import sys
import time

from generic_arm_controller.hand_eye import calibration_io as cio
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

MIN_SAMPLES = 10
OFFSET_TIMEOUT_S = 8.0


def _tf_to_lists(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robot', required=True, help='profile name, e.g. fr3_real')
    parser.add_argument('--duration', type=float, default=40.0, help='seconds to record')
    parser.add_argument('--period', type=float, default=0.4, help='seconds between samples')
    parser.add_argument('--marker-topic', default='/aruco_single/pose')
    parser.add_argument('--camera-root-frame', default='camera_link',
                        help='root frame of the camera, the calibration child frame')
    parser.add_argument('--out', default=None,
                        help='output JSON (default: <workspace>/calibration_results/captures)')
    parser.add_argument('--workspace-dir', default=None, help='workspace root (default: auto)')
    args, ros_args = parser.parse_known_args(argv)

    try:
        profile = cio.load_robot_profile(args.robot)
        base = profile['base_frame']
        flange = profile['end_effector_frame']
        optical = profile.get('camera_optical_frame', 'camera_color_optical_frame')
        workspace = args.workspace_dir or cio.workspace_dir()
    except (cio.CalibrationError, KeyError) as ex:
        print(f'ERROR: {ex}')
        return 1

    out_path = args.out
    if out_path is None:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        out_path = os.path.join(workspace, 'calibration_results', 'captures',
                                f'{args.robot}_{stamp}.json')

    rclpy.init(args=ros_args)
    node = Node('capture_hand_eye')
    buf = Buffer()
    TransformListener(buf, node)

    marker = {'msg': None}
    node.create_subscription(
        PoseStamped, args.marker_topic, lambda m: marker.__setitem__('msg', m), 10)

    try:
        # Fixed offset camera root -> optical frame (the marker pose is expressed in the
        # optical frame, calibrations are expressed up to the camera root).
        offset = None
        t0 = time.time()
        while time.time() - t0 < OFFSET_TIMEOUT_S and offset is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform(args.camera_root_frame, optical, rclpy.time.Time())
            except TransformException:
                continue
            offset = dict(zip(('t', 'q'), _tf_to_lists(tf)))
        if offset is None:
            print(f"ERROR: no TF '{args.camera_root_frame}' -> '{optical}': is the camera "
                  'driver / simulation running?')
            return 1

        samples = []
        skipped = 0
        print(f"Recording {args.duration:.0f} s on '{args.marker_topic}' "
              f"({base} -> {flange}): move the arm, keep the marker in view")
        t0 = time.time()
        next_sample = t0
        while time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() < next_sample:
                continue
            next_sample = time.time() + args.period

            m = marker['msg']
            if m is None:
                continue
            try:
                ee = buf.lookup_transform(
                    base, flange, rclpy.time.Time.from_msg(m.header.stamp))
            except TransformException:
                skipped += 1
                continue

            ee_t, ee_q = _tf_to_lists(ee)
            samples.append({
                'ee_t': ee_t, 'ee_q': ee_q,
                'marker_t': [m.pose.position.x, m.pose.position.y, m.pose.position.z],
                'marker_q': [m.pose.orientation.x, m.pose.orientation.y,
                             m.pose.orientation.z, m.pose.orientation.w],
            })
            print(f'  {time.time() - t0:4.1f} s  samples {len(samples):3d}', end='\r')
        print()

        if marker['msg'] is None:
            print(f"ERROR: nothing received on '{args.marker_topic}': is the marker in view?")
            return 1
        if len(samples) < MIN_SAMPLES:
            print(f'ERROR: only {len(samples)} samples (skipped {skipped} without a TF at the '
                  f'detection time): too few, keep the marker in view and move more slowly.')
            return 1

        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, 'w') as f:
            json.dump({'robot': args.robot, 'base_frame': base, 'ee_frame': flange,
                       'camera_offset': offset, 'samples': samples}, f)
        print(f'Saved {len(samples)} samples to {out_path} (skipped {skipped} without a TF)')
        print(f'Next: ros2 run generic_arm_controller evaluate_hand_eye '
              f'--capture {out_path} --robot {args.robot}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

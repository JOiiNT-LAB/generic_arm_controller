#!/usr/bin/env python3
"""Automated version of the README's "End-to-End Validation Tutorial
(Move -> Save -> Execute)": moves the arm to a couple of dummy poses,
confirms it actually reached them via /fk_pose, teaches a short
pose+gripper sequence, replays it with /execute_saved_tasks, and
reports PASS/FAIL for every step instead of requiring manual
copy-pasted service calls.

Usage (defaults match the UR10e tutorial poses in the main README):
    ros2 run generic_arm_controller validate_pipeline

For any other robot, pass its base frame and two reachable poses
(find them by jogging /target_cartesian_pose by hand and reading
/fk_pose back, as documented in the README):
    ros2 run generic_arm_controller validate_pipeline \
        --base-frame fr3_link0 --pose1 0.3,0.0,0.5 --pose2 0.3,0.0,0.3

Exit code is 0 if every check passed, 1 otherwise (usable in CI/smoke tests).
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from generic_arm_interfaces.srv import SavePose, GripperCommand

ORIENTATION = (0.0, 1.0, 0.0, 0.0)  # gripper pointing down, same as the README/note.md examples
REACH_TOLERANCE = 0.03   # m
REACH_TIMEOUT = 8.0      # s - profile's trajectory_duration is 3s, leave margin
SETTLE_TIME = 4.0        # extra settle time before trusting a "reached" pose


class ValidationResult:
    def __init__(self):
        self.checks = []

    def record(self, name, ok, detail=""):
        self.checks.append((name, ok, detail))
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {name}" + (f" - {detail}" if detail else ""))

    @property
    def all_ok(self):
        return all(ok for _, ok, _ in self.checks)


class PipelineValidator(Node):
    def __init__(self, base_frame: str):
        super().__init__('pipeline_validator')
        self.base_frame = base_frame
        cb = ReentrantCallbackGroup()

        self.pose_pub = self.create_publisher(PoseStamped, '/target_cartesian_pose', 10)
        self._last_fk = None
        self.create_subscription(PoseStamped, '/fk_pose', self._on_fk, 10, callback_group=cb)

        self.save_pose_client = self.create_client(SavePose, '/save_pose', callback_group=cb)
        self.gripper_client = self.create_client(GripperCommand, '/gripper/command', callback_group=cb)
        self.save_gripper_client = self.create_client(
            GripperCommand, '/save_gripper_action', callback_group=cb)
        self.clear_client = self.create_client(Trigger, '/clear_saved_poses', callback_group=cb)
        self.execute_client = self.create_client(Trigger, '/execute_saved_tasks', callback_group=cb)

    def _on_fk(self, msg):
        self._last_fk = msg

    def wait_for_services(self, timeout=15.0):
        clients = {
            '/save_pose': self.save_pose_client,
            '/gripper/command': self.gripper_client,
            '/save_gripper_action': self.save_gripper_client,
            '/clear_saved_poses': self.clear_client,
            '/execute_saved_tasks': self.execute_client,
        }
        return [name for name, client in clients.items()
                if not client.wait_for_service(timeout_sec=timeout)]

    def send_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = ORIENTATION
        self.pose_pub.publish(msg)

    def wait_until_reached(self, x, y, z, tolerance=REACH_TOLERANCE, timeout=REACH_TIMEOUT):
        deadline = time.time() + timeout
        dist = None
        while time.time() < deadline:
            if self._last_fk is not None:
                p = self._last_fk.pose.position
                dist = math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2)
                if dist <= tolerance:
                    return True, dist
            time.sleep(0.2)
        return False, dist

    @staticmethod
    def call(client, request, timeout=15.0):
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-frame', default='base_link',
                         help='Reference frame for the poses (default: base_link, i.e. UR10e)')
    parser.add_argument('--pose1', default='0.70,0.0,0.80',
                         help='First pose as "x,y,z" (default: UR10e "home"-ish pose)')
    parser.add_argument('--pose2', default='0.70,0.0,0.40',
                         help='Second pose as "x,y,z" (default: UR10e lower pose)')
    args, _ros_args = parser.parse_known_args(remove_ros_args(sys.argv)[1:])
    return args


def _xyz(s):
    return tuple(float(v) for v in s.split(','))


def main():
    rclpy.init()
    args = parse_args()
    pose1 = _xyz(args.pose1)
    pose2 = _xyz(args.pose2)

    node = PipelineValidator(base_frame=args.base_frame)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    result = ValidationResult()

    print("=== Step 0: checking required services are available ===")
    missing = node.wait_for_services(timeout=15.0)
    result.record("all required services available", not missing,
                  f"missing: {missing}" if missing else "all present")

    if not missing:
        print("\n=== Step 1: move + confirm arrival ===")
        for label, (x, y, z) in (("pose1", pose1), ("pose2", pose2)):
            node.send_pose(x, y, z)
            ok, dist = node.wait_until_reached(x, y, z)
            detail = f"distance={dist:.3f}m" if dist is not None else "no /fk_pose received"
            result.record(f"arm reaches {label} {(x, y, z)}", ok, detail)
            time.sleep(SETTLE_TIME)

        print("\n=== Step 2: teach pose1 -> gripper close -> pose2 ===")
        x, y, z = pose1
        node.send_pose(x, y, z)
        time.sleep(SETTLE_TIME)
        res = node.call(node.save_pose_client,
                         SavePose.Request(save_mode=0, task_name='validate_pose1', reference_frame=''))
        result.record("/save_pose pose1", bool(res and res.success), res.message if res else "timeout")

        res = node.call(node.gripper_client, GripperCommand.Request(command='close'))
        result.record("/gripper/command close", bool(res and res.success),
                       res.message if res else "timeout")

        res = node.call(node.save_gripper_client, GripperCommand.Request(command='close'))
        result.record("/save_gripper_action close", bool(res and res.success),
                       res.message if res else "timeout")

        x, y, z = pose2
        node.send_pose(x, y, z)
        time.sleep(SETTLE_TIME)
        res = node.call(node.save_pose_client,
                         SavePose.Request(save_mode=0, task_name='validate_pose2', reference_frame=''))
        result.record("/save_pose pose2", bool(res and res.success), res.message if res else "timeout")

        print("\n=== Step 3: replay the taught sequence ===")
        res = node.call(node.execute_client, Trigger.Request(), timeout=60.0)
        result.record("/execute_saved_tasks replay", bool(res and res.success),
                       res.message if res else "timeout")

        print("\n=== Step 4: cleanup ===")
        res = node.call(node.clear_client, Trigger.Request())
        result.record("/clear_saved_poses cleanup", bool(res and res.success),
                       res.message if res else "timeout")
    else:
        print("Aborting: launch 'arm_gz_bringup bringup.launch.py' first.")

    node.destroy_node()
    rclpy.shutdown()

    print("\n=== SUMMARY ===")
    for name, ok, _detail in result.checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")

    if result.all_ok:
        print("\nALL CHECKS PASSED")
        sys.exit(0)
    else:
        print("\nSOME CHECKS FAILED")
        sys.exit(1)


if __name__ == '__main__':
    main()

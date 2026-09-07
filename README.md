<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/ARISE_logo-dark_mode.png">
  <source media="(prefers-color-scheme: light)" srcset="images/ARISE_logo-light_mode.png">
  <img alt="ARISE logo" src="images/ARISE_logo-light_mode.png" width="300">
</picture>

# Generic Arm Controller - Modular Robotic Control System

## 🌍 About ARISE

ARISE aims towards making industrial HRI more accessible and cost-effective, in particular in healthcare, intra-logistics and manufacturing sectors. These modules hope to present an integration between FIWARE Orion Context Broker and eProsima Vulcanexus to enable context-aware robotic and industrial applications, alongside ROS4HRI as an open-source ROS standard and a set of ROS packages to facilitate the development of Human-Robot Interaction (HRI) capabilities on robots.

---

## 📋 Overview

This package provides a modular and generic control system for robotic arms (UR, Franka, KUKA, etc.) with integration of:
- **Robotic arm control** with forward/inverse kinematics
- **Artificial vision** via RealSense RGB-D
- **ArUco marker recognition** for dynamic picking
- **Gripper control** (QB Softhand Industry, Robotiq, RG2, etc.)
- **Task system** to save and execute movement sequences

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│          robot_vision_ik_traj_setup.launch.py          │
│                   (Main Launcher)                       │
└────────────┬────────────────────────────────────────────┘
             │
    ┌────────┼────────┬──────────┬──────────┐
    │        │        │          │          │
    ▼        ▼        ▼          ▼          ▼
  Arm     RealSense  ArUco    Task      Gripper
 Control   Camera   Detector  Executor   Control
```

### Main Components

| Component | Description | Node/Launch |
|-----------|-------------|------------|
| **Arm Control** | Robot arm control + IK trajectory | `ik_trajectory_node` |
| **Vision** | RealSense RGB-D camera | `rs_launch.py` |
| **ArUco Detection** | Marker recognition | `aruco_ros/single.launch.py` |
| **Task Manager** | Save/execute sequences | `task_saving_node_complete`, `task_executor_node_complete` |
| **Gripper** | Gripper control | `/gripper/command` service |

---

## 🔗 Related Packages

| Package | Repo | Relation |
|---|---|---|
| [`generic_arm_interfaces`](https://github.com/JOiiNT-LAB/generic_arm_interfaces) | own submodule | **Dependency.** Defines the `GripperCommand`/`SavePose` services this package implements (`/gripper/command`, `/save_pose`). Kept in its own repo — not nested here — so other clients can depend on the service contract without pulling in this whole package. |

This package is a self-contained control layer: it only knows about `generic_arm_interfaces`. Anything built on top of it (natural-language interfaces, orchestration, ...) is documented in the top-level [project README](../../../README.md), not here — see modularity note there.

## 📦 Standalone Build

This package depends on `generic_arm_interfaces` (shared `.srv` definitions, see table above). If you clone this repo on its own, outside the `ros2_arise_vulcanexus_V2` workspace, pull it in with [vcstool](https://github.com/dirk-thomas/vcstool) before building:

```bash
vcs import < generic_arm_controller.repos
```

---

## 🚀 Quick Start

`robot_vision_ik_traj_setup.launch.py` is the one launcher this package ships: it starts the robot-agnostic control stack (IK/FK, gripper, task manager) plus optional vision, selecting robot-specific frames/topics purely from `generic_arm_controller/config/robots/<robot>.yaml` via the `robot:=` argument — no per-robot Python launch files needed here.

How you invoke it depends on where the robot is running:

- **Simulation (Gazebo):** don't call it directly — launch it through [`arm_gz_bringup/launch/bringup.launch.py`](../../arm_gz_bringup/launch/bringup.launch.py), which starts the matching Gazebo/`ros2_control` bringup *and* includes this launcher with the same `robot:=` name. See the top-level [Multi-Robot Usage Guide](../../../README.md#-multi-robot-usage-guide) for the full argument list (`enable_realsense`, `enable_qb`, `enable_llm`, `open_chat`).
- **Real hardware:** the robot driver (`ur_robot_driver`, `franka_bringup`, or a separate real-time PC for `fr3_real`) is started independently, then this launcher is called directly — see [Real Hardware Setup](#-real-robot---physical-ur10e) below and the top-level README's [Real Hardware per Robot](../../../README.md#real-hardware-per-robot) section for Franka.

#### Examples (direct invocation, e.g. on real hardware)

```bash
# Default profile (ur10e), no extras
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py

# Franka FR3 profile
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py robot:=fr3

# With RealSense
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py enable_realsense:=true

# With QB Softhand
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py enable_qb:=true

# Complete: robot + vision + gripper
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py \
  robot:=fr3 enable_realsense:=true enable_qb:=true
```

There's no independent `gripper:=` argument: which gripper driver comes up is tied to the robot (`default_gripper_type` in that robot's YAML profile; QB Softhand is opt-in via `enable_qb`). Adding a new robot means adding `config/robots/<robot>.yaml` — see [Adding a New Robot](../../../README.md#adding-a-new-robot) in the top-level README.

---

## 📡 Main Commands

### 1️⃣ Arm Control - Cartesian Movements

Send a target position (x, y, z) and orientation to the arm:

```bash
ros2 topic pub /target_cartesian_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    pose: {position: {x: 0.5, y: 0.2, z: 0.4}, \
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --once
```

**Parameters:**
- `x, y, z`: Position in meters relative to `base_link`
- `orientation`: Quaternion (x, y, z, w) for end-effector orientation

### 2️⃣ Pose Saving

The system supports 3 save modes:

#### Mode 0: Absolute (World Frame)
Saves fixed poses relative to the robot base. Ideal for predefined positions.

```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 0, task_name: 'home_pose', reference_frame: ''}"
```

#### Mode 1: Camera-Relative
Saves poses relative to the camera. Perfect for interacting with objects at variable positions in the FOV.

```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 1, task_name: 'pre_grasp_from_camera', reference_frame: ''}"
```

#### Mode 2: ArUco Marker-Relative
Saves poses relative to an ArUco marker. Enables dynamic picking of labeled objects.

```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 2, task_name: 'pick_aruco_dynamic', reference_frame: 'aruco_marker_frame'}"
```

### 3️⃣ Task Management

#### Execute All Saved Tasks

```bash
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
```

#### Clear All Saved Poses

```bash
ros2 service call /clear_saved_poses std_srvs/srv/Trigger "{}"
```

### 4️⃣ Gripper Control

#### Close

```bash
ros2 service call /gripper/command generic_arm_interfaces/srv/GripperCommand "{command: 'close'}"
```

#### Open

```bash
ros2 service call /gripper/command generic_arm_interfaces/srv/GripperCommand "{command: 'open'}"
```

---

## 🤖 QB Softhand Industry

### Motor Activation

```bash
ros2 service call /qb_softhand_industry_communication_handler/activate_motors \
  qb_softhand_industry_srvs/srv/Trigger "{}"
```

### Direct Commands (Advanced)

**Close completely** (position: 3000)
```bash
ros2 service call /qb_softhand_industry_communication_handler/set_command \
  qb_softhand_industry_srvs/srv/SetCommand \
  "{max_repeats: 1, set_commands: true, position_command: 3000}"
```

**Open completely** (position: 0)
```bash
ros2 service call /qb_softhand_industry_communication_handler/set_command \
  qb_softhand_industry_srvs/srv/SetCommand \
  "{max_repeats: 1, set_commands: true, position_command: 0}"
```

**Range:** 0 (open) to 3500 (closed)

---

## 🎥 Artificial Vision

### RealSense Camera

Launched automatically with `enable_realsense:=true`, or manually:

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Available topics:**
- `/camera/color/image_raw` - RGB image
- `/camera/depth/image_rect_raw` - Depth map
- `/camera/color/camera_info` - Camera intrinsic parameters

### ArUco Recognition

```bash
ros2 launch aruco_ros single.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  marker_size:=0.15 \
  marker_id:=0 \
  camera_frame:=camera_link
```

**Output:** `aruco_marker_frame` - TF of detected marker

---

## 🏭 Gazebo Simulation

### With UR Description + Camera + Robotiq

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py \
  description_package:=tools_config \
  description_file:=ur_camera_robotiq.urdf.xacro \
  ur_type:=ur10e
```

---

## 🔴 Real Robot - Physical UR10e

### Prerequisites

1. **Network Configuration:**
   - PC: IP address `192.168.56.1`
   - UR10e: IP address `192.168.56.100`

2. **UR+ Panel Setup:**
   - Access URCap → External Control
   - Set PC address: `192.168.56.1`

### Launch

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e \
  robot_ip:=192.168.56.100 \
  launch_rviz:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller
```

---

## 📚 Complete Workflow: Dynamic Picking

### Step 1: Start the System

```bash
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py \
  enable_realsense:=true enable_qb:=true
```

### Step 2: Position the ArUco Marker

Position the object with ArUco marker (e.g., ID=0) in the camera's FOV.

### Step 3: Save Approach Poses

**Pre-grasp relative to camera:**
```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 1, task_name: 'pre_grasp', reference_frame: ''}"
```

**Grasp relative to ArUco:**
```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 2, task_name: 'grasp', reference_frame: 'aruco_marker_frame'}"
```

**Absolute home:**
```bash
ros2 service call /save_pose generic_arm_interfaces/srv/SavePose \
  "{save_mode: 0, task_name: 'home', reference_frame: ''}"
```

### Step 4: Execute the Sequence

```bash
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
```

The robot will execute: pre-grasp → grasp → home

---

## 🧪 Automated Validation

Instead of running the workflow above by hand, `validate_pipeline` does move → confirm arrival (via `/fk_pose`) → save → replay → cleanup on its own and prints a PASS/FAIL summary (exit code 1 on failure):

```bash
ros2 run generic_arm_controller validate_pipeline
# or, for a robot other than UR10e:
ros2 run generic_arm_controller validate_pipeline --base-frame fr3_link0 --pose1 0.3,0.0,0.5 --pose2 0.3,0.0,0.3
```

See the main [README's End-to-End Validation Tutorial](../../../README.md#-end-to-end-validation-tutorial-move--save--execute) for the manual, step-by-step equivalent.

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| `[ERROR] Failed to detect ArUco marker` | Increase brightness, check marker ID, verify size |
| `IK not converging` | Target position unreachable, check joint limits |
| `RealSense not found` | Run `ros2 run realsense2_camera list_devices` to verify connection |
| `Gripper not responding` | Verify USB/ethernet communication, run `activate_motors` |

---

## 📁 Package Structure

```
generic_arm_controller/
├── launch/
│   └── robot_vision_ik_traj_setup.launch.py   (control stack: IK/FK, gripper, task manager, vision)
├── generic_arm_controller/
│   ├── ik_trajectory_node.py
│   ├── fk_node.py
│   ├── gripper_manager.py
│   ├── task_saving_node_complete.py
│   ├── task_executor_node_complete.py
│   ├── urdf_loader.py
│   ├── marker_lock.py
│   ├── check_saved_deltas.py
│   └── validate_pipeline.py
├── config/
│   └── robots/
│       ├── ur10e.yaml
│       ├── fr3.yaml
│       └── fr3_real.yaml
└── calibration_results/
    └── hand_eye_transform_<robot>.yaml
```

Robot-agnosticism lives entirely in `config/robots/<robot>.yaml` (`base_frame`, `end_effector_frame`, `joint_names`, `action_server_name`, `default_gripper_type`, IK tuning), read by every node above — there is no per-robot Python code or per-robot launch file in this package. The Gazebo/`ros2_control` side of "swap robot configurations" lives one level up, in the `arm_gz_bringup` package (`arm_gz_bringup/launch/<robot>.launch.py`), orchestrated together with this package's launcher by `arm_gz_bringup/launch/bringup.launch.py` — see the top-level README's [Multi-Robot Usage Guide](../../../README.md#-multi-robot-usage-guide) and [Adding a New Robot](../../../README.md#adding-a-new-robot).

---

## 📖 References

- **UR Driver:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **QB Softhand:** https://index.ros.org/r/qb_softhand_industry/
- **ArUco ROS:** https://github.com/pal-robotics/aruco_ros
- **RealSense:** https://github.com/IntelRealSense/realsense-ros
- **PyIK (Pinocchio):** https://github.com/stack-of-tasks/pinocchio

---

## 📝 License

Apache-2.0

## 👨‍💻 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**Last Updated:** June 2026
**Maintainer:** JOiiNT-LAB

---

<img alt="Co-funded by the European Union" src="images/EN_Co_fundedbytheEU_RGB_Monochrome.png" width="250">

Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or HADEA. Neither the European Union nor the granting authority can be held responsible for them.

# Generic Arm Controller - Modular Robotic Control System

## 📋 Overview

This package provides a modular and generic control system for robotic arms (UR, Franka, KUKA, etc.) with integration of:
- **Robotic arm control** with forward/inverse kinematics
- **Artificial vision** via RealSense RGB-D
- **ArUco marker recognition** for dynamic picking
- **Gripper control** (QB Softhand Industry, Robotiq, RG2, etc.)
- **Task system** to save and execute movement sequences
- **Conversational LLM interface** for natural language commands

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
| **LLM Interface** | Interactive chat | `llm_app/chatlive` |

---

## 🚀 Quick Start

### Option 1: Traditional Launcher (All-in-One)

This is the original `robot_vision_ik_traj_setup.launch.py` - simpler but less modular.

#### Scenario 1: Pure Simulation (without hardware)

```bash
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py
```

#### Scenario 2: With RealSense

```bash
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py enable_realsense:=true
```

#### Scenario 3: With QB Softhand

```bash
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py enable_qb:=true
```

#### Scenario 4: Complete System

```bash
ros2 launch generic_arm_controller robot_vision_ik_traj_setup.launch.py \
  enable_realsense:=true \
  enable_qb:=true
```

---

### Option 2: Modular Launcher (Recommended) 🌟

This is the new `system_setup.launch.py` - modular, scalable, and easier to compose different hardware configurations.

#### With UR10e + Robotiq Gripper

```bash
ros2 launch generic_arm_controller system_setup.launch.py \
  robot:=ur10e \
  gripper:=robotiq
```

#### With UR10e + Robotiq + RealSense

```bash
ros2 launch generic_arm_controller system_setup.launch.py \
  robot:=ur10e \
  gripper:=robotiq \
  enable_realsense:=true
```

#### With UR10e + QB Softhand + RealSense + ArUco

```bash
ros2 launch generic_arm_controller system_setup.launch.py \
  robot:=ur10e \
  gripper:=qb_softhand \
  enable_realsense:=true \
  enable_aruco:=true
```

#### With Custom Target Frame (for different robots)

```bash
# Franka Panda (uses panda_link0 instead of base_link)
ros2 launch generic_arm_controller system_setup.launch.py \
  robot:=franka \
  target_frame:=panda_link0 \
  gripper:=robotiq

# KUKA LBR (uses base instead of base_link)
ros2 launch generic_arm_controller system_setup.launch.py \
  robot:=kuka \
  target_frame:=base \
  gripper:=robotiq
```

---

### Launch Arguments Comparison

| Argument | Launcher 1 | Launcher 2 |
|----------|-----------|-----------|
| `enable_realsense` | ✓ | ✓ |
| `enable_qb` | ✓ | Uses `gripper:=qb_softhand` |
| `robot` | ❌ | ✓ (ur10e, franka, etc.) |
| `gripper` | ❌ | ✓ (robotiq, qb_softhand, rg2) |
| `target_frame` | ✓ | ✓ |
| `enable_aruco` | ✓ | ✓ |

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

## 🗣️ LLM Interface (Natural Commands)

### Interactive Chat

Open a second terminal:

```bash
ros2 run llm_app chatlive
```

**Available Commands:**
- `"Save home pose"` → Save absolute pose
- `"Close gripper"` → Gripper close
- `"Open gripper"` → Gripper open
- `"Execute tasks"` → Execute saved sequence
- `"Clear poses"` → Clear saved sequence

No "go to [x, y, z]" intent exists in the chat — direct Cartesian moves are done via the `/target_cartesian_pose` topic, not natural language.

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
│   ├── robot_vision_ik_traj_setup.launch.py (Traditional all-in-one launcher)
│   ├── system_setup.launch.py (NEW - Modular orchestrator)
│   ├── robots/
│   │   └── ur10e.launch.py (UR10e robot config)
│   ├── grippers/
│   │   ├── robotiq_gripper.launch.py (Robotiq RG2/RG6)
│   │   └── qb_softhand.launch.py (QB Softhand Industry)
│   └── sensors/
│       ├── realsense_sensor.launch.py (RealSense RGB-D)
│       └── aruco_sensor.launch.py (ArUco detection)
├── generic_arm_controller/
│   ├── ik_trajectory_node.py
│   ├── task_saving_node_complete.py
│   ├── task_executor_node_complete.py
│   ├── fk_node.py
│   ├── gripper_manager.py
│   └── ...
├── config/
│   └── ik_trajectory_node_params.yaml
└── calibration_results/
    └── hand_eye_transform.yaml
```

### Launcher Architecture

**Modular Design (NEW `system_setup.launch.py`):**
```
system_setup.launch.py (Orchestrator)
├── robots/ur10e.launch.py
├── grippers/robotiq_gripper.launch.py (or qb_softhand.launch.py)
├── sensors/realsense_sensor.launch.py (optional)
├── sensors/aruco_sensor.launch.py (optional)
├── task_executor_node
├── task_saving_node
└── llm_app (delayed start)
```

This architecture allows you to:
- ✅ Swap robot configurations (e.g., ur10e → franka)
- ✅ Swap gripper types (e.g., robotiq → qb_softhand → rg2)
- ✅ Enable/disable sensors independently
- ✅ Keep logic nodes separate from hardware configuration

---

## 🔧 Configuration

### Parametric Target Frame

The system supports different robot configurations via the `target_frame` parameter:

```bash
# For UR (default: base_link)
ros2 launch generic_arm_controller system_setup.launch.py robot:=ur10e

# For Franka Panda (panda_link0)
ros2 launch generic_arm_controller system_setup.launch.py robot:=ur10e target_frame:=panda_link0

# For KUKA LBR (base)
ros2 launch generic_arm_controller system_setup.launch.py robot:=ur10e target_frame:=base
```

### Adding New Robot Configurations

To add support for a new robot (e.g., Franka):

1. Create `launch/robots/franka.launch.py` with your robot-specific nodes
2. Update `system_setup.launch.py` to include it:
   ```python
   franka_launcher = IncludeLaunchDescription(
       PathJoinSubstitution([...]),
       condition=IfCondition(LaunchConfiguration('robot') == 'franka')
   )
   ```

### Adding New Gripper Types

To add a new gripper (e.g., RG2):

1. Create `launch/grippers/rg2_gripper.launch.py`
2. Update `system_setup.launch.py` to include it
3. Use it:
   ```bash
   ros2 launch generic_arm_controller system_setup.launch.py \
     robot:=ur10e gripper:=rg2
   ```

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

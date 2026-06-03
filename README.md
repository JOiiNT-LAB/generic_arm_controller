# UR10e ROS2 - Sistema di Controllo Integrato

## 📋 Panoramica

Questo pacchetto fornisce un sistema completo di controllo per il robot UR10e con integrazione di:
- **Controllo del braccio robotico** UR10e con cinematica diretta/inversa
- **Visione artificiale** tramite RealSense RGB-D
- **Riconoscimento ArUco** per il picking dinamico
- **Gripper** (QB Softhand Industry o Robotiq)
- **Sistema di task** per salvare ed eseguire sequenze di movimenti
- **Interfaccia conversazionale** con LLM per comandi in linguaggio naturale

---

## 🏗️ Architettura del Sistema

```
┌─────────────────────────────────────────────────────────┐
│          robot_vision_ik_traj_setup.launch.py          │
│                   (Main Launcher)                       │
└────────────┬────────────────────────────────────────────┘
             │
    ┌────────┼────────┬──────────┬──────────┐
    │        │        │          │          │
    ▼        ▼        ▼          ▼          ▼
  UR10e   RealSense  ArUco    Task      Gripper
 Control   Camera   Detector  Executor   Control
```

### Componenti Principali

| Componente | Descrizione | Node/Launch |
|-----------|-----------|------------|
| **UR10e Control** | Controllo braccio + IK trajectory | `ur10e_ik_trajectory_node` |
| **Visione** | RealSense RGB-D camera | `rs_launch.py` |
| **ArUco Detection** | Riconoscimento marker | `aruco_ros/single.launch.py` |
| **Task Manager** | Salva/esegui sequenze | `task_saving_node_complete`, `task_executor_node_complete` |
| **Gripper** | Controllo gripper | Servizio `/gripper_control` |
| **LLM Interface** | Chat interattiva | `llm_app/chatlive` |

---

## 🚀 Avvio Veloce

### Scenario 1: Simulazione Pura (senza hardware)

```bash
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py
```

### Scenario 2: Con RealSense

```bash
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py enable_realsense:=true
```

### Scenario 3: Con QB Softhand

```bash
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py enable_qb:=true
```

### Scenario 4: Sistema Completo

```bash
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py \
  enable_realsense:=true \
  enable_qb:=true
```

---

## 📡 Comandi Principali

### 1️⃣ Controllo del Braccio - Movimenti Cartesiani

Invia una posizione target (x, y, z) e orientamento al braccio:

```bash
ros2 topic pub /target_cartesian_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    pose: {position: {x: 0.5, y: 0.2, z: 0.4}, \
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --once
```

**Parametri:**
- `x, y, z`: Posizione in metri rispetto a `base_link`
- `orientation`: Quaternione (x, y, z, w) per l'orientamento end-effector

### 2️⃣ Salvataggio Pose

Il sistema supporta 3 modalità di salvataggio:

#### Mode 0: Assoluto (World Frame)
Salva pose fisse rispetto alla base del robot. Ideale per posizioni pre-definite.

```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 0, task_name: 'home_pose', reference_frame: ''}"
```

#### Mode 1: Relativo alla Telecamera
Salva pose relative alla camera. Perfetto per interagire con oggetti in posizioni variabili nel FOV.

```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 1, task_name: 'pre_grasp_from_camera', reference_frame: ''}"
```

#### Mode 2: Relativo a ArUco Marker
Salva pose relative a un marker ArUco. Consente picking dinamico di oggetti etichettati.

```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 2, task_name: 'pick_aruco_dynamic', reference_frame: 'aruco_marker_frame'}"
```

### 3️⃣ Gestione Task

#### Eseguire Tutti i Task Salvati

```bash
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
```

#### Eliminare Tutte le Pose Salvate

```bash
ros2 service call /clear_saved_poses std_srvs/srv/Trigger "{}"
```

### 4️⃣ Controllo Gripper

#### Chiudere

```bash
ros2 service call /gripper_control ur_msgs/srv/GripperCommand "{command: 'close'}"
```

#### Aprire

```bash
ros2 service call /gripper_control ur_msgs/srv/GripperCommand "{command: 'open'}"
```

---

## 🤖 QB Softhand Industry

### Attivazione Motori

```bash
ros2 service call /qb_softhand_industry_communication_handler/activate_motors \
  qb_softhand_industry_srvs/srv/Trigger "{}"
```

### Comandi Diretti (Advanced)

**Chiudere completamente** (posizione: 3000)
```bash
ros2 service call /qb_softhand_industry_communication_handler/set_command \
  qb_softhand_industry_srvs/srv/SetCommand \
  "{max_repeats: 1, set_commands: true, position_command: 3000}"
```

**Aprire completamente** (posizione: 0)
```bash
ros2 service call /qb_softhand_industry_communication_handler/set_command \
  qb_softhand_industry_srvs/srv/SetCommand \
  "{max_repeats: 1, set_commands: true, position_command: 0}"
```

**Range:** 0 (aperto) a 3500 (chiuso)

---

## 🎥 Visione Artificiale

### RealSense Camera

Lanciata automaticamente con `enable_realsense:=true`, oppure manualmente:

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Topics disponibili:**
- `/camera/color/image_raw` - Immagine RGB
- `/camera/depth/image_rect_raw` - Mappa di profondità
- `/camera/color/camera_info` - Parametri interni camera

### Riconoscimento ArUco

```bash
ros2 launch aruco_ros single.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  marker_size:=0.15 \
  marker_id:=0 \
  camera_frame:=camera_link
```

**Output:** `aruco_marker_frame` - TF del marker rilevato

---

## 🗣️ Interfaccia LLM (Comandi Naturali)

### Chat Interattiva

Apri un secondo terminale:

```bash
ros2 run llm_app chatlive
```

**Comandi Disponibili:**
- `"Salva la posa home"` → Salva pose assoluta
- `"Chiudi la mano"` → Gripper close
- `"Apri la mano"` → Gripper open
- `"Esegui i task"` → Esegui sequenza salvata
- `"Vai a [x, y, z]"` → Movimento cartesiano

---

## 🏭 Gazebo Simulation

### Con descrizione UR + Camera + Robotiq

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py \
  description_package:=tools_config \
  description_file:=ur_camera_robotiq.urdf.xacro \
  ur_type:=ur10e
```

---

## 🔴 Robot Reale - UR10e Fisico

### Prerequisiti

1. **Configurazione di rete:**
   - PC: indirizzo IP `192.168.56.1`
   - UR10e: indirizzo IP `192.168.56.100`

2. **Setup UR+ Panel:**
   - Accedi a URCap → External Control
   - Imposta indirizzo PC: `192.168.56.1`

### Lancio

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e \
  robot_ip:=192.168.56.100 \
  launch_rviz:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller
```

---

## 📚 Workflow Completo: Picking Dinamico

### Step 1: Avvia il Sistema

```bash
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py \
  enable_realsense:=true enable_qb:=true
```

### Step 2: Posiziona il Marker ArUco

Posiziona l'oggetto con marker ArUco (es. ID=0) nel FOV della camera.

### Step 3: Salva Pose di Approccio

**Pre-grasp relativa a camera:**
```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 1, task_name: 'pre_grasp', reference_frame: ''}"
```

**Grasp relativa ad ArUco:**
```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 2, task_name: 'grasp', reference_frame: 'aruco_marker_frame'}"
```

**Home assoluta:**
```bash
ros2 service call /save_pose ur_msgs/srv/SavePose \
  "{save_mode: 0, task_name: 'home', reference_frame: ''}"
```

### Step 4: Esegui la Sequenza

```bash
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
```

Il robot eseguirà: pre-grasp → grasp → home

---

## 🔧 Troubleshooting

| Problema | Soluzione |
|----------|-----------|
| `[ERROR] Failed to detect ArUco marker` | Aumentare luminosità, controllare ID marker, verificare size |
| `IK not converging` | Posizione target non raggiungibile, verificare limiti articolari |
| `RealSense not found` | `ros2 run realsense2_camera list_devices` per verificare connessione |
| `Gripper non risponde` | Verificare comunicazione USB/ethernet, eseguire `activate_motors` |

---

## 📁 Struttura Package

```
ur10e_ros2/
├── launch/
│   └── robot_vision_ik_traj_setup.launch.py
├── src/
│   ├── ur10e_ik_trajectory_node.cpp
│   ├── task_saving_node_complete.cpp
│   └── task_executor_node_complete.cpp
├── msg/
└── srv/
```

---

## 📖 Riferimenti

- **UR Driver:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **QB Softhand:** https://index.ros.org/r/qb_softhand_industry/
- **ArUco ROS:** https://github.com/pal-robotics/aruco_ros
- **RealSense:** https://github.com/IntelRealSense/realsense-ros

---
  frame_id: 'base_link'
pose:
  position:
    x: 0.70
    y: 0.0
    z: 0.25
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.75
    y: 0.10
    z: 0.20
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once
ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.45
    y: 0.0
    z: 0.60
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.50
    y: -0.45
    z: 0.45
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once














ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.70
    y: 0.0
    z: 0.8
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.70
    y: 0.0
    z: 0.3 
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0" --once

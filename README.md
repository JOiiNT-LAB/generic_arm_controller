# ur10e_ros2 Redme provissorio -----

 
 ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.56.100 \
  output_filename:=/home/rosuser/ros2_ws/calibration.yaml

 
## commando per accendere il robot real
```

ros2 launch ur_robot_driver ur_control.launch.py     ur_type:=ur10e     robot_ip:=192.168.56.100     launch_rviz:=true     joint_controller:=scaled_joint_trajectory_controller
```

### NB vai su urcap->external_control e imposta il controllo by 192.168.56.1 (il pc deve avere indirizzo di rete 192.168.56.1)




## commando per vedere posizione la posizione cartesiana

```
ros2 run ur10e_ros2 fk_node 
```


## commando per attivare il controllo basato su traiettoria
```

ros2 run ur10e_ros2 ur10e_ik_trajectory_node 
```


## commando per mandare la posizione
```

 ros2 topic pub /target_cartesian_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: -0.5, y: 0.2, z: 0.702}, orientation: {x: 0.442, y: 0.699, z: -0.331, w: -0.454}}}" --once
```










# CALIBRAZIONE
## lanciare la realsense
```
ros2 launch realsense2_camera rs_launch.py
```

## lanciare il nodo aruco specificare il marker_id in base all'aruco
```
ros2 launch aruco_ros single.launch.py \
    image_topic:=/camera/camera/color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    marker_size:=0.15 \
    marker_id:=257 \
    camera_frame:=camera_link
```

## Avviare il nodo di calibrazione della camera
```
ros2 run my_handeye_python calibrator_node 
```

## avviare  il servizio per prendere le pos
```

ros2 service call /calibration_take_pose std_srvs/srv/Empty '{}'
```

## fare il calcolo della calibrazione 
```
ros2 service call /calibrate std_srvs/srv/Empty {}\ 
```

Una volta creata viene generato un file yaml


## utilizzare quest come laucnhe complessivo 
```
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py 
```


il nodo
```
ros2 run my_ur10e_fk marker_pose_transformer_node
```

fallisce, forse:
-calibrazion errata
-posizione irrangiubile
-calcoli errati nel nodo





Ho creato il nodo task_saving_node_complete, parte con il launch robot_vision_ik_setup.launch.py 




- save_mode: 0 (assoluto): Per posizioni fisse nel mondo, rispetto alla base del robot.
- save_mode: 1 (relativo alla telecamera):
Molto utile con telecamera fissa: Permette al robot di interagire con oggetti in posizioni variabili nel campo visivo della telecamera fissa.
Meno evidente con telecamera eye-in-hand: La posa salvata descrive una relazione piccola e quasi statica tra l'end-effector e la telecamera, che si muovono insieme. Il movimento evidente arriva quando il robot si sposta per inquadrare un nuovo oggetto e poi si posiziona relativamente a quel punto inquadrato.
-save_mode: 2 (relativo a ArUco): Ti permette di definire una posa direttamente relativa a un oggetto specifico (il marker ArUco), indipendentemente da dove si trovi l'oggetto o il robot.


```
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 0, task_name: 'home_pose', reference_frame: ''}"
```
```
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 1, task_name: 'pre_grasp_from_camera', reference_frame: ''}"
```
```
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 2, task_name: 'pick_aruco_dynamic', reference_frame: 'aruco_marker_frame'}"

```

Pulire le pose
```
ros2 service call /clear_saved_poses std_srvs/srv/Trigger "{}"
```




lanciare se non è lanciato dal luanch
```
ros2 run ur10e_ros2 task_executor_node_complete 
```

e successivamente chiamare
```
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
```

per eseguire tutti i task














# Lanciare in modo virtuale ur10e con rviz
usiamo un robot_ip inventato 


#con quello di traiettorie
```
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=127.0.0.1 \
    use_fake_hardware:=true \
    launch_rviz:=true \
    initial_joint_controller:=scaled_joint_trajectory_controller
```
## commando per accendere il robot real
```

ros2 launch ur_robot_driver ur_control.launch.py     ur_type:=ur10e     robot_ip:=192.168.56.100     launch_rviz:=true     joint_controller:=scaled_joint_trajectory_controller
```

Lanciare il nodo di controllo

'''
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py 
'''







per provarlo (IN SIMULAZIONE!!) usare il commando
```

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.8
    y: 0.8
    z: 0.9
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

Implementeazine di un nodo che generi le traiettorie partendo da questo ik_node

```
ros2 topic pub /target_robot_pose geometry_msgs/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.5, y: 0.5, z: 0.9}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
```
ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.70  # Avanti
    y: 0.0
    z: 0.45  # Altezza media
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once
```

```

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.7  # Avanti
    y: 0.3
    z: 0.45  # Altezza media
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once
```



```

ros2 topic pub /target_cartesian_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position:
    x: 0.70
    y: -0.50
    z: 0.65
  orientation:
    x: 0.5  # Combinazione Roll/Yaw estrema
    y: 0.5
    z: 0.5
    w: -0.5" --once
```
###SOFTHAND



### SOFTHAND
Segui questo info  per installare la mano
https://index.ros.org/r/qb_softhand_industry/

attivare la mano
ros2 launch qb_softhand_industry_driver softhand_industry_communication_handler.launch.py 


attivare i motori della mano
ros2 service call   /qb_softhand_industry_communication_handler/activate_motors   qb_softhand_industry_srvs/srv/Trigger "{}"

Si Possono usare i topic ufficiali di qb
chiudere la mano
ros2 service call   /qb_softhand_industry_communication_handler/set_command   qb_softhand_industry_srvs/srv/SetCommand   "{max_repeats: 1, set_commands: true, position_command: 3000}"

Aprire la mano
ros2 service call   /qb_softhand_industry_communication_handler/set_command   qb_softhand_industry_srvs/srv/SetCommand   "{max_repeats: 1, set_commands: true, position_command: 0}"

max 3500 min 0

Aggiunto i servizi al launcher dell ur
```
 ros2 service call /gripper_control ur_msgs/srv/GripperCommand "{command: 'close'}"
 ros2 service call /gripper_control ur_msgs/srv/GripperCommand "{command: 'open'}"
```



### Centrealizzazione
Ho inserito tutto i commandi anche di linguaggio naturale in unico launcher, Questo lancia: LLM app + FK + IK + Task executor + tutto il sistema 
# Avvia senza dispositivi (simulazione)
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py

# Avvia con realsense
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py enable_realsense:=true

# Avvia con qb softhand
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py enable_qb:=true

# Avvia con entrambi
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py enable_realsense:=true enable_qb:=true

Terminale 2 (chat interattiva - in un altro docker exec):
ros2 run voice_command_interpreter nl_savepose_parser
🗣️ Comando:Salva la posa home
Chiudi la mano
Apri la mano
Esegui i task
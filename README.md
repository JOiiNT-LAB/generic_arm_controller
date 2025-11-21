# ur10e_ros2
## Redme provissorio

 
 

 
 commando per accendere il robot
ros2 launch ur_robot_driver ur_control.launch.py     ur_type:=ur10e     robot_ip:=192.168.56.100     launch_rviz:=true     joint_controller:=scaled_joint_trajectory_controller

### vai su urcap->external_control e imposta il controllo by 192.168.56.1 (il pc deve avere indirizzo di rete 192.168.56.1)
 




 
 commando per vedere posizione
 ros2 run ur10e_ros2 fk_node 


 commando per il nodo ik
ros2 run ur10e_ros2 ur10e_ik_trajectory_node 


 
 
 commando per mandare la posizione
 ros2 topic pub /target_cartesian_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: -0.5, y: 0.2, z: 0.702}, orientation: {x: 0.442, y: 0.699, z: -0.331, w: -0.454}}}" --once





andare su 






CALIBRAZIONE
realsense
ros2 launch realsense2_camera rs_launch.py

aruco
ros2 launch aruco_ros single.launch.py     image_topic:=/camera/camera/color/image_raw     camera_info_topic:=/camera/camera/color/camera_info     marker_size:=0.15     marker_id:=257     camera_frame:=camera_link
per avviare il nodo
ros2 run my_handeye_python calibrator_node 
per prendere le pose
ros2 service call /calibration_take_pose std_srvs/srv/Empty '{}'

per calibrare
 ros2 service call /calibrate std_srvs/srv/Empty {}\ 





sto creando il luaunch 
 ros2 launch my_ur10e_fk robot_vision_ik_setup.launch.py 


il nodo
ros2 run my_ur10e_fk marker_pose_transformer_node
fallisce, forse:
-calibrazion errata
-posizione irrangiubile
-calcoli errati nel nodo





Ho creato il nodo PoseSaverNode, parte con il launch robot_vision_ik_setup.launch.py 
con il servizio json
ros2 service call /save_current_pose std_srvs/srv/Trigger "{}"
salvo le pose in file json

pulire le pose
ros2 service call /clear_saved_poses std_srvs/srv/Trigger "{}"





save_mode: 0 (assoluto): Per posizioni fisse nel mondo, rispetto alla base del robot.
save_mode: 1 (relativo alla telecamera):
Molto utile con telecamera fissa: Permette al robot di interagire con oggetti in posizioni variabili nel campo visivo della telecamera fissa.
Meno evidente con telecamera eye-in-hand: La posa salvata descrive una relazione piccola e quasi statica tra l'end-effector e la telecamera, che si muovono insieme. Il movimento evidente arriva quando il robot si sposta per inquadrare un nuovo oggetto e poi si posiziona relativamente a quel punto inquadrato.
save_mode: 2 (relativo a ArUco): Questo è il tuo vero "jolly" per la telecamera eye-in-hand. Ti permette di definire una posa direttamente relativa a un oggetto specifico (il marker ArUco), indipendentemente da dove si trovi l'oggetto o il robot.



ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 0, task_name: 'home_pose', reference_frame: ''}"
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 1, task_name: 'pre_grasp_from_camera', reference_frame: ''}"
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 2, task_name: 'pick_aruco_257', reference_frame: 'aruco_marker_257'}"
ros2 service call /save_pose ur_msgs/srv/SavePose "{save_mode: 2, task_name: 'pick_aruco_257', reference_frame: 'aruco_marker_frame'}"

lanciare 
ros2 run ur10e_ros2 task_executor_node_complete 


e successivamente chiamare
ros2 service call /execute_saved_tasks std_srvs/srv/Trigger "{}"
per eseguire tutti i task














## lancare in modo virtuale ur10e con rviz
usiamo un robot_ip inventato 
#con il position controller
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=127.0.0.1 \
    use_fake_hardware:=true \
    launch_rviz:=true \
    initial_joint_controller:=forward_position_controller
#con quello di traiettorie
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=127.0.0.1 \
    use_fake_hardware:=true \
    launch_rviz:=true \
    initial_joint_controller:=scaled_joint_trajectory_controller


controllo in poisizione diretto con il nodo ik_node_position_controllers.py
prende in ingrsso una cartesian pose e restituisce una posa diretta serve per il real time o teleoperazione
quando i percorsi non sono prestabiliti
lanciare il commando  per avviare il nodo

ros2 run  ur10e_ros2 ik_node_position_controllers
ros2 launch ur10e_ros2 robot_vision_ik_traj_setup.launch.py

per provarlo (IN SIMULAZIONE!!) usare il commaando
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

Implementeazine di un nodo che generi le traiettorie partendo da questo ik_node

 ros2 topic pub /target_robot_pose geometry_msgs/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.5, y: 0.5, z: 0.9}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"


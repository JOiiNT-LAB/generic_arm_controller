from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


# ---------------------------------------------------------------------------
# Helper: legge la calibrazione mano-occhio e crea il static_transform_publisher
# ---------------------------------------------------------------------------
def load_calibration_from_yaml(context):
    calibration_file_path = PathJoinSubstitution([
        FindPackageShare('generic_arm_controller'),
        'calibration_results',
        'hand_eye_transform.yaml'
    ]).perform(context)

    if not os.path.exists(calibration_file_path):
        raise FileNotFoundError(
            f"Calibration file not found: {calibration_file_path}"
        )

    try:
        with open(calibration_file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        trans = calib_data['translation']
        rot   = calib_data['rotation']

        static_tf_args = [
            str(trans['x']), str(trans['y']), str(trans['z']),
            str(rot['x']),   str(rot['y']),   str(rot['z']), str(rot['w']),
            calib_data['parent_frame_id'],
            calib_data['child_frame_id'],
        ]

        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_hand_eye_tf_publisher',
                output='screen',
                arguments=static_tf_args,
            )
        ]
    except Exception as e:
        raise RuntimeError(f"Error loading calibration YAML: {e}")


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------
def generate_launch_description():

    # -----------------------------------------------------------------------
    # Argomenti
    # -----------------------------------------------------------------------
    enable_realsense_arg = DeclareLaunchArgument(
        'enable_realsense',
        default_value='false',
        description='Enable Realsense camera'
    )
    enable_qb_arg = DeclareLaunchArgument(
        'enable_qb',
        default_value='false',
        description='Enable QBSofthand gripper'
    )
    enable_robotiq_gripper_arg = DeclareLaunchArgument(
        'enable_robotiq_gripper',
        default_value='true',  # Cambia a 'false' se vuoi disabilitare la pinza di default
        description='Enable Robotiq gripper'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='Target TF frame for IK trajectory execution'
    )

    # -----------------------------------------------------------------------
    # Nodi hardware / TF — partono subito
    # -----------------------------------------------------------------------

    static_tf_node = OpaqueFunction(function=load_calibration_from_yaml)

    fk_node = Node(
        package='generic_arm_controller',
        executable='fk_node',
        name='fk_node',
        output='screen',
        # respawn=False è il default; lo esplicitiamo così il crash è visibile
        # invece di venire mascherato da un respawn silenzioso.
        respawn=False,
    )

    ik_node = Node(
        package='generic_arm_controller',
        executable='ik_trajectory_node',
        name='ik_trajectory_node',
        output='screen',
        respawn=False,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('generic_arm_controller'),
                'config',
                'ik_trajectory_node_params.yaml',
            ])
        ],
    )

    # -----------------------------------------------------------------------
    # Gripper node
    # FIX 3: il nodo importa qb_softhand_industry_srvs anche quando QB è
    # disabilitato, causando un ImportError se il pacchetto non è nel PATH.
    # Soluzione: la guardia enable_qb qui sotto fa partire il nodo solo se
    # il flag è true; in alternativa (se il nodo serve sempre per RG2) assicura
    # che qb_softhand_industry_srvs sia installato oppure gestisci l'import
    # con un try/except nel codice Python (vedi gripper_manager.py fix sotto).
    # -----------------------------------------------------------------------
    gripper_node = Node(
        package='generic_arm_controller',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        respawn=False,
        # Rimuovi la condition se il nodo deve girare sempre (RG2 standalone).
        # In quel caso devi proteggere l'import QB con try/except nel py.
        # condition=IfCondition(LaunchConfiguration('enable_qb')),
    )

    # -----------------------------------------------------------------------
    # Nodi task — partono dopo gripper e IK
    # -----------------------------------------------------------------------

    task_executor_node = Node(
        package='generic_arm_controller',
        executable='task_executor_node_complete',
        name='task_executor_node_complete',
        output='screen',
        respawn=False,
        parameters=[
            {
                'target_frame': LaunchConfiguration('target_frame'),
            }
        ],
    )

    task_saving_node = Node(
        package='generic_arm_controller',
        executable='task_saving_node_complete',
        name='task_saving_node_complete',
        output='screen',
        respawn=False,
    )

    # -----------------------------------------------------------------------
    # Realsense (opzionale)
    # -----------------------------------------------------------------------
    realsense_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('enable_realsense')),
    )

    # -----------------------------------------------------------------------
    # QB SoftHand (opzionale)
    # -----------------------------------------------------------------------
    qb_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('qb_softhand_industry_driver'),
            'launch',
            'softhand_industry_communication_handler.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('enable_qb')),
    )
    
    # -----------------------------------------------------------------------
    # NUOVO: Spawner Automatico del Controller della Pinza (Simulata/Reale)
    # Gira solo se la pinza viene abilitata tramite l'argomento enable_qb
    # -----------------------------------------------------------------------
    spawn_robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('enable_robotiq_gripper')),
    )

    aruco_detect_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_size': 0.1778,                      # <-- CORRETTO: tieni il valore originale del tuo SDF
            'marker_id': 0,                             # <-- CORRETTO: l'ID del tuo marker è 0
            'camera_frame': 'camera_color_optical_frame', # <-- FONDAMENTALE: indica il frame ottico della telecamera
            'marker_frame': 'aruco_marker',
            'reference_frame': 'camera_color_optical_frame', # <-- FONDAMENTALE: allinea il riferimento al frame ottico
            'dictionary': 10,                           # DICT_ARUCO_ORIGINAL (corretto per i marker standard di Gazebo)
        }],
        remappings=[
            ('/image', '/camera_sensor/realsense_camera/image_raw'),
            ('/camera_info', '/camera_sensor/realsense_camera/camera_info')
        ]
    )
    # -----------------------------------------------------------------------
    # llm_app — FIX 1: deve partire PER ULTIMO.
    # Tutti i servizi di cui ha bisogno (/save_pose, /gripper/command, ecc.)
    # devono essere già registrati quando llm_app fa il wait_for_service.
    #
    # FIX 2 (timing): usiamo TimerAction con un ritardo di 3 secondi per
    # dare tempo ai nodi sopra di completare il loro __init__ e registrare
    # i servizi ROS prima che llm_app inizi a cercarli.
    # Aumenta il valore se vedi ancora timeout nell'avvio.
    # -----------------------------------------------------------------------
    llm_app_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('llm_app'),
            'launch',
            'llm_app.launch.py',
        ])
    )

    llm_app_delayed = TimerAction(
        period=3.0,          # secondi di attesa — aumenta a 5.0 se necessario
        actions=[llm_app_launch],
    )

    # -----------------------------------------------------------------------
    # Ordine di avvio dichiarativo
    # (ROS 2 non garantisce ordine tra Node(), ma il TimerAction garantisce
    # che llm_app parta dopo che i nodi sopra hanno avuto tempo di inizializzarsi)
    # -----------------------------------------------------------------------
    return LaunchDescription([
        # Argomenti
        enable_realsense_arg,
        enable_qb_arg,
        enable_robotiq_gripper_arg,
        target_frame_arg,
        # 1. TF statica calibrazione (non ha dipendenze)
        static_tf_node,

        # 2. Nodi hardware e cinematica
        fk_node,
        ik_node,

        # 3. Gripper
        gripper_node,

        # 4. Nodi task (dipendono da IK e gripper)
        task_executor_node,
        task_saving_node,

        # 5. Periferiche opzionali
        realsense_launch,
        qb_launch,
        spawn_robotiq_gripper_controller,  # <--- INSERITO QUI
        aruco_detect_node,  # <--- INSERITO QUI
        # 6. llm_app — ULTIMO, con ritardo esplicito
        llm_app_delayed,
    ])
























from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml # Importa la libreria YAML


# Funzione per leggere il file YAML di calibrazione e avviare il static_transform_publisher
def load_calibration_from_yaml(context):
    # Costruisci il percorso completo al file YAML di calibrazione
    # Assicurati che 'ur10e_ros2' sia il nome corretto del tuo pacchetto
    calibration_file_path = PathJoinSubstitution([
        FindPackageShare('ur10e_ros2'),
        'calibration_results',
        'hand_eye_transform.yaml'
    ]).perform(context) # .perform(context) è necessario per risolvere le sostituzioni al runtime

    # Verifica che il file esista prima di provare a caricarlo
    if not os.path.exists(calibration_file_path):
        # Se il file non viene trovato, solleva un errore significativo
        raise FileNotFoundError(f"Errore: File di calibrazione non trovato al percorso: {calibration_file_path}")

    try:
        # Apri e carica i dati YAML
        with open(calibration_file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        # Estrai i valori di traslazione e rotazione
        trans = calib_data['translation']
        rot = calib_data['rotation']

        # Prepara gli argomenti per il nodo static_transform_publisher
        # Nota l'ordine: x, y, z, roll, pitch, yaw (o quaternion x,y,z,w), parent_frame, child_frame
        # Qui stai usando i quaternioni (x, y, z, w)
        static_tf_args = [
            str(trans['x']), str(trans['y']), str(trans['z']),
            str(rot['x']), str(rot['y']), str(rot['z']), str(rot['w']),
            calib_data['parent_frame_id'],
            calib_data['child_frame_id']
        ]
        
        # Restituisci una lista di azioni, in questo caso un singolo nodo
        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_hand_eye_tf_publisher',
                output='screen', # Mostra l'output del nodo sulla console
                arguments=static_tf_args
            )
        ]
    except Exception as e:
        # Gestisce eventuali errori durante il caricamento o l'analisi del YAML
        raise RuntimeError(f"Errore durante il caricamento della calibrazione dal file YAML: {e}")

def generate_launch_description():
    return LaunchDescription([
        # --- 1. Avvia il Nodo Static Transform Publisher per la calibrazione ---
        # OpaqueFunction esegue la funzione Python 'load_calibration_from_yaml'
        # e include le azioni (Nodi) che essa restituisce.
        OpaqueFunction(function=load_calibration_from_yaml),

        # --- 2. Avvia il tuo Nodo FK (Forward Kinematics) ---
        Node(
            package='ur10e_ros2', # Assicurati che questo sia il nome esatto del tuo pacchetto
            executable='fk_node', # Nome dell'eseguibile definito in setup.py
            name='fk_node',       # Nome del nodo ROS 2
            output='screen',      # Mostra l'output del nodo sulla console
        ),
        Node(
            package='ur10e_ros2',
            executable='task_executor_node_complete',
            name='task_executor_node_complete', # Usa un nome descrittivo per il nodo
            output='screen',
        ),

        Node(
            package='ur10e_ros2',
            executable='task_saving_node_complete',
            name='task_saving_node_complete', # Usa un nome descrittivo per il nodo
            output='screen',
        ),

        Node(
            package='ur10e_ros2', # Assicurati che questo sia il nome esatto del tuo pacchetto
            executable='ur10e_ik_trajectory_node', # Nome dell'eseguibile definito in setup.py
            name='ur10e_ik_trajectory_node',       # Nome del nodo ROS 2
            output='screen',                      # Mostra l'output del nodo sulla console
        ),

    ])


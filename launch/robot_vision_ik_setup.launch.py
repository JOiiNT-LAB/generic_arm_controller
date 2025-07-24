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

        # --- 3. Avvia il tuo Nodo IK (Inverse Kinematics) ---
        Node(
            package='ur10e_ros2', # Assicurati che questo sia il nome esatto del tuo pacchetto
            executable='ur10e_ik_trajectory_node', # Nome dell'eseguibile definito in setup.py
            name='ur10e_ik_trajectory_node',       # Nome del nodo ROS 2
            output='screen',                      # Mostra l'output del nodo sulla console
        ),

        # --- Puoi aggiungere altri nodi qui se necessario ---
        # Ad esempio, il tuo nodo task_orchestrator
        Node(
            package='ur10e_ros2',
            executable='PoseSaverNode',
            name='PoseSaverNode', # Usa un nome descrittivo per il nodo
            output='screen',
        ),
    ])




# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory # Importa per un metodo alternativo e più robusto
# from transformations import quaternion_from_euler # Potresti aver bisogno di questa libreria per conversioni


# # Funzione per leggere il file YAML e estrarre i parametri
# def load_calibration_from_yaml_function(context): # Rinominato per chiarezza
#     # Percorso al file di calibrazione
#     package_share_directory = get_package_share_directory('ur10e_ros2')
#     calibration_file_path = os.path.join(
#         package_share_directory,
#         'calibration_results',
#         'hand_eye_transform.yaml'
#     )

#     if not os.path.exists(calibration_file_path):
#         raise FileNotFoundError(f"ERRORE: File di calibrazione non trovato: {calibration_file_path}. Assicurati che sia presente e copiato correttamente in install/ur10e_ros2/calibration_results.")

#     try:
#         with open(calibration_file_path, 'r') as f:
#             calib_data = yaml.safe_load(f)

#         trans = calib_data.get('translation')
#         rot = calib_data.get('rotation')
#         parent_frame_id = calib_data.get('parent_frame_id')
#         child_frame_id = calib_data.get('child_frame_id')

#         # Validazione dei dati estratti
#         if not all([trans, rot, parent_frame_id, child_frame_id]):
#             raise ValueError("File YAML di calibrazione incompleto. Mancano 'translation', 'rotation', 'parent_frame_id' o 'child_frame_id'.")

#         # Determina il formato di rotazione (quaternion o RPY)
#         # Supponiamo che il tuo YAML contenga un campo 'format: quaternion' o 'format: rpy'
#         rotation_format = calib_data.get('format', 'quaternion') # Default a quaternion se non specificato

#         if rotation_format == 'quaternion':
#             # Assicurati che i nomi dei campi nel YAML siano 'x', 'y', 'z', 'w'
#             if not all(k in rot for k in ['x', 'y', 'z', 'w']):
#                 raise ValueError("Formato quaternione specificato, ma mancano chiavi (x,y,z,w) in 'rotation' nel YAML.")
#             q_x, q_y, q_z, q_w = rot['x'], rot['y'], rot['z'], rot['w']
#         elif rotation_format == 'rpy':
#             # Assicurati che i nomi dei campi nel YAML siano 'roll', 'pitch', 'yaw'
#             if not all(k in rot for k k in ['roll', 'pitch', 'yaw']):
#                  raise ValueError("Formato RPY specificato, ma mancano chiavi (roll,pitch,yaw) in 'rotation' nel YAML.")
#             # Converti RPY in Quaternione
#             # Avrai bisogno di installare 'transformations' (pip install transformations)
#             # o implementare la tua conversione RPY-Quaternione
#             q_x, q_y, q_z, q_w = quaternion_from_euler(rot['roll'], rot['pitch'], rot['yaw'])
#         else:
#             raise ValueError(f"Formato di rotazione non supportato: {rotation_format}. Usa 'quaternion' o 'rpy'.")


#         static_tf_args = [
#             str(trans['x']), str(trans['y']), str(trans['z']),
#             str(q_x), str(q_y), str(q_z), str(q_w), # Quaternioni
#             parent_frame_id,
#             child_frame_id
#         ]
        
#         return [
#             Node(
#                 package='tf2_ros',
#                 executable='static_transform_publisher',
#                 name='camera_hand_eye_tf_publisher',
#                 output='screen',
#                 arguments=static_tf_args
#             )
#         ]
#     except yaml.YAMLError as e:
#         raise RuntimeError(f"ERRORE: Errore nel parsing del file YAML di calibrazione: {e}")
#     except Exception as e:
#         raise RuntimeError(f"ERRORE: Durante il caricamento della calibrazione dal YAML: {e}")


# def generate_launch_description():
#     return LaunchDescription([
#         # --- 1. Pubblicazione della trasformazione statica da YAML ---
#         OpaqueFunction(function=load_calibration_from_yaml_function),

#         # --- 2. Nodo della Cinematica Diretta (FK) ---
#         Node(
#             package='ur10e_ros2',
#             executable='fk_node', # Questo deve corrispondere all'entry_point in setup.py
#             name='fk_node',       # Nome del nodo, può essere unico
#             output='screen',
#             # parameters=[{'param_name': 'param_value'}] # Esempio di come passare parametri
#         ),

#         # --- 3. Nodo della Cinematica Inversa (IK) / Traiettoria ---
#         Node(
#             package='ur10e_ros2',
#             executable='ur10e_ik_trajectory_node', # Questo deve corrispondere all'entry_point
#             name='ur10e_ik_trajectory_node',       # Nome del nodo
#             output='screen',
#         ),

#         # --- Puoi aggiungere qui gli altri tuoi nodi se necessario ---
#         # Node(
#         #     package='ur10e_ros2',
#         #     executable='ik_node_position_controllers',
#         #     name='ik_node_position_controllers',
#         #     output='screen',
#         # ),
#         # Node(
#         #     package='ur10e_ros2',
#         #     executable='marker_pose_transformer_node',
#         #     name='marker_pose_transformer_node',
#         #     output='screen',
#         # ),
#         # Node(
#         #     package='ur10e_ros2',
#         #     executable='object_grapping_node',
#         #     name='object_grapping_node',
#         #     output='screen',
#         # ),
#         # Node(
#         #     package='ur10e_ros2',
#         #     executable='pose_publisher_node', # Ho usato il nome suggerito per coerenza
#         #     name='pose_publisher_node',
#         #     output='screen',
#         # ),
#     ])
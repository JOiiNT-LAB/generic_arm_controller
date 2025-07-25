import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response 

import json
import os
import time
from std_msgs.msg import Bool 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Importazioni per TF2
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs # <-- QUESTA RIGA È NECESSARIA!

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started. (Executes saved poses, performs TF transformations if needed)')
        
        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        # --- Inizializzazione di TF2 (OBBLIGATORIO ORA) ---
        # Questo nodo si occuperà delle trasformazioni TF.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF2 Buffer and Listener initialized.')

        self.target_pose_publisher = self.create_publisher(PoseStamped, '/target_cartesian_pose', 10)
        self.get_logger().info('Publisher created for /target_cartesian_pose.')

        self.execute_task_service = self.create_service(
            Trigger, 
            'execute_saved_tasks', 
            self.execute_saved_tasks_callback,
            callback_group=self.reentrant_callback_group 
        )
        self.get_logger().info('Service /execute_saved_tasks created.')
        
        self.qos_profile_ik_feedback = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.ik_result_subscription = self.create_subscription(
            Bool,
            '/ik_action_result',
            self.ik_result_callback,
            self.qos_profile_ik_feedback,
            callback_group=self.reentrant_callback_group 
        )
        self.get_logger().info('Subscribed to /ik_action_result with REENTRANT callback group.')

        self.ik_action_result_received = False
        self.ik_action_success = False
        self.saved_poses = []
        self.ik_completion_timeout = 20.0 

        home_dir = os.path.expanduser("~")
        self.ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
        self.task_results_dir_ws = os.path.join(self.ros2_ws_dir, 'task_results')
        self.json_file_path_ws = os.path.join(self.task_results_dir_ws, 'robot_poses_ws.json')
        self.get_logger().info(f"Looking for saved poses in: {self.json_file_path_ws}")

    def ik_result_callback(self, msg: Bool):
        """Callback per il topic /ik_action_result. Semplicemente aggiorna i flag di stato."""
        self.ik_action_result_received = True 
        self.ik_action_success = msg.data     
        
        if self.ik_action_success:
            self.get_logger().info("IK action completed successfully for the last sent pose.")
        else:
            self.get_logger().warn("IK action failed for the last sent pose.")

    def load_poses_from_file(self) -> bool:
        """Carica le pose dal file JSON del workspace."""
        if not os.path.exists(self.json_file_path_ws):
            self.get_logger().error(f"File di pose non trovato: {self.json_file_path_ws}")
            self.saved_poses = []
            return False

        try:
            with open(self.json_file_path_ws, 'r') as f:
                self.saved_poses = json.load(f)
            self.get_logger().info(f"Caricate {len(self.saved_poses)} pose dal file.")
            if not self.saved_poses:
                self.get_logger().warn("Il file delle pose è vuoto. Nessun task da eseguire.")
                return False
            return True
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Errore nel parsing del file JSON '{self.json_file_path_ws}': {e}")
            self.saved_poses = []
            return False
        except Exception as e:
            self.get_logger().error(f"Errore durante il caricamento delle pose: {e}")
            self.saved_poses = []
            return False

    def execute_saved_tasks_callback(self, request: Trigger.Request, response: Trigger_Response):
        """Callback del servizio per avviare l'esecuzione delle pose."""
        self.get_logger().info("Richiesta di esecuzione task ricevuta.")

        if not self.load_poses_from_file():
            response.success = False
            response.message = "Impossibile caricare le pose o il file è vuoto/inesistente."
            self.get_logger().error(response.message)
            return response

        if not self.saved_poses:
            response.success = False
            response.message = "Nessuna posa da eseguire nel file."
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info(f"Inizio esecuzione di {len(self.saved_poses)} task...")

        # Definizione del frame target per la trasformazione
        # Il tuo nodo IK si aspetta sempre le pose in 'base_link'
        target_frame_for_ik = 'base_link' 
        # NOTA: Qui potresti volere 'odom' o un altro frame fisso,
        # a seconda di dove il tuo nodo IK si aspetta i comandi.
        # Ho mantenuto 'base_link' per coerenza con il tuo esempio IK.

        for i, pose_data in enumerate(self.saved_poses):
            task_name = pose_data.get('task_name', f'Unnamed Task {i}')
            self.get_logger().info(f"--- Esecuzione Task {i+1}/{len(self.saved_poses)}: {task_name} ---")

            # Crea il messaggio PoseStamped iniziale usando i dati dal JSON
            current_pose_stamped = PoseStamped()
            current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            try:
                source_frame_from_json = pose_data['source_frame']
                current_pose_stamped.header.frame_id = source_frame_from_json
            except KeyError:
                self.get_logger().error(f"Manca 'source_frame' per il task '{task_name}'. Salto questo task.")
                continue

            try:
                transform_data = pose_data['transform']
                translation = transform_data['translation']
                rotation = transform_data['rotation']

                current_pose_stamped.pose.position.x = translation['x']
                current_pose_stamped.pose.position.y = translation['y']
                current_pose_stamped.pose.position.z = translation['z']

                current_pose_stamped.pose.orientation.x = rotation['x']
                current_pose_stamped.pose.orientation.y = rotation['y']
                current_pose_stamped.pose.orientation.z = rotation['z']
                current_pose_stamped.pose.orientation.w = rotation['w']
            except KeyError as e:
                self.get_logger().error(f"Dati 'transform' incompleti per il task '{task_name}': Manca chiave {e}. Salto questo task.")
                continue

            # --- ESEGUI LA TRASFORMAZIONE TF QUI SE NECESSARIO ---
            # Se la posa è già in base_link, non serve trasformarla.
            # Altrimenti, trasformala in base_link.
            final_pose_for_ik = PoseStamped() # Posa che verrà effettivamente inviata all'IK
            
            if source_frame_from_json == target_frame_for_ik:
                # La posa è già nel frame corretto, usala direttamente
                final_pose_for_ik = current_pose_stamped
                self.get_logger().info(f"Posa '{task_name}' è già nel frame '{target_frame_for_ik}'.")
            else:
                # La posa deve essere trasformata
                try:
                    # Tenta di trasformare la posa dal suo source_frame al target_frame_for_ik ('base_link')
                    # Usiamo il timestamp della posa stessa per il lookup se possibile, altrimenti 'now'
                    now = rclpy.time.Time()
                    self.get_logger().info(f"Tentativo di trasformare posa da '{source_frame_from_json}' a '{target_frame_for_ik}' per task '{task_name}'.")
                    final_pose_for_ik = self.tf_buffer.transform(
                        current_pose_stamped,
                        target_frame_for_ik,
                        timeout=rclpy.duration.Duration(seconds=1.0) # Timeout per il lookup
                    )
                    self.get_logger().info(f"Posa '{task_name}' trasformata con successo in '{target_frame_for_ik}'.")
                except TransformException as ex:
                    self.get_logger().error(
                        f"Impossibile trasformare la posa '{task_name}' da '{source_frame_from_json}' a '{target_frame_for_ik}': {ex}. "
                        "Salto questo task."
                    )
                    continue # Salta al prossimo task se la trasformazione fallisce

            # --- Pubblica la posa target ---
            # Ora la posa pubblicata avrà sempre frame_id = 'base_link'
            self.target_pose_publisher.publish(final_pose_for_ik)
            self.get_logger().info(f"Posa '{task_name}' (finalmente in '{final_pose_for_ik.header.frame_id}') pubblicata su /target_cartesian_pose. In attesa del completamento dell'azione IK...")

            # --- Attendi il completamento dell'azione IK usando i flag e un piccolo sleep ---
            self.ik_action_result_received = False 
            self.ik_action_success = False        
            start_time = self.get_clock().now()
            
            while not self.ik_action_result_received:
                time.sleep(0.01) 
                
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9 
                
                if elapsed_time > self.ik_completion_timeout:
                    self.get_logger().warn(
                        f"Timeout ({self.ik_completion_timeout:.1f}s) while waiting for IK action result for '{task_name}'. "
                        "Proceeding to next task."
                    )
                    break 

            if not self.ik_action_result_received: 
                self.get_logger().warn(f"No IK action result received for '{task_name}'. Assumed failure or communication issue.")
            elif not self.ik_action_success:
                self.get_logger().warn(f"IK action for '{task_name}' failed. Proceeding to next task.")
            else:
                self.get_logger().info(f"IK action for '{task_name}' completed successfully. Moving to the next pose.")

        self.get_logger().info("Esecuzione di tutti i task completata.")
        response.success = True
        response.message = f"Execution of {len(self.saved_poses)} tasks completed."
        return response
    
def main(args=None):
    rclpy.init(args=args)
    task_executor_node = TaskExecutorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(task_executor_node)

    try:
        executor.spin() 
    except KeyboardInterrupt:
        task_executor_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        executor.remove_node(task_executor_node)
        task_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
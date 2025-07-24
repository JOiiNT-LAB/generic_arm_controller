import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response # <-- CORREZIONE QUI: Usa 'Trigger_Response' con underscore, non 'TriggerResponse'

import json
import os
import time
from std_msgs.msg import Bool # <-- NUOVA IMPORTAZIONE
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor 
from rclpy.callback_groups import ReentrantCallbackGroup

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started. (Executes saved poses, waiting for IK action completion)')
        
        # --- NUOVO: Crea un gruppo di callback rientrante ---
        # Questo è il passaggio chiave.
        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        # --- Publisher per il topic /target_cartesian_pose --- 
        self.target_pose_publisher = self.create_publisher(PoseStamped, '/target_cartesian_pose', 10)
        self.get_logger().info('Publisher created for /target_cartesian_pose.')

        # --- Servizio per avviare l'esecuzione dei task ---
        # Assegna il gruppo di callback al servizio
        self.execute_task_service = self.create_service(
            Trigger, 
            'execute_saved_tasks', 
            self.execute_saved_tasks_callback,
            callback_group=self.reentrant_callback_group  # <-- MODIFICA
        )
        self.get_logger().info('Service /execute_saved_tasks created.')
        
        # --- QoS per il feedback dell'azione IK ---
        self.qos_profile_ik_feedback = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- Subscriber per il risultato dell'azione IK ---
        # Assegna lo stesso gruppo di callback alla sottoscrizione
        self.ik_result_subscription = self.create_subscription(
            Bool,
            '/ik_action_result',
            self.ik_result_callback,
            self.qos_profile_ik_feedback,
            callback_group=self.reentrant_callback_group # <-- MODIFICA
        )
        self.get_logger().info('Subscribed to /ik_action_result with REENTRANT callback group.')

        # --- Variabili di stato (invariate) ---
        self.ik_action_result_received = False
        self.ik_action_success = False
        self.saved_poses = []
        self.ik_completion_timeout = 20.0 

        # --- Percorsi file (invariati) ---
        home_dir = os.path.expanduser("~")
        self.ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
        self.task_results_dir_ws = os.path.join(self.ros2_ws_dir, 'task_results')
        self.json_file_path_ws = os.path.join(self.task_results_dir_ws, 'robot_poses_ws.json')
        self.get_logger().info(f"Looking for saved poses in: {self.json_file_path_ws}")


    def ik_result_callback(self, msg: Bool):
        """Callback per il topic /ik_action_result. Semplicemente aggiorna i flag di stato."""
        self.ik_action_result_received = True # Questo flag indica che abbiamo ricevuto un risultato
        self.ik_action_success = msg.data     # Questo flag indica il successo o fallimento dell'IK
        
        if self.ik_action_success:
            self.get_logger().info("IK action completed successfully for the last sent pose.")
        else:
            self.get_logger().warn("IK action failed for the last sent pose.")

    # Il metodo load_poses_from_file non cambia e non è mostrato per brevità.

    def load_poses_from_file(self) -> bool:
        """Carica le pose dal file JSON del workspace. (Il codice è invariato)"""
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

        for i, pose_data in enumerate(self.saved_poses):
            task_name = pose_data.get('task_name', f'Unnamed Task {i}')
            self.get_logger().info(f"--- Esecuzione Task {i+1}/{len(self.saved_poses)}: {task_name} ---")

            # Crea il messaggio PoseStamped (codice invariato)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'

            try:
                position = pose_data['position']
                orientation = pose_data['orientation']

                pose_msg.pose.position.x = position['x']
                pose_msg.pose.position.y = position['y']
                pose_msg.pose.position.z = position['z']

                pose_msg.pose.orientation.x = orientation['x']
                pose_msg.pose.orientation.y = orientation['y']
                pose_msg.pose.orientation.z = orientation['z']
                pose_msg.pose.orientation.w = orientation['w']
            except KeyError as e:
                self.get_logger().error(f"Dati posa incompleti per il task '{task_name}': Manca chiave {e}. Salto questo task.")
                continue

            # --- Pubblica la posa target ---
            self.target_pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Posa '{task_name}' pubblicata su /target_cartesian_pose. In attesa del completamento dell'azione IK...")

            # --- Attendi il completamento dell'azione IK usando i flag e un piccolo sleep ---
            self.ik_action_result_received = False # Resetta il flag prima di ogni invio
            self.ik_action_success = False        # Resetta il flag di successo prima di ogni invio
            start_time = self.get_clock().now()
            
            # Loop di attesa per il risultato dell'azione IK
            # Il MultiThreadedExecutor assicurerà che ik_result_callback venga eseguita
            while not self.ik_action_result_received:
                # Usiamo un piccolo sleep per non saturare la CPU mentre aspettiamo
                # L'executor farà comunque il suo lavoro in background
                time.sleep(0.0001) # Sospende il thread attuale per 10ms
                
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9 
                
                if elapsed_time > self.ik_completion_timeout:
                    self.get_logger().warn(
                        f"Timeout ({self.ik_completion_timeout:.1f}s) while waiting for IK action result for '{task_name}'. "
                        "Proceeding to next task."
                    )
                    break # Esci dal loop di attesa e passa al prossimo task

            # --- Logica dopo l'attesa ---
            if not self.ik_action_result_received: # Se siamo usciti per timeout
                self.get_logger().warn(f"No IK action result received for '{task_name}'. Assumed failure or communication issue.")
                # Puoi aggiungere qui una logica per fermare l'intera sequenza se un task fallisce.
                # Per ora, si procede al prossimo.
            elif not self.ik_action_success:
                self.get_logger().warn(f"IK action for '{task_name}' failed. Proceeding to next task.")
            else:
                self.get_logger().info(f"IK action for '{task_name}' completed successfully. Moving to the next pose.")

        self.get_logger().info("Esecuzione di tutti i task completata.")
        response.success = True
        response.message = f"Execution of {len(self.saved_poses)} tasks completed."
        return response
    


# Assicurati che la tua funzione main() sia configurata per usare MultiThreadedExecutor:
def main(args=None):
    rclpy.init(args=args)
    task_executor_node = TaskExecutorNode()

    # CREAZIONE E AVVIO DEL MULTITHREADED EXECUTOR
    executor = MultiThreadedExecutor()
    executor.add_node(task_executor_node)

    try:
        executor.spin() # L'executor farà girare il nodo su più thread
    except KeyboardInterrupt:
        task_executor_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        executor.remove_node(task_executor_node)
        task_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
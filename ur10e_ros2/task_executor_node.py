import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response # <-- CORREZIONE QUI: Usa 'Trigger_Response' con underscore, non 'TriggerResponse'

import json
import os
import time # Mantenuto per time.sleep in caso di timeout (ma ora usato solo per il timeout IK)
from std_msgs.msg import Bool # <-- NUOVA IMPORTAZIONE

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.get_logger().info('Task Executor Node Started. (Executes saved poses, waiting for IK action completion)')

        # --- Publisher per il topic /target_cartesian_pose ---
        self.target_pose_publisher = self.create_publisher(PoseStamped, '/target_cartesian_pose', 10)
        self.get_logger().info('Publisher created for /target_cartesian_pose.')

        # --- Servizio per avviare l'esecuzione dei task ---
        self.execute_task_service = self.create_service(Trigger, 'execute_saved_tasks', self.execute_saved_tasks_callback)
        self.get_logger().info('Service /execute_saved_tasks created.')

        # --- Subscriber per il risultato dell'azione IK ---
        self.ik_result_subscription = self.create_subscription(
            Bool,
            '/ik_action_result',
            self.ik_result_callback,
            1 # QoS History depth
        )
        self.get_logger().info('Subscribed to /ik_action_result topic for IK action completion status.')

        # --- Flag per gestire l'attesa del risultato IK ---
        self.ik_action_result_received = False
        self.ik_action_success = False

        # --- Lista per contenere le pose caricate ---
        self.saved_poses = []

        # Variabili di configurazione
        # Tempo massimo di attesa per il risultato dell'azione IK
        self.ik_completion_timeout = 15.0 # Secondi: dà al robot il tempo di muoversi e al nodo IK di pubblicare

        # Percorso del file JSON nel workspace (lo stesso del PoseSaverNode)
        home_dir = os.path.expanduser("~")
        self.ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
        self.task_results_dir_ws = os.path.join(self.ros2_ws_dir, 'task_results')
        self.json_file_path_ws = os.path.join(self.task_results_dir_ws, 'robot_poses_ws.json')

        self.get_logger().info(f"Looking for saved poses in: {self.json_file_path_ws}")

    def ik_result_callback(self, msg: Bool):
        """Callback per il topic /ik_action_result."""
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
            self.saved_poses = [] # Assicura che la lista sia vuota
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

    def execute_saved_tasks_callback(self, request: Trigger.Request, response: Trigger_Response): # <-- Modifica qui il type hint
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

            # Crea il messaggio PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link' # Assicurati che il frame_id sia corretto per il tuo IK

            # Estrai i dati di posizione e orientamento dal dizionario
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
                continue # Passa al prossimo task se i dati sono incompleti

            # --- Pubblica la posa target ---
            self.target_pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Posa '{task_name}' pubblicata su /target_cartesian_pose. In attesa del completamento dell'azione IK...")

            # --- Attendi il completamento dell'azione IK ---
            self.ik_action_result_received = False # Resetta il flag prima di ogni invio
            self.ik_action_success = False        # Resetta il flag di successo prima di ogni invio
            start_time = self.get_clock().now()
            
            # Loop di attesa per il risultato dell'azione IK
            # Consenti al nodo di elaborare i callback mentre è in attesa
            while not self.ik_action_result_received:
                rclpy.spin_once(self, timeout_sec=0.1) # Elabora i callback con un piccolo timeout
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9 # Tempo trascorso in secondi
                
                if elapsed_time > self.ik_completion_timeout:
                    self.get_logger().warn(
                        f"Timeout ({self.ik_completion_timeout:.1f}s) while waiting for IK action result for '{task_name}'. "
                        "Proceeding to next task."
                    )
                    break # Esci dal loop di attesa e passa al prossimo task

            if not self.ik_action_result_received: # Se siamo usciti per timeout
                self.get_logger().warn(f"No IK action result received for '{task_name}'. Assumed failure or communication issue.")
                # Decisione: interrompere l'intera sequenza o continuare?
                # Per ora, continuiamo al prossimo task, ma potresti voler mettere un 'return response' qui per fermarti.
            elif not self.ik_action_success:
                self.get_logger().warn(f"IK action for '{task_name}' failed. Proceeding to next task.")
                # Decisione: interrompere l'intera sequenza o continuare?
            else:
                self.get_logger().info(f"IK action for '{task_name}' completed successfully. Moving to the next pose.")

        self.get_logger().info("Esecuzione di tutti i task completata.")
        response.success = True
        response.message = f"Execution of {len(self.saved_poses)} tasks completed."
        return response

def main(args=None):
    rclpy.init(args=args)
    task_executor_node = TaskExecutorNode()
    try:
        rclpy.spin(task_executor_node)
    except KeyboardInterrupt:
        task_executor_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        task_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
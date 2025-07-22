import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger

import numpy as np
import os
import json # Importa per la gestione dei file JSON
from ament_index_python.packages import get_package_share_directory

from .poseManager import PoseManager




class TaskOrchestrator(Node):
    def __init__(self):
        super().__init__('task_orchestrator')
        self.get_logger().info('Task Orchestrator Node Started. (IK Simulated)')

        # Inizializza PoseManager con il nome del pacchetto corretto
        self.pose_manager = PoseManager(
            # Assicurati che 'ur10e_ros2' sia il nome corretto del tuo pacchetto
            file_path=os.path.join(get_package_share_directory('ur10e_ros2'), 'config', 'robot_poses.json'),
            logger=self.get_logger()
        )
        self.get_logger().info(f"PoseManager inizializzato per '{self.pose_manager.file_path}'.")

        # --- Variabili per la posa corrente (dal topic FK del nodo IK) ---
        self.current_fk_pose = None
        self.fk_pose_subscription = self.create_subscription(
            PoseStamped,
            '/ur10e_fk_pose',
            self.fk_pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /ur10e_fk_pose topic for current end-effector pose.')

        # RIMOVIAMO IL CLIENT DI SERVIZIO PER L'IK ESTERNA
        # self.ik_solver_client = self.create_client(SolveIK, 'solve_ik')
        # while not self.ik_solver_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Servizio /solve_ik (da nodo IK) non disponibile, attendo...')
        # self.get_logger().info('Servizio /solve_ik (da nodo IK) disponibile.')
        self.get_logger().warn('Servizio IK esterno disabilitato. La soluzione IK sarà simulata.')


        # --- Action Client per inviare traiettorie al controller del robot ---
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('In attesa del server di azione /scaled_joint_trajectory_controller/follow_joint_trajectory...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server di azione trovato.')

        # --- Servizi esposti da questo nodo (per l'interazione utente) ---
        self.save_pose_service = self.create_service(Trigger, 'save_current_pose', self.save_current_pose_callback)
        self.get_logger().info('Service /save_current_pose created.')
        self.clear_poses_service = self.create_service(Trigger, 'clear_saved_poses', self.clear_saved_poses_callback)
        self.get_logger().info('Service /clear_saved_poses created.')
        self.execute_task_service = self.create_service(Trigger, 'execute_saved_task', self.execute_saved_task_callback)
        self.get_logger().info('Service /execute_saved_task created.')

        # Nomi dei giunti del robot UR10e
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.trajectory_duration = 2.0 # Durata per ogni segmento tra due waypoints

    def fk_pose_callback(self, msg: PoseStamped):
        """
        Callback per il topic /ur10e_fk_pose. Aggiorna la posa cartesiana corrente.
        """
        self.current_fk_pose = msg.pose
        # self.get_logger().debug(f"Aggiornata posa FK: {self.current_fk_pose.position.x:.3f}")

    async def _send_ik_request(self, target_pose: Pose):
        """
        SIMULAZIONE: Invece di chiamare un servizio IK esterno,
        questo metodo restituisce una posa dei giunti fissa o simulata.
        Per una vera registrazione, dovresti registrare i joint_angles direttamente.
        """
        self.get_logger().warn("Simulando la soluzione IK. NON viene chiamato un servizio IK reale.")
        
        # Puoi restituire una posa dei giunti di esempio per test
        # Ad esempio, una posa di "casa" o una posa semplice per la simulazione
        simulated_joint_angles = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]) # Esempio di posa "casa"
        
        # Se vuoi che l'IK "fallisca" in certe condizioni per test
        # if target_pose.position.z < 0: # Esempio di condizione di fallimento
        #     self.get_logger().error("Simulazione IK fallita per posa non valida (Z < 0).")
        #     return None, False

        self.get_logger().info(f"Simulazione IK riuscita. Angoli giunti: {np.round(simulated_joint_angles, 3)}")
        return simulated_joint_angles, True

    async def _send_trajectory_goal(self, start_q: np.ndarray, target_q: np.ndarray):
        """Invia un goal di traiettoria al controller del robot."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point1 = JointTrajectoryPoint()
        point1.positions = start_q.tolist()
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 0
        goal_msg.trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = target_q.tolist()
        point2.time_from_start.sec = int(self.trajectory_duration)
        point2.time_from_start.nanosec = int((self.trajectory_duration - int(self.trajectory_duration)) * 1e9)
        goal_msg.trajectory.points.append(point2)

        self.get_logger().info(f"Invio goal di traiettoria al controller. Durata: {self.trajectory_duration}s")
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        goal_handle = await self._send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error('Goal di traiettoria rifiutato dal server di azione.')
            return False

        self.get_logger().info('Goal di traiettoria accettato. In attesa del risultato...')
        result_response = await goal_handle.get_result_async()

        if result_response.status == rclpy.action.client.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Traiettoria completata con successo!')
            return True
        else:
            self.get_logger().warn(f'Traiettoria fallita con stato: {result_response.status} (Errore: {result_response.result.error_code})')
            return False

    def save_current_pose_callback(self, request, response):
        """
        Servizio ROS2 per salvare la posa cartesiana corrente dell'end-effector.
        Legge la posa dal topic /ur10e_fk_pose.
        """
        if self.current_fk_pose is None:
            response.success = False
            response.message = "Nessuna posa corrente dell'end-effector ricevuta dal nodo IK (/ur10e_fk_pose)."
            self.get_logger().warn(response.message)
            return response

        # Converti il messaggio Pose in un dizionario serializzabile per JSON
        pose_to_save = {
            "position": {
                "x": self.current_fk_pose.position.x,
                "y": self.current_fk_pose.position.y,
                "z": self.current_fk_pose.position.z,
            },
            "orientation": {
                "x": self.current_fk_pose.orientation.x,
                "y": self.current_fk_pose.orientation.y,
                "z": self.current_fk_pose.orientation.z,
                "w": self.current_fk_pose.orientation.w,
            }
        }
        
        self.pose_manager.add_pose(pose_to_save)
        response.success = True
        response.message = f"Posa cartesiana salvata con successo. Totale pose: {len(self.pose_manager.get_poses())}"
        self.get_logger().info(f"Posa cartesiana salvata: X:{self.current_fk_pose.position.x:.3f}, Y:{self.current_fk_pose.position.y:.3f}, Z:{self.current_fk_pose.position.z:.3f}")
        return response

    def clear_saved_poses_callback(self, request, response):
        """Servizio per cancellare tutte le pose salvate."""
        self.pose_manager.clear_poses()
        response.success = True
        response.message = "Tutte le pose salvate sono state eliminate."
        self.get_logger().info("Tutte le pose salvate sono state eliminate.")
        return response

    async def execute_saved_task_callback(self, request, response):
        """
        Servizio per eseguire la sequenza di pose salvate.
        Ogni segmento viene eseguito sequenzialmente.
        """
        saved_cartesian_poses = self.pose_manager.get_poses()
        if not saved_cartesian_poses:
            response.success = False
            response.message = "Nessuna posa salvata da eseguire."
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info(f"Avvio esecuzione task con {len(saved_cartesian_poses)} pose salvate.")
        
        # Otteniamo la posa attuale cartesiana dal topic FK
        if self.current_fk_pose is None:
            response.success = False
            response.message = "Impossibile ottenere la posa cartesiana attuale dal topic FK. Impossibile iniziare il task."
            self.get_logger().error(response.message)
            return response

        # Per la prima transizione, il punto di partenza è la posa cartesiana attuale del robot.
        # Dobbiamo "simulare" la conversione in angoli dei giunti.
        self.get_logger().info("Simulazione IK per la posa di partenza attuale del robot...")
        current_start_q, ik_success = await self._send_ik_request(self.current_fk_pose)
        
        if not ik_success:
            response.success = False
            response.message = "Fallimento simulato nel risolvere l'IK per la posa di partenza attuale del robot. Interrompo il task."
            self.get_logger().error(response.message)
            return response

        task_succeeded = True
        for i, target_cartesian_pose_dict in enumerate(saved_cartesian_poses):
            self.get_logger().info(f"Esecuzione segmento {i+1}/{len(saved_cartesian_poses)}.")

            # Ricostruisci il messaggio Pose dal dizionario salvato
            target_pose_ros = Pose()
            target_pose_ros.position.x = target_cartesian_pose_dict["position"]["x"]
            target_pose_ros.position.y = target_cartesian_pose_dict["position"]["y"]
            target_pose_ros.position.z = target_cartesian_pose_dict["position"]["z"]
            target_pose_ros.orientation.x = target_cartesian_pose_dict["orientation"]["x"]
            target_pose_ros.orientation.y = target_cartesian_pose_dict["orientation"]["y"]
            target_pose_ros.orientation.z = target_cartesian_pose_dict["orientation"]["z"]
            target_pose_ros.orientation.w = target_cartesian_pose_dict["orientation"]["w"]

            self.get_logger().info(f"Posa target cartesiana: X:{target_pose_ros.position.x:.3f}, Y:{target_pose_ros.position.y:.3f}, Z:{target_pose_ros.position.z:.3f}")

            # Chiedi al nodo IK di risolvere per la posa target cartesiana (SIMULATO)
            target_q, ik_success = await self._send_ik_request(target_pose_ros)
            
            if not ik_success:
                self.get_logger().error(f"Fallimento simulato nel risolvere l'IK per la posa target {i+1}. Interrompo il task.")
                task_succeeded = False
                break
            
            # Invia la traiettoria e aspetta il completamento
            segment_success = await self._send_trajectory_goal(current_start_q, target_q)
            
            if not segment_success:
                self.get_logger().error(f"Esecuzione del segmento {i+1} fallita. Interrompo il task.")
                task_succeeded = False
                break
            
            # La posa di partenza per il prossimo segmento sarà la posa target appena raggiunta (in angoli giunti)
            current_start_q = target_q.copy() 

        if task_succeeded:
            response.success = True
            response.message = "Task completato con successo!"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Task fallito durante l'esecuzione di uno o più segmenti."
            self.get_logger().warn(response.message)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    task_orchestrator = TaskOrchestrator()
    try:
        rclpy.spin(task_orchestrator)
    except KeyboardInterrupt:
        task_orchestrator.get_logger().info('Interruzione da tastiera, chiusura.')
    finally:
        task_orchestrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
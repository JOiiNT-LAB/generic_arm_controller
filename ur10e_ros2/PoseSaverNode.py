import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger

import numpy as np
import os
import json
from ament_index_python.packages import get_package_share_directory

from .poseManager import PoseManager # Assumi che PoseManager sia nello stesso pacchetto

class PoseSaverNode(Node): # <-- NOME DEL NODO CAMBIATO QUI
    def __init__(self):
        super().__init__('pose_saver_node') # <-- NOME DEL NODO CAMBIATO QUI
        self.get_logger().info('Pose Saver Node Started. (Saving and Managing Poses)') # <-- LOG AGGIORNATO

        self.pose_manager = PoseManager(
            file_path=os.path.join(get_package_share_directory('ur10e_ros2'), 'config', 'robot_poses.json'),
            logger=self.get_logger()
        )
        self.get_logger().info(f"PoseManager initialized for '{self.pose_manager.file_path}'.") # <-- LOG AGGIORNATO

        self.current_fk_pose = None
        self.fk_pose_subscription = self.create_subscription(
            PoseStamped,
            '/ur10e_fk_pose',
            self.fk_pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /ur10e_fk_pose topic for current end-effector pose.')

        self.get_logger().warn('IK functionality and task execution disabled. Only pose management.') # <-- LOG AGGIORNATO
        self._action_client = None
        self.get_logger().info('Action client for robot control disabled.')

        self.save_pose_service = self.create_service(Trigger, 'save_current_pose', self.save_current_pose_callback)
        self.get_logger().info('Service /save_current_pose created.')
        self.clear_poses_service = self.create_service(Trigger, 'clear_saved_poses', self.clear_saved_poses_callback)
        self.get_logger().info('Service /clear_saved_poses created.')
        self.get_logger().info('Service /execute_saved_task disabled.')

        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.trajectory_duration = 2.0
        
        # --- Aggiunto per la gestione dei nomi dei task ---
        self.task_counter = 0
        self._load_task_counter() # Carica il contatore all'avvio

    def _load_task_counter(self):
        """Carica l'ultimo contatore dei task dal file JSON per mantenere l'univocità."""
        try:
            # Tenta di leggere il file specifico per il workspace
            home_dir = os.path.expanduser("~")
            ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
            task_results_dir_ws = os.path.join(ros2_ws_dir, 'task_results')
            json_file_path_ws = os.path.join(task_results_dir_ws, 'robot_poses_ws.json')

            if os.path.exists(json_file_path_ws):
                with open(json_file_path_ws, 'r') as f:
                    all_poses = json.load(f)
                    if all_poses:
                        # Trova il massimo numero di task esistente
                        max_task_num = 0
                        for pose_data in all_poses:
                            if 'task_name' in pose_data:
                                try:
                                    # Estrai il numero dal nome del task (es. 'task_12' -> 12)
                                    task_num_str = pose_data['task_name'].split('_')[-1]
                                    task_num = int(task_num_str)
                                    if task_num > max_task_num:
                                        max_task_num = task_num
                                except (ValueError, IndexError):
                                    self.get_logger().warn(f"Task name '{pose_data['task_name']}' does not follow expected format (task_N).") # <-- LOG AGGIORNATO
                        self.task_counter = max_task_num + 1
                        self.get_logger().info(f"Task counter restored to: {self.task_counter}") # <-- LOG AGGIORNATO
            else:
                self.get_logger().info("File 'robot_poses_ws.json' not found. Starting task counter from 0.") # <-- LOG AGGIORNATO
        except Exception as e:
            self.get_logger().error(f"Error loading task counter: {e}") # <-- LOG AGGIORNATO
            self.task_counter = 0 # Reimposta a 0 in caso di errore

    def fk_pose_callback(self, msg: PoseStamped):
        self.current_fk_pose = msg.pose

    def save_current_pose_callback(self, request, response):
        if self.current_fk_pose is None:
            response.success = False
            response.message = "No current end-effector pose received from IK node (/ur10e_fk_pose)." # <-- MESSAGGIO AGGIORNATO
            self.get_logger().warn(response.message)
            return response

        # --- Genera un nome di task univoco ---
        task_name = f"task_{self.task_counter}"
        self.task_counter += 1 # Incrementa per il prossimo salvataggio

        pose_to_save = {
            "task_name": task_name, # Aggiungi il nome del task
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
        
        # Primo salvataggio tramite PoseManager (nella directory del pacchetto)
        self.pose_manager.add_pose(pose_to_save)
        
        # --- Secondo salvataggio nella directory del workspace ---
        try:
            home_dir = os.path.expanduser("~")
            ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
            task_results_dir_ws = os.path.join(ros2_ws_dir, 'task_results')
            os.makedirs(task_results_dir_ws, exist_ok=True)
            
            all_current_poses = self.pose_manager.get_poses()

            json_file_path_ws = os.path.join(task_results_dir_ws, 'robot_poses_ws.json')
            
            with open(json_file_path_ws, 'w') as f:
                json.dump(all_current_poses, f, indent=4)
            self.get_logger().info(f"Pose '{task_name}' also saved in (ws results): {json_file_path_ws}") # <-- LOG AGGIORNATO

        except Exception as e:
            self.get_logger().error(f"Error saving JSON file in ws results: {e}") # <-- LOG AGGIORNATO
            response.success = False
            response.message = f"Pose '{task_name}' saved in package but error in workspace save: {e}" # <-- MESSAGGIO AGGIORNATO
            return response

        response.success = True
        response.message = f"Pose '{task_name}' successfully saved in both locations. Total poses: {len(self.pose_manager.get_poses())}" # <-- MESSAGGIO AGGIORNATO
        self.get_logger().info(f"Pose '{task_name}' saved: X:{self.current_fk_pose.position.x:.3f}, Y:{self.current_fk_pose.position.y:.3f}, Z:{self.current_fk_pose.position.z:.3f}") # <-- LOG AGGIORNATO
        return response

    def clear_saved_poses_callback(self, request, response):
        self.pose_manager.clear_poses()

        try:
            home_dir = os.path.expanduser("~")
            ros2_ws_dir = os.path.join(home_dir, 'ros2_ws')
            task_results_dir_ws = os.path.join(ros2_ws_dir, 'task_results')
            json_file_path_ws = os.path.join(task_results_dir_ws, 'robot_poses_ws.json')

            if os.path.exists(json_file_path_ws):
                os.remove(json_file_path_ws)
                self.get_logger().info(f"File '{json_file_path_ws}' removed.") # <-- LOG AGGIORNATO
            else:
                self.get_logger().info(f"File '{json_file_path_ws}' not found, no action taken.") # <-- LOG AGGIORNATO
            
            self.task_counter = 0 # Resetta il contatore quando le pose vengono cancellate

        except Exception as e:
            self.get_logger().error(f"Error removing JSON file in ws results: {e}") # <-- LOG AGGIORNATO
            response.success = False
            response.message = f"All saved poses in package were deleted, but error removing workspace file: {e}" # <-- MESSAGGIO AGGIORNATO
            return response

        response.success = True
        response.message = "All saved poses have been cleared." # <-- MESSAGGIO AGGIORNATO
        self.get_logger().info("All saved poses have been cleared. Task counter reset to 0.") # <-- LOG AGGIORNATO
        return response


def main(args=None):
    rclpy.init(args=args)
    # Crea un'istanza del nodo con il nuovo nome
    pose_saver_node = PoseSaverNode() # <-- ISTANZA DEL NODO CAMBIATA QUI
    try:
        rclpy.spin(pose_saver_node) # <-- NOME ISTANZA CAMBIATO QUI
    except KeyboardInterrupt:
        pose_saver_node.get_logger().info('Keyboard interrupt, shutting down.') # <-- LOG AGGIORNATO
    finally:
        pose_saver_node.destroy_node() # <-- NOME ISTANZA CAMBIATO QUI
        rclpy.shutdown()

if __name__ == '__main__':
    main()
from urllib3 import request
import rclpy
from rclpy.node import Node
import json
import os

# Importazioni per TF2
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Importazione del nostro nuovo servizio e del servizio Trigger standard
from ur_msgs.srv import SavePose
from ur_msgs.srv import GripperCommand

from std_srvs.srv import Trigger # Importa il servizio Trigger standard
from qb_softhand_industry_srvs.srv import SetCommand  # ROS2 Humble

class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')
        self.get_logger().info('Pose Saver Node Started.')

        # --- Inizializzazione di TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF2 Buffer and Listener initialized.')

        # Frame di default
        self.base_frame = 'base_link'
        self.end_effector_frame = 'tool0' # Assicurati che questo sia il nome corretto del frame del tuo EE

        # Gestore delle pose
        self.all_poses = []
        # Percorso assoluto del file JSON direttamente nella cartella src
        # Salva SEMPRE in: ~/ros2_ws/src/task_result/
        workspace_dir = os.path.expanduser('~/ros2_ws/src')
        task_result_dir = os.path.join(workspace_dir, 'task_result')

        # Crea la cartella se non esiste
        os.makedirs(task_result_dir, exist_ok=True)

        self.json_file_path = os.path.join(task_result_dir, 'robot_poses_ws.json')

        self.get_logger().info(f"Saving poses to: {self.json_file_path}")


        self._load_poses()

        # --- Servizio per salvare la posa ---
        self.save_pose_service = self.create_service(
            SavePose, 
            'save_pose',
            self.save_pose_callback
        )
        self.get_logger().info('Service /save_pose created.')

        # --- Nuovo servizio per pulire le pose (utilizzando std_srvs/srv/Trigger) ---
        self.clear_poses_service = self.create_service(
            Trigger, # Usiamo std_srvs/srv/Trigger qui
            'clear_saved_poses', 
            self.clear_poses_callback
        )
        self.get_logger().info('Service /clear_saved_poses created using std_srvs/srv/Trigger.')
        # --- Servizio per aprire/chiudere mano ---
        self.save_gripper_service = self.create_service(
            GripperCommand,
            'gripper_control',
            self.gripper_callback
        )

    def _load_poses(self):
        """Carica le pose esistenti dal file JSON."""
        try:
            if os.path.exists(self.json_file_path):
                with open(self.json_file_path, 'r') as f:
                    self.all_poses = json.load(f)
                self.get_logger().info(f"Loaded {len(self.all_poses)} poses from {self.json_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load poses: {e}")

    def _save_poses_to_file(self):
        """Salva l'intera lista di pose nel file JSON."""
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
            with open(self.json_file_path, 'w') as f:
                json.dump(self.all_poses, f, indent=4)
            self.get_logger().info(f"Successfully saved {len(self.all_poses)} poses to file.")
        except Exception as e:
            self.get_logger().error(f"Failed to save poses to file: {e}")

    def save_pose_callback(self, request: SavePose.Request, response: SavePose.Response):
        target_frame = self.end_effector_frame
        source_frame = ""
        pose_type = ""
            # --- Interpretazione del save_mode numerico ---
        if request.save_mode == 0:
            # EE rispetto a base_link
            pose_type = "absolute"
            source_frame = self.base_frame

        elif request.save_mode == 1:
            # EE rispetto alla camera (frame fisso)
            pose_type = "relative"
            source_frame = "camera_color_optical_frame"

        elif request.save_mode == 2:
            # EE rispetto a un frame ARUCO
            pose_type = "relative"
            source_frame = request.reference_frame

            if not source_frame or source_frame == "":
                response.success = False
                response.message = (
                    "ERROR: save_mode=2 (ARUCO) requires a valid reference_frame."
                )
                self.get_logger().error(response.message)
                return response

        else:
            response.success = False
            response.message = f"ERROR: Invalid save_mode={request.save_mode}. Must be 0, 1, or 2."
            self.get_logger().error(response.message)
            return response

        try:
            # Ottieni la trasformazione T_source->target
            now = rclpy.time.Time()
            transform_stamped = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            
            t = transform_stamped.transform.translation
            r = transform_stamped.transform.rotation

            # Crea il dizionario da salvare
            pose_data = {
                "task_name": request.task_name,
                "type": pose_type,
                "source_frame": source_frame,
                "target_frame": target_frame,
                "transform": {
                    "translation": {"x": t.x, "y": t.y, "z": t.z},
                    "rotation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w}
                }
            }
            
            # Aggiungi la posa alla lista e salva su file
            self.all_poses.append(pose_data)
            self._save_poses_to_file()

            response.success = True
            response.message = f"Successfully saved pose '{request.task_name}' ({pose_type}: {target_frame} w.r.t. {source_frame})."
            self.get_logger().info(response.message)

        except TransformException as ex:
            response.success = False
            response.message = f"Could not transform '{target_frame}' to '{source_frame}': {ex}"
            self.get_logger().error(response.message)
        
        return response

    def clear_poses_callback(self, request: Trigger.Request, response: Trigger.Response):
        """
        Callback per il servizio di pulizia delle pose utilizzando std_srvs/srv/Trigger.
        Resetta la lista delle pose e svuota il file JSON.
        """
        self.all_poses = []
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True) # Assicurati che la directory esista
            with open(self.json_file_path, 'w') as f:
                json.dump([], f) # Scrive un array vuoto nel file
            response.success = True
            response.message = "All saved poses have been cleared."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to clear poses: {e}"
            self.get_logger().error(response.message)
        return response
    def gripper_callback(self, request, response):
        """Apri o chiudi la mano in base al comando."""
        self.get_logger().info(f"Gripper command received: {request.command}")

        client = self.create_client(SetCommand, '/qb_softhand_industry_communication_handler/set_command')
        if not client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = "QB SoftHand service non disponibile!"
            return response

        req = SetCommand.Request()
        req.max_repeats = 1
        req.set_commands = True

        if request.command.lower() == "close":
            req.position_command = 3500
            state_str = "closed"
        elif request.command.lower() == "open":
            req.position_command = 0
            state_str = "open"
        else:
            response.success = False
            response.message = f"Comando gripper non valido: {request.command}"
            return response

        # Chiamata asincrona
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._gripper_done_callback(f, state_str))

        response.success = True
        response.message = f"Comando gripper '{request.command}' inviato!"
        return response

    def _gripper_done_callback(self, future, state_str):
        """Callback quando QB SoftHand ha terminato l'azione"""
        try:
            result = future.result()
            self.get_logger().info(f"QB SoftHand eseguito: {result}")

            # Salva sul JSON lo stato corretto (open o closed)
            gripper_entry = {
                "type": "gripper",
                "state": state_str,
                "position_command": 0 if state_str=="open" else 3500
            }
            self.all_poses.append(gripper_entry)
            self._save_poses_to_file()
            self.get_logger().info(f"Stato gripper '{state_str}' salvato sul JSON.")
        except Exception as e:
            self.get_logger().error(f"Errore eseguendo QB SoftHand: {e}")


# Funzione main (invariata)
def main(args=None):
    rclpy.init(args=args)
    pose_saver_node = PoseSaverNode()
    try:
        rclpy.spin(pose_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        pose_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
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
from std_srvs.srv import Trigger # Importa il servizio Trigger standard

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
        self.json_file_path = os.path.join(os.path.expanduser("~"), 'ros2_ws', 'task_results', 'robot_poses_ws.json')
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
            'clear_saved_poses', # Ho usato il nome che hai provato prima, per coerenza
            self.clear_poses_callback
        )
        self.get_logger().info('Service /clear_saved_poses created using std_srvs/srv/Trigger.')

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
        
        if request.save_mode == SavePose.Request.SAVE_MODE_EE_TO_BASE:
            pose_type = "absolute"
            source_frame = self.base_frame
        elif request.save_mode == SavePose.Request.SAVE_MODE_EE_TO_CAMERA:
            pose_type = "relative"
            source_frame = "camera_color_optical_frame" # Assicurati che il nome del frame sia corretto
        elif request.save_mode == SavePose.Request.SAVE_MODE_EE_TO_ARUCO:
            pose_type = "relative"
            source_frame = request.reference_frame
            if not source_frame:
                response.success = False
                response.message = "Error: A 'reference_frame'  is required for ARUCO mode."
                self.get_logger().error(response.message)
                return response
        else:
            response.success = False
            response.message = f"Error: Invalid save_mode '{request.save_mode}'."
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
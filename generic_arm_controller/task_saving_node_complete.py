import rclpy
from rclpy.node import Node
import json
import os
import datetime
import shutil  # <-- Aggiunto per gestire i backup dei file completi

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ur_msgs.srv import SavePose
from ur_msgs.srv import GripperCommand as GripperSrv
from std_srvs.srv import Trigger


class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')
        self.get_logger().info('Pose Saver Node Started.')

        # Declare ROS2 parameters for flexibility across different robots
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'tool0')

        # Read parameters from ROS2 launch configuration
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.all_poses          = []

        # Manteniamo la tua cartella e il tuo file originale (L'esecutore leggerà sempre questo!)
        workspace_dir    = os.path.expanduser('~/ros2_ws/src')
        self.task_result_dir  = os.path.join(workspace_dir, 'task_result')
        os.makedirs(self.task_result_dir, exist_ok=True)
        self.json_file_path = os.path.join(self.task_result_dir, 'robot_poses_ws.json')
        self.get_logger().info(f"Saving to: {self.json_file_path}")

        # Carica le pose accumulate in precedenza se il file esiste già
        self._load_poses()

        self.save_pose_service = self.create_service(
            SavePose, 'save_pose', self.save_pose_callback
        )

        self.save_gripper_service = self.create_service(
            GripperSrv, 'save_gripper_action', self.save_gripper_action_callback
        )

        self.clear_poses_service = self.create_service(
            Trigger, 'clear_saved_poses', self.clear_poses_callback
        )

        self.get_logger().info(
            'Services ready: /save_pose, /save_gripper_action, /clear_saved_poses'
        )

    # ------------------------------------------------------------------
    # I/O file JSON (Invariati per garantire l'accumulo nello stesso file)
    # ------------------------------------------------------------------

    def _load_poses(self):
        try:
            if os.path.exists(self.json_file_path):
                with open(self.json_file_path, 'r') as f:
                    self.all_poses = json.load(f)
                self.get_logger().info(
                    f"Loaded {len(self.all_poses)} entries. Nuovi salvataggi saranno accodati."
                )
        except Exception as e:
            self.get_logger().error(f"Failed to load poses: {e}")

    def _save_poses_to_file(self):
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
            with open(self.json_file_path, 'w') as f:
                json.dump(self.all_poses, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to save to file: {e}")

    # ------------------------------------------------------------------
    # Callback: salva posa robot (Accumula nel file principale)
    # ------------------------------------------------------------------

    def save_pose_callback(self, request: SavePose.Request, response: SavePose.Response):
        target_frame = self.end_effector_frame
        source_frame = ""
        pose_type    = ""

        if request.save_mode == 0:
            pose_type    = "absolute"
            source_frame = self.base_frame
        elif request.save_mode == 1:
            pose_type    = "relative"
            source_frame = "camera_color_optical_frame"
        elif request.save_mode == 2:
            pose_type    = "relative"
            source_frame = request.reference_frame
            if not source_frame:
                response.success = False
                response.message = "ERROR: save_mode=2 richiede reference_frame."
                self.get_logger().error(response.message)
                return response
        else:
            response.success = False
            response.message = f"ERROR: save_mode={request.save_mode} non valido."
            self.get_logger().error(response.message)
            return response

        try:
            now = rclpy.time.Time()
            ts  = self.tf_buffer.lookup_transform(
                source_frame, target_frame, now,
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            t = ts.transform.translation
            r = ts.transform.rotation

            pose_data = {
                "task_type":    "move",
                "task_name":    request.task_name,
                "type":         pose_type,
                "source_frame": source_frame,
                "target_frame": target_frame,
                "transform": {
                    "translation": {"x": t.x, "y": t.y, "z": t.z},
                    "rotation":    {"x": r.x, "y": r.y, "z": r.z, "w": r.w}
                }
            }

            self.all_poses.append(pose_data)
            self._save_poses_to_file()

            response.success = True
            response.message = (
                f"Saved '{request.task_name}' "
                f"({pose_type}: {target_frame} w.r.t. {source_frame}). Total entries: {len(self.all_poses)}"
            )
            self.get_logger().info(response.message)

        except TransformException as ex:
            response.success = False
            response.message = f"TF error: {ex}"
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # Callback: salva azione gripper (Accumula nel file principale)
    # ------------------------------------------------------------------

    def save_gripper_action_callback(
        self, request: GripperSrv.Request, response: GripperSrv.Response
    ):
        command      = request.command.strip().lower()
        gripper_type = request.gripper_type.strip().lower() \
                       if request.gripper_type else "auto"
        position     = float(request.position)

        valid_commands = ("open", "close", "move")
        if command not in valid_commands:
            response.success = False
            response.message = f"Comando '{command}' non valido. Usa: {valid_commands}."
            self.get_logger().error(response.message)
            return response

        if command == "move":
            if not (0.0 <= position <= 1.0):
                response.success = False
                response.message = f"position={position:.3f} fuori range [0.0, 1.0] per 'move'."
                self.get_logger().error(response.message)
                return response

        gripper_data = {
            "task_type":    "gripper",
            "command":      command,
            "position":     position,   
            "gripper_type": gripper_type
        }

        self.all_poses.append(gripper_data)
        self._save_poses_to_file()

        pos_info = f"position={position:.3f}" if command == "move" else command
        response.success = True
        response.message = (
            f"Saved gripper action: {pos_info} ({gripper_type}). "
            f"Total entries: {len(self.all_poses)}."
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Callback: azzera tutto MA salva un backup cronologico prima
    # ------------------------------------------------------------------

    def clear_poses_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        try:
            # Se ci sono delle pose accumulate, prima di cancellarle facciamo un backup storico
            if self.all_poses:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_filename = f"backup_poses_{timestamp}.json"
                backup_path = os.path.join(self.task_result_dir, backup_filename)
                
                # Copia il file pieno corrente nel file di backup storico
                if os.path.exists(self.json_file_path):
                    shutil.copyfile(self.json_file_path, backup_path)
                    msg_backup = f"Backup creato con successo: {backup_filename}. "
                else:
                    msg_backup = "Nessun file su disco da backuppare. "
            else:
                msg_backup = "Lista già vuota. "

            # Ora svuota l'array e azzera il file principale
            self.all_poses = []
            with open(self.json_file_path, 'w') as f:
                json.dump([], f)
                
            response.success = True
            response.message = f"{msg_backup}Tutte le voci correnti sono state azzerate in robot_poses_ws.json."
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to clear/backup: {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
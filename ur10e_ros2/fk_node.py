import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin 
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import subprocess
import re
from geometry_msgs.msg import PoseStamped # Importa PoseStamped per pubblicare la posa
from scipy.spatial.transform import Rotation as R # Importa la classe Rotation

class UR10eFKNode(Node):
    def __init__(self):
        super().__init__('ur10e_fk_node')

        self.get_logger().info('Initializing UR10e Forward Kinematics Node...')

        try:
            # Ottieni la directory share del pacchetto 'ur_description'
            ur_description_share_dir = get_package_share_directory('ur_description')

            # Percorso al file XACRO principale
            urdf_xacro_path = os.path.join(ur_description_share_dir, 'urdf', 'ur.urdf.xacro')

            # Percorso della directory delle mesh (già inclusa in ur_description_share_dir/meshes)
            mesh_dir_path = os.path.join(ur_description_share_dir, 'meshes')

            # File temporaneo per il URDF generato da xacro (con package:// URI)
            temp_urdf_file_raw = "/tmp/ur10e_pinocchio_generated_raw.urdf"
            # File temporaneo per il URDF modificato con percorsi assoluti (da passare a Pinocchio)
            urdf_path_for_pinocchio = "/tmp/ur10e_pinocchio_final.urdf"

            # Costruisci il comando xacro per generare il URDF senza ros2_control/gazebo
            xacro_command = [
                'ros2', 'run', 'xacro', 'xacro', urdf_xacro_path,
                'ur_type:=ur10e',
                'name:=ur10e_pinocchio',
                'transmission_hw_interface:=""',
                'sim_gazebo:=false',
                'sim_ignition:=false',
                'use_fake_hardware:=false',
                'headless_mode:=false'
            ]

            self.get_logger().info(f"Generating clean URDF using xacro: {' '.join(xacro_command)} > {temp_urdf_file_raw}")

            try:
                # Esegui il comando xacro e salva l'output nel file temporaneo "raw"
                with open(temp_urdf_file_raw, 'w') as f:
                    subprocess.run(xacro_command, check=True, stdout=f, stderr=subprocess.PIPE)
                self.get_logger().info(f"Raw URDF generated at: {temp_urdf_file_raw}")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to generate raw URDF with xacro. Stderr: {e.stderr.decode()}")
                raise
            except Exception as e:
                self.get_logger().error(f"Error during raw xacro generation: {e}")
                raise

            # --- MODIFICA CRUCIALE: Leggi il URDF grezzo e sostituisci i percorsi "package://" ---
            with open(temp_urdf_file_raw, 'r') as f_in:
                urdf_content = f_in.read()

            # Espressioni regolari per sostituire gli URI package:// con percorsi assoluti
            # Assumiamo che le mesh siano nel sottocartella ur10e/visual e ur10e/collision
            # all'interno della directory delle mesh del pacchetto ur_description.
            # Questo è il pattern comune per i robot UR.

            # Sostituzione per le mesh visual
            urdf_content = re.sub(
                r'filename="package://ur_description/meshes/ur10e/visual/([^"]+)"',
                lambda m: f'filename="{os.path.join(mesh_dir_path, "ur10e", "visual", m.group(1))}"',
                urdf_content
            )
            # Sostituzione per le mesh collision
            urdf_content = re.sub(
                r'filename="package://ur_description/meshes/ur10e/collision/([^"]+)"',
                lambda m: f'filename="{os.path.join(mesh_dir_path, "ur10e", "collision", m.group(1))}"',
                urdf_content
            )

            # Salva il URDF modificato in un nuovo file temporaneo, che Pinocchio userà
            with open(urdf_path_for_pinocchio, 'w') as f_out:
                f_out.write(urdf_content)
            self.get_logger().info(f"URDF with absolute mesh paths saved to: {urdf_path_for_pinocchio}")
            # --- FINE MODIFICA ---

        except Exception as e:
            self.get_logger().error(f"Failed to get URDF/mesh paths or generate/process URDF: {e}")
            raise # Rilancia l'eccezione se la fase di preparazione fallisce

        if not os.path.exists(urdf_path_for_pinocchio):
            self.get_logger().error(f"Final URDF file not found at: {urdf_path_for_pinocchio}. Something went wrong with processing.")
            raise FileNotFoundError(f"Final URDF file missing: {urdf_path_for_pinocchio}")

        # --- Carica il modello con Pinocchio ---
        try:
            # Ora Pinocchio riceve un URDF con percorsi assoluti, non ha bisogno di risolvere "package://"
            self.model = pin.buildModelFromUrdf(urdf_path_for_pinocchio)
            self.data = self.model.createData()
            self.get_logger().info('Robot model loaded successfully with Pinocchio.')
            self.get_logger().info(f"Model has {self.model.nq} degrees of freedom.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio model from {urdf_path_for_pinocchio}.")
            self.get_logger().error(f"Specific Pinocchio error: {e}")
            raise # Rilancia l'eccezione

        # Determina i nomi dei giunti controllabili da Pinocchio
        self.controllable_joint_ids = []
        self.joint_names = []
        # model.nq sono i gradi di libertà attivi
        # model.joints[0] è il giunto base (universe)
        for i in range(1, self.model.nq + 1): # Itera sui giunti Pinocchio a partire dal primo giunto mobile
            joint_name = self.model.names[self.model.joints[i].id]
            if self.model.joints[i].nq > 0: # Assicurati che sia un giunto mobile (non fisso)
                 self.controllable_joint_ids.append(self.model.joints[i].id)
                 self.joint_names.append(joint_name)

        self.get_logger().info(f"Controllable joints: {self.joint_names}")
        self.q = pin.neutral(self.model) # Inizializza la configurazione dei giunti alla posizione neutra

        # --- AGGIUNTA: Publisher per la posa dell'end-effector ---
        self.fk_pose_publisher = self.create_publisher(PoseStamped, '/ur10e_fk_pose', 10)
        self.get_logger().info('Publishing end-effector pose to /ur10e_fk_pose topic.')
        # --- FINE AGGIUNTA ---

        # ROS2 subscription per JointState
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info('Subscribed to /joint_states topic.')

    def joint_callback(self, msg):
        # Mappa i nomi dei giunti ai loro valori nella JointState
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}

        # Aggiorna la configurazione dei giunti di Pinocchio 'q'
        for joint_name in self.joint_names:
            if joint_name in name_to_pos:
                # Trova l'ID del giunto Pinocchio
                joint_id = self.model.getJointId(joint_name)
                pin_q_idx = self.model.joints[joint_id].idx_q 
                
                # Assicurati che idx_q sia un indice valido per un giunto a 1 DOF
                # Per UR10e, tutti i giunti sono a 1 DOF, quindi nq sarà 1.
                if self.model.joints[joint_id].nq == 1:
                    self.q[pin_q_idx] = name_to_pos[joint_name]
                else:
                    self.get_logger().warn(
                        f"Joint '{joint_name}' has {self.model.joints[joint_id].nq} DOFs. "
                        "Current logic only handles 1-DOF joints properly for q update."
                    )
            # else:
            #     self.get_logger().warn(f"Joint '{joint_name}' not found in /joint_states message. Using previous value.")

        # Esegui la cinematica diretta
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data) # Aggiorna i posizionamenti dei frame dopo la FK

        # Ottieni la posa dell'end-effector ('tool0' è il frame standard per UR)
        try:
            ee_frame_id = self.model.getFrameId('tool0')
            if ee_frame_id < self.model.nframes:
                ee_pose = self.data.oMf[ee_frame_id]
                pos = ee_pose.translation
                rot_matrix = ee_pose.rotation # Matrice di rotazione 3x3

                # Conversione della Matrice di Rotazione in Quaternione
                rotation = R.from_matrix(rot_matrix)
                quaternion = rotation.as_quat() # Restituisce [x, y, z, w]

                # self.get_logger().info(
                #     f'End-effector position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f} '
                #     f'Orientation (Quaternion - x,y,z,w): x={quaternion[0]:.3f}, y={quaternion[1]:.3f}, z={quaternion[2]:.3f}, w={quaternion[3]:.3f}'
                # )

                # --- AGGIUNTA: Pubblica la posa come PoseStamped ---
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'base_link' # O il frame base del tuo robot, es. 'base' o 'world'

                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]

                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                self.fk_pose_publisher.publish(pose_msg)
                # --- FINE AGGIUNTA ---

            else:
                self.get_logger().warn("Frame 'tool0' not found in Pinocchio model. Check URDF.")
        except Exception as e:
            self.get_logger().error(f"Error getting end-effector pose: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UR10eFKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
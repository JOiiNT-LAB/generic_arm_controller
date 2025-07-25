import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker # Nuovo import
from std_msgs.msg import ColorRGBA # Nuovo import
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException

class MarkerPoseTransformerNode(Node):
    def __init__(self):
        super().__init__('marker_pose_transformer_node')
        self.get_logger().info('Initializing Marker Pose Transformer Node...')

        # --- Caricamento Calibrazione Hand-Eye (T_EE_Camera) ---
        self.T_EE_Camera_matrix = None
        try:
            package_share_directory = get_package_share_directory('ur10e_ros2')
            self.calibration_file_path = os.path.join(
                package_share_directory, 'calibration_results', 'hand_eye_transform.yaml'
            )
            self._load_hand_eye_calibration_from_yaml()
        except Exception as e:
            self.get_logger().error(f"Failed to find or load Hand-Eye calibration: {e}")
            self.T_EE_Camera_matrix = None
        
        # --- Variabili e Sottoscrizioni ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.latest_fk_pose = None  # Per memorizzare la posa attuale da /ur10e_fk_pose
        self.has_published_pose = False

        # --- Offset per il task ---
        # Aumentato l'offset Z per vedere meglio l'effetto
        # Puoi tornare a 0.2 se vuoi 20cm
        self.approach_offset_z = -0.15 # 40 cm sopra il marker per visualizzare

        # --- Sottoscrizioni ---
        self.marker_pose_sub = self.create_subscription(
            PoseStamped, '/aruco_single/pose', self.marker_pose_callback, 10
        )
        self.fk_pose_sub = self.create_subscription(
            PoseStamped, '/ur10e_fk_pose', self._fk_pose_callback, 10
        )
        self.get_logger().info('Subscribed to /aruco_single/pose and /ur10e_fk_pose topics.')

        # --- Publisher per la cinematica inversa ---
        self.ik_target_pose_pub = self.create_publisher(
            PoseStamped, '/target_cartesian_pose', 10
        )
        self.get_logger().info('Publishing final IK target poses on /target_cartesian_pose.')

        # --- Publisher per il Marker di visualizzazione (NOVITÀ) ---
        self.marker_viz_pub = self.create_publisher(
            Marker, '/visualization_marker', 10
        )
        self.get_logger().info('Publishing target visualization marker on /visualization_marker.')


    def _fk_pose_callback(self, msg: PoseStamped):
        """Salva l'ultima posa ricevuta dal topic di forward kinematics."""
        self.latest_fk_pose = msg

    def _load_hand_eye_calibration_from_yaml(self):
        if not os.path.exists(self.calibration_file_path):
            self.get_logger().error(f"File di calibrazione non trovato: {self.calibration_file_path}")
            raise FileNotFoundError(f"File di calibrazione non trovato")
        try:
            with open(self.calibration_file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            trans = calib_data['translation']
            rot = calib_data['rotation']
            R_EE_Camera = Rotation.from_quat([rot['x'], rot['y'], rot['z'], rot['w']]).as_matrix()
            t_EE_Camera = np.array([trans['x'], trans['y'], trans['z']])
            self.T_EE_Camera_matrix = np.eye(4)
            self.T_EE_Camera_matrix[:3, :3] = R_EE_Camera
            self.T_EE_Camera_matrix[:3, 3] = t_EE_Camera
            self.get_logger().info(f"Calibrazione Hand-Eye caricata con successo.")
        except Exception as e:
            self.get_logger().error(f"Errore caricamento calibrazione: {e}")
            raise RuntimeError(f"Errore caricamento calibrazione: {e}")
            
    def marker_pose_callback(self, msg: PoseStamped):
        if self.has_published_pose:
            return

        if self.T_EE_Camera_matrix is None:
            self.get_logger().warn("Calibrazione Hand-Eye (T_EE_Camera) non disponibile.")
            return

        if self.latest_fk_pose is None:
            self.get_logger().warn("Posa attuale da /ur10e_fk_pose non ancora ricevuta. Attendo...")
            return
        # 1. Converti posa marker da camera a matrice (T_Camera_Marker)
        q_cam_marker = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        t_cam_marker = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        T_Camera_Marker_matrix = np.eye(4)
        T_Camera_Marker_matrix[:3, :3] = Rotation.from_quat(q_cam_marker).as_matrix()
        T_Camera_Marker_matrix[:3, 3] = t_cam_marker
        self.get_logger().info(f"DEBUG: Marker pose from camera (T_Camera_Marker): Pos(x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}) Orient(x={msg.pose.orientation.x:.3f}, y={msg.pose.orientation.y:.3f}, z={msg.pose.orientation.z:.3f}, w={msg.pose.orientation.w:.3f})")


        # 2. Calcola posa marker rispetto a end-effector (T_EE_Marker)
        T_EE_Marker_matrix = self.T_EE_Camera_matrix @ T_Camera_Marker_matrix
        t_ee_marker_position = T_EE_Marker_matrix[:3, 3]
        self.get_logger().info(f"DEBUG: Marker position relative to tool0 (T_EE_Marker): "
                                f"X={t_ee_marker_position[0]:.3f}, Y={t_ee_marker_position[1]:.3f}, Z={t_ee_marker_position[2]:.3f}")


        # 3. Applica offset per la posizione target rispetto a tool0
        t_ee_target_position = t_ee_marker_position + np.array([0.0, 0.0, self.approach_offset_z])
        self.get_logger().info(f"DEBUG: Target position relative to tool0 after offset (t_ee_target_position): "
                                f"X={t_ee_target_position[0]:.3f}, Y={t_ee_target_position[1]:.3f}, Z={t_ee_target_position[2]:.3f}")


        # 4. Ottieni la trasformazione attuale da 'base_link' a 'tool0'???????????????????????????
        try:
            transform_base_to_tool0 = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            T_base_to_tool0_matrix = self.transform_to_matrix(transform_base_to_tool0)
            self.get_logger().info(f"DEBUG: T_base_to_tool0_matrix (Z component of translation): {T_base_to_tool0_matrix[2, 3]:.3f}")

        except TransformException as ex:
            self.get_logger().error(f'Impossibile ottenere trasformazione da base_link a tool0: {ex}')
            return

        # 5. Trasforma il vettore posizione target dal frame 'tool0' al frame 'base_link'
        target_pos_homogeneous = np.append(t_ee_target_position, 1) # Aggiungi 1 per la coordinata omogenea
        final_pos_homogeneous = T_base_to_tool0_matrix @ target_pos_homogeneous
        final_target_position = final_pos_homogeneous[:3] # Estrai solo le coordinate x, y, z

        # 6. Assembla il messaggio finale PoseStamped per IK
        ik_target_pose_msg = PoseStamped()
        ik_target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        ik_target_pose_msg.header.frame_id = 'base_link' 

        ik_target_pose_msg.pose.position.x = final_target_position[0]
        ik_target_pose_msg.pose.position.y = final_target_position[1]
        ik_target_pose_msg.pose.position.z = final_target_position[2]
        ik_target_pose_msg.pose.orientation = self.latest_fk_pose.pose.orientation # Usa l'orientamento attuale

        self.ik_target_pose_pub.publish(ik_target_pose_msg)

        
        self.get_logger().info(f"DEBUG: Published IK target pose for EE in 'base_link' frame: "
                               f"x={ik_target_pose_msg.pose.position.x:.3f}, y={ik_target_pose_msg.pose.position.y:.3f}, z={ik_target_pose_msg.pose.position.z:.3f}")
       
       
       
        # --- PUBBLICAZIONE DEL MARKER DI VISUALIZZAZIONE (NOVITÀ) ---
        marker_viz_msg = Marker()
        marker_viz_msg.header.frame_id = "base_link"
        marker_viz_msg.header.stamp = self.get_clock().now().to_msg()
        marker_viz_msg.ns = "target_marker"
        marker_viz_msg.id = 0
        marker_viz_msg.type = Marker.SPHERE # Tipo di marker: sfera
        marker_viz_msg.action = Marker.ADD # Azione: aggiungi/modifica il marker

        marker_viz_msg.pose.position = ik_target_pose_msg.pose.position
        marker_viz_msg.pose.orientation.x = 0.0 # Per una sfera, l'orientamento non ha impatto visivo
        marker_viz_msg.pose.orientation.y = 0.0
        marker_viz_msg.pose.orientation.z = 0.0
        marker_viz_msg.pose.orientation.w = 1.0

        marker_viz_msg.scale.x = 0.05 # Diametro della sfera (5 cm)
        marker_viz_msg.scale.y = 0.05
        marker_viz_msg.scale.z = 0.05

        marker_viz_msg.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8) # Colore: Magenta, Semi-trasparente (RGBA)

        # Durata del marker (0 significa infinito, rimarrà finché il nodo è attivo o RViz è aperto)
        marker_viz_msg.lifetime.sec = 0 
        self.marker_viz_pub.publish(marker_viz_msg)
        self.get_logger().info("Pubblicato marker sferico di visualizzazione in RViz2.")
        # --- FINE SEZIONE MARKER ---


        self.has_published_pose = True

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """Converte un messaggio TransformStamped in una matrice di trasformazione 4x4 NumPy."""
        rot = transform.transform.rotation
        trans = transform.transform.translation
        R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
        t = np.array([trans.x, trans.y, trans.z])
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = t
        return matrix

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseTransformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interruzione da tastiera, chiusura in corso.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
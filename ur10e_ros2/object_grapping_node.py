import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform # Importa anche Transform se definisci l'offset in quel modo
import tf2_ros
import tf2_geometry_msgs # Per funzioni come do_transform_pose
from tf2_ros import TransformException
import numpy as np
from scipy.spatial.transform import Rotation
import math

class ObjectGrabbingNode(Node):
    def __init__(self):
        super().__init__('object_grabbing_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TOPIC DI PUBBLICAZIONE: La posa target per il tuo nodo IK esistente
        self.ee_target_pose_publisher = self.create_publisher(
            PoseStamped, '/target_cartesian_pose', 10) # <-- Sostituisci con il topic corretto del tuo nodo IK

        # TOPIC DI SOTTOSCRIZIONE: Posa dell'oggetto dal nodo di visione
        self.object_pose_subscriber = self.create_subscription(
            PoseStamped, '/aruco_CT/aruco_single/pose', self.object_pose_callback, 10) # <-- Sostituisci con il topic del tuo oggetto

        self.get_logger().info("Nodo di calcolo posa di presa avviato.")

        # --- DEFINIZIONE FONDAMENTALE: Posa del End-Effector rispetto all'Oggetto ---
        # Questa è la posa relativa tra il frame dell'oggetto e il frame del tuo end-effector
        # quando il tuo end-effector è nella posizione desiderata per l'interazione (es. presa).
        # Devi personalizzare questi valori in base alla tua pinza e all'oggetto.
        # Esempio: Posizionamento 10 cm sopra il centro dell'oggetto, con l'asse Z della pinza
        # rivolto verso il basso (quindi rotazione di 180 gradi attorno all'asse Y dell'oggetto).

        # Traslazione dell'EE rispetto al frame dell'oggetto
        offset_translation = np.array([0.0, 0.0, 0.10]) # 10 cm sopra l'oggetto (lungo Z dell'oggetto)

        # Rotazione dell'EE rispetto al frame dell'oggetto
        # Esempio: l'asse X dell'oggetto punta in avanti, l'asse Z in alto.
        # Se la tua pinza si chiude lungo il suo asse X e vuoi che chiuda lungo l'asse Y dell'oggetto,
        # e l'asse Z della pinza deve puntare verso l'oggetto (quindi verso il basso rispetto all'oggetto Z),
        # potresti aver bisogno di una rotazione complessa.

        # Per una pinza che si apre lungo il proprio asse X e la cui base è il frame tool0:
        # Se vogliamo che l'asse Z della pinza punti verso il basso (nell'oggetto)
        # e l'asse X della pinza si allinei, ad esempio, con l'asse X dell'oggetto.
        # Un approccio comune è ruotare di 180 gradi attorno all'asse Y per "capovolgere" l'EE
        # se il tool0 ha Z in avanti e la pinza in Z.

        # Facciamo un esempio semplice: punta l'EE dritto verso l'oggetto (Z dell'EE punta a -Z dell'oggetto)
        # e mantieni gli assi X e Y allineati. Questo spesso è una rotazione attorno all'asse X o Y di 180 gradi.
        # Se tool0 ha Z che punta in avanti, e vuoi che punti in basso:
        # Rotazione di -PI/2 (o -90 deg) attorno all'asse Y del frame tool0.
        # MA QUESTA E' RELATIVA ALL'OGGETTO

        # Supponiamo che l'asse Z del tuo tool0 (end-effector) punti in avanti
        # E che tu voglia che punti direttamente verso il basso sull'oggetto.
        # Se l'asse Z dell'oggetto è verso l'alto, allora devi ruotare l'end-effector
        # di 180 gradi attorno al suo asse Y per "capovolgerlo".

        offset_rotation_quat = Rotation.from_euler('xyz', [0, math.pi, 0]).as_quat() # 180 gradi attorno Y

        self.T_object_to_ee_target_matrix = np.eye(4)
        self.T_object_to_ee_target_matrix[:3, 3] = offset_translation
        self.T_object_to_ee_target_matrix[:3, :3] = Rotation.from_quat(offset_rotation_quat).as_matrix()

        self.get_logger().info(f"Offset End-Effector rispetto Oggetto definito:\n{self.T_object_to_ee_target_matrix}")
        # ------------------------------------------------------------------

    def object_pose_callback(self, msg: PoseStamped):
        object_pose_in_camera_frame = msg # La posa dell'oggetto nel frame della telecamera

        # Frame di riferimento desiderato per la posa target finale del robot
        target_robot_base_frame = 'base_link' # O 'odom', a seconda del tuo setup robotico

        self.get_logger().info(f"Ricevuta posa oggetto nel frame: {object_pose_in_camera_frame.header.frame_id}")

        try:
            # 1. Ottieni la trasformazione da 'base_link' a 'camera_color_optical_frame'.
            #    TF2 usa internamente la tua TF statica 'tool0 -> camera_color_optical_frame'
            #    e la TF corrente del robot 'base_link -> tool0'.
            transform_base_to_camera = self.tf_buffer.lookup_transform(
                target_robot_base_frame,
                object_pose_in_camera_frame.header.frame_id, # 'camera_color_optical_frame'
                object_pose_in_camera_frame.header.stamp,    # Usa lo timestamp della posa dell'oggetto
                timeout=rclpy.duration.Duration(seconds=0.1) # Breve timeout
            )
            self.get_logger().info(f"Trovata trasformazione da {object_pose_in_camera_frame.header.frame_id} a {target_robot_base_frame}.")

            # 2. Trasforma la posa dell'oggetto dal frame della telecamera al frame della base del robot.
            object_pose_in_base_frame: PoseStamped = tf2_geometry_msgs.do_transform_pose(
                object_pose_in_camera_frame, transform_base_to_camera
            )
            self.get_logger().info(f"Posa oggetto nel frame {target_robot_base_frame}: {object_pose_in_base_frame.pose.position}")

            # 3. Applica la trasformazione desiderata dell'EE rispetto all'oggetto.
            #    Converti la posa dell'oggetto (ora in base_link) in una matrice 4x4.
            T_base_to_object_matrix = self.pose_to_matrix(object_pose_in_base_frame.pose)

            # Moltiplica le matrici per ottenere la posa target finale dell'EE nel frame della base.
            # Questa è la concatenazione: T_base_to_object * T_object_to_ee_target
            T_base_to_ee_target_matrix = T_base_to_object_matrix @ self.T_object_to_ee_target_matrix

            # Converti la matrice risultante in un messaggio PoseStamped.
            target_ee_pose_stamped = PoseStamped()
            target_ee_pose_stamped.header.frame_id = target_robot_base_frame
            target_ee_pose_stamped.header.stamp = self.get_clock().now().to_msg() 
            target_ee_pose_stamped.pose = self.matrix_to_pose(T_base_to_ee_target_matrix)

            self.get_logger().info(f"Posa target calcolata per l'End-Effector (tool0) nel frame {target_robot_base_frame}:")
            self.get_logger().info(f"Pos: X:{target_ee_pose_stamped.pose.position.x:.3f}, Y:{target_ee_pose_stamped.pose.position.y:.3f}, Z:{target_ee_pose_stamped.pose.position.z:.3f}")
            self.get_logger().info(f"Ori: X:{target_ee_pose_stamped.pose.orientation.x:.3f}, Y:{target_ee_pose_stamped.pose.orientation.y:.3f}, Z:{target_ee_pose_stamped.pose.orientation.z:.3f}, W:{target_ee_pose_stamped.pose.orientation.w:.3f}")

            # 4. Pubblica la posa target: il tuo nodo IK la riceverà e pianificherà il movimento.
            self.ee_target_pose_publisher.publish(target_ee_pose_stamped)

        except TransformException as ex:
            self.get_logger().warn(f'Impossibile ottenere o trasformare la posa: {ex}')
        except Exception as e:
            self.get_logger().error(f'Errore inaspettato nel callback: {e}')

    # Funzioni ausiliarie per conversioni Pose <-> Matrix (già definite nel tuo codice di calibrazione)
    def pose_to_matrix(self, pose):
        q = pose.orientation
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = position
        return transform_matrix

    def matrix_to_pose(self, matrix):
        pose = PoseStamped().pose
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = ObjectGrabbingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
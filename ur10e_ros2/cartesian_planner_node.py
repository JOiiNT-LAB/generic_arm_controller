import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

def slerp_quaternion(q0, q1, t):
    """
    Esegue l'interpolazione sferica lineare (SLERP) tra due quaternioni.
    """
    # Normalizza i quaternioni per essere sicuro che siano unitari
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    dot = np.dot(q0, q1)
    
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    
    theta = np.arccos(dot)
    
    sin_theta = np.sin(theta)
    s0 = np.sin((1 - t) * theta) / sin_theta
    s1 = np.sin(t * theta) / sin_theta
    
    return (s0 * q0) + (s1 * q1)


class CartesianPlannerNode(Node):
    """
    ROS2 Node che genera una traiettoria di pose cartesiane lineari.
    """
    def __init__(self):
        super().__init__('cartesian_planner_node')

        self.get_logger().info('Initializing Cartesian Planner Node...')

        # --- Parametri ---
        self.waypoint_count = 1000  # Numero totale di waypoint da generare lungo la traiettoria
        self.waypoint_delay = 0.01  # Ritardo (in secondi) tra la pubblicazione di ogni waypoint
        
        # Sostituiamo questi parametri con delle soglie di tolleranza
        self.position_tolerance = 0.01  # Tolleranza in metri (5 mm) per la posizione
        self.orientation_tolerance = 0.1  # Tolleranza per l'orientamento (valore adimensionale per quaternioni)

        # --- Iscrizioni e pubblicazione ---
        self.current_pose_subscription = self.create_subscription(
            PoseStamped,
            '/ur10e_fk_pose',
            self.current_pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /ur10e_fk_pose topic for current robot pose.')

        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            '/target_robot_pose',
            self.target_pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /target_robot_pose topic.')

        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            '/target_cartesian_pose',
            10
        )
        self.get_logger().info('Publishing to /target_cartesian_pose topic.')

        # Variabili di stato
        self.current_pose = None
        self.target_pose = None
        self.has_current_pose = False
        self.is_moving = False
        self.waypoint_index = 0
        
        # Inizializza il timer a None. Verrà creato solo quando serve
        self.waypoint_timer = None
   
    def current_pose_callback(self, msg: PoseStamped):
        """
        Callback per la posa corrente.
        """
        if not self.has_current_pose:
            self.get_logger().info("Received initial current pose from FK node.")
            self.has_current_pose = True
        
        self.current_pose = msg.pose
        
        # Questo è il nuovo controllo per fermare il movimento
        if self.is_moving and self.check_target_reached():
            self.stop_movement()

    def target_pose_callback(self, msg: PoseStamped):
        """
        Callback per la posa target. Avvia la generazione e la pubblicazione dei waypoint.
        """
        if self.is_moving:
            self.get_logger().warn("Robot is already moving. Ignoring new target pose.")
            return
        
        if not self.has_current_pose:
            self.get_logger().warn("Waiting for initial current pose from FK node before planning a trajectory.")
            return

        self.get_logger().info("Received new target pose. Starting trajectory planning...")

        self.is_moving = True
        self.target_pose = msg.pose
        self.waypoint_index = 0
        
        # Avvia il timer per pubblicare i waypoint a intervalli regolari
        if self.waypoint_timer is None:
            self.waypoint_timer = self.create_timer(self.waypoint_delay, self.publish_waypoint)
    
    def publish_waypoint(self):
        """
        Callback del timer. Genera e pubblica il prossimo waypoint.
        """
        # Verifica se abbiamo già pubblicato tutti i waypoint
        if self.waypoint_index >= self.waypoint_count:
            self.get_logger().info("All planned waypoints have been published. Waiting for robot to reach target...")
            return

        # Posa di partenza (la posa corrente del robot)
        current_position = np.array([self.current_pose.position.x, 
                                     self.current_pose.position.y, 
                                     self.current_pose.position.z])
        current_orientation_quat = np.array([self.current_pose.orientation.x, 
                                             self.current_pose.orientation.y, 
                                             self.current_pose.orientation.z, 
                                             self.current_pose.orientation.w])
        
        # Posa di arrivo
        target_position = np.array([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
        target_orientation_quat = np.array([self.target_pose.orientation.x, 
                                            self.target_pose.orientation.y, 
                                            self.target_pose.orientation.z, 
                                            self.target_pose.orientation.w])
        
        # Calcola il parametro di interpolazione (t) in base al waypoint_index
        t = (self.waypoint_index + 1) / self.waypoint_count
        
        # Genera il waypoint corrente
        # traiettoiria lineare per la posizione e SLERP per l'orientamento
        # viene generato un waypoint alla volta mentre il robot si muove verso il target
        interp_position = current_position + t * (target_position - current_position)
        interp_orientation_quat = slerp_quaternion(current_orientation_quat, target_orientation_quat, t)

        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = 'base_link'
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        
        waypoint_msg.pose.position.x = interp_position[0]
        waypoint_msg.pose.position.y = interp_position[1]
        waypoint_msg.pose.position.z = interp_position[2]
        
        waypoint_msg.pose.orientation.x = interp_orientation_quat[0]
        waypoint_msg.pose.orientation.y = interp_orientation_quat[1]
        waypoint_msg.pose.orientation.z = interp_orientation_quat[2]
        waypoint_msg.pose.orientation.w = interp_orientation_quat[3]
        
        # Pubblica il waypoint e aggiorna l'indice
        self.waypoint_publisher.publish(waypoint_msg)
        self.get_logger().info(f"Published waypoint {self.waypoint_index + 1}/{self.waypoint_count}")
        self.waypoint_index += 1
    
    def check_target_reached(self):
        """
        Verifica se la posa corrente è vicina alla posa target.
        """
        if self.current_pose is None or self.target_pose is None:
            return False

        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
        target_pos = np.array([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
        
        # Calcola la distanza euclidea tra le posizioni
        position_diff = np.linalg.norm(current_pos - target_pos)

        # Calcola la differenza tra le orientazioni dei quaternioni
        # Un modo semplice è usare il dot product. Se è vicino a 1 (o -1), sono molto simili.
        current_quat = np.array([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
        target_quat = np.array([self.target_pose.orientation.x, self.target_pose.orientation.y, self.target_pose.orientation.z, self.target_pose.orientation.w])
        
        dot_product = np.abs(np.dot(current_quat, target_quat))
        
        if position_diff < self.position_tolerance and dot_product > (1 - self.orientation_tolerance):
            self.get_logger().info("Target pose reached!")
            return True
        return False
        
    def stop_movement(self):
        """
        Ferma il timer e resetta lo stato.
        """
        if self.waypoint_timer:
            self.waypoint_timer.destroy()
            self.waypoint_timer = None
        self.is_moving = False
        self.get_logger().info("Movement complete. Planner is now ready for a new target.")


def main(args=None):
    rclpy.init(args=args)
    cartesian_planner_node = CartesianPlannerNode()
    try:
        rclpy.spin(cartesian_planner_node)
    except KeyboardInterrupt:
        cartesian_planner_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        cartesian_planner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
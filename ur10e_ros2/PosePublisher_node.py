import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # <--- Importa PoseStamped
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time # Usato implicitamente da rclpy.time
import math
import transformations as tf_trans

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        # --- SETTINGS ---
        self.target_frame = 'base_link'       # Frame di riferimento per le pose pubblicate (es. 'base_link')
        self.end_effector_frame = 'tool0'     # Frame dell'end-effector del robot da tracciare per la posa iniziale (da TF)
        
        self.total_trajectory_duration_s =10.0 # Durata desiderata per l'intera traiettoria (es. 5 secondi)
        self.interpolation_steps = 200        # Numero di passi per l'interpolazione (più alto = più fluido, più punti)
                
        # Calcola la frequenza di pubblicazione per distribuire uniformemente i waypoint sulla durata totale.
        self.publish_frequency = self.interpolation_steps / self.total_trajectory_duration_s
        self.get_logger().info(f"Calculated publishing frequency: {self.publish_frequency:.2f} Hz")
        
        # --- Publisher per i comandi al nodo IK ---
        # Pubblica sul topic '/target_cartesian_pose' usando geometry_msgs/PoseStamped
        self.publisher_ = self.create_publisher(PoseStamped, '/target_cartesian_pose', 10)

        # --- TF2 Setup ---
        # Inizializza il TF Buffer e il Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- NUOVO: Subscriber per ricevere la posa target dall'esterno ---
        # Questo nodo si mette in ascolto sul topic '/set_target_pose'
        self.target_subscriber_ = self.create_subscription(
            PoseStamped,
            '/set_target_pose',
            self.target_pose_callback, # La callback che gestirà il nuovo target
            10
        )
        
        # Inizializza la posa target a None. Sarà impostata dal subscriber.
        self.target_pose_stamped = None 
        
        # Variabili di stato per la traiettoria
        self.interpolated_commands = []
        self.command_index = 0
        self.trajectory_in_progress = False # Flag per indicare se una traiettoria è in esecuzione
        self.current_start_pose = None # Posa da cui iniziare l'interpolazione (ottenuta da TF)

        # Il timer iniziale tenterà di avviare la traiettoria solo quando riceveremo un target
        # e avremo la posa iniziale del robot.
        self.initialization_attempt_timer = self.create_timer(1.0, self.try_initialize_trajectory_if_ready)
        
        self.get_logger().info('Pose Publisher Node started.')
        self.get_logger().info(f"Waiting to get initial pose of '{self.end_effector_frame}' from TF and a target pose on '/set_target_pose'.")

    # --- NUOVO: Callback per la posa target esterna ---
    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received new target pose from external command: "
                               f"Pos: X:{msg.pose.position.x:.3f}, Y:{msg.pose.position.y:.3f}, Z:{msg.pose.position.z:.3f}, "
                               f"Ori: W:{msg.pose.orientation.w:.3f}, X:{msg.pose.orientation.x:.3f}, Y:{msg.pose.orientation.y:.3f}, Z:{msg.pose.orientation.z:.3f}")
        
        # Verifica se il quaternione è valido
        quat_array = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        quat_norm = np.linalg.norm(quat_array)
        if quat_norm < 0.99 or quat_norm > 1.01: # Check for reasonable unit quaternion
            self.get_logger().warn(f"Received non-unit quaternion (norm: {quat_norm:.3f}). Attempting to normalize.")
            if quat_norm > 1e-6: # Avoid division by zero
                msg.pose.orientation.x = quat_array[0] / quat_norm
                msg.pose.orientation.y = quat_array[1] / quat_norm
                msg.pose.orientation.z = quat_array[2] / quat_norm
                msg.pose.orientation.w = quat_array[3] / quat_norm
            else:
                self.get_logger().error("Received near-zero quaternion. Cannot normalize. Using default orientation (identity).")
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0

        self.target_pose_stamped = msg # Memorizza la nuova posa target
        self.target_pose_stamped.header.frame_id = self.target_frame # Assicurati che il frame sia corretto, anche se dovrebbe già esserlo dall'input

        # Se non c'è una traiettoria in corso, prova ad avviarne una nuova
        if not self.trajectory_in_progress:
            self.try_initialize_trajectory_if_ready()
        else:
            self.get_logger().warn("A trajectory is already in progress. New target will be processed after current one finishes.")
            # Puoi implementare una logica per interrompere la traiettoria corrente qui,
            # ma per ora aspettiamo che finisca.

    # --- Funzione per tentare l'inizializzazione solo quando tutto è pronto ---
    def try_initialize_trajectory_if_ready(self):
        self.get_logger().debug("Attempting to initialize trajectory.")
        if self.target_pose_stamped is None:
            self.get_logger().info("Still waiting for an external target pose.")
            return

        # Tentativo di ottenere la posa iniziale dal TF
        try:
            # Tempo corrente per la lookup, con un timeout per evitare blocchi indefiniti
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.end_effector_frame,
                rclpy.time.Time(), # Utilizza il tempo più recente disponibile
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Controllo robusto: Assicura che il quaternione non sia (0,0,0,0) o spazzatura
            received_quat_array = np.array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w])
            
            quat_norm = np.linalg.norm(received_quat_array)
            if quat_norm < 1e-6: # Quaternion is too small, likely invalid
                raise ValueError("Received near-zero quaternion from TF lookup. Transform might be invalid or uninitialized.")
            
            # Se la norma è troppo grande (giusto per prevenzione, TF dovrebbe dare unitari)
            if quat_norm > 1.01 or quat_norm < 0.99:
                 self.get_logger().warn(f"TF quaternion for '{self.end_effector_frame}' is not unit (norm: {quat_norm:.3f}). Normalizing.")
                 received_quat_array = received_quat_array / quat_norm

            # Crea una PoseStamped dalla trasformazione TF per la posa iniziale
            self.current_start_pose = PoseStamped()
            self.current_start_pose.header.frame_id = self.target_frame
            self.current_start_pose.pose.position.x = transform.transform.translation.x
            self.current_start_pose.pose.position.y = transform.transform.translation.y
            self.current_start_pose.pose.position.z = transform.transform.translation.z
            self.current_start_pose.pose.orientation.x = received_quat_array[0]
            self.current_start_pose.pose.orientation.y = received_quat_array[1]
            self.current_start_pose.pose.orientation.z = received_quat_array[2]
            self.current_start_pose.pose.orientation.w = received_quat_array[3]
            
            self.get_logger().info(
                f"Obtained initial pose of '{self.end_effector_frame}' (relative to '{self.target_frame}'): "
                f"Pos: x={self.current_start_pose.pose.position.x:.3f}, y={self.current_start_pose.pose.position.y:.3f}, z={self.current_start_pose.pose.position.z:.3f}, "
                f"Ori: w={self.current_start_pose.pose.orientation.w:.3f}, x={self.current_start_pose.pose.orientation.x:.3f}, y={self.current_start_pose.pose.orientation.y:.3f}, z={self.current_start_pose.pose.orientation.z:.3f}"
            )
            
            # Se siamo qui, abbiamo sia la posa iniziale che la posa target esterna.
            # Possiamo avviare la generazione e pubblicazione della traiettoria.
            self.initialization_attempt_timer.cancel() # Ferma il timer di tentativo di inizializzazione
            self.generate_and_start_trajectory() # Avvia la generazione e pubblicazione
            
        except (LookupException, ConnectivityException, ExtrapolationException, ValueError) as ex:
            self.get_logger().warn(f"Could not get transform from '{self.end_effector_frame}' to '{self.target_frame}': {ex}. Retrying...", throttle_duration_sec=1.0)
            self.get_logger().debug("Exiting try_initialize_trajectory_if_ready due to TF lookup error or invalid transform.")
# Dentro la classe PosePublisher

        # ... (resto del codice) ...
    def generate_and_start_trajectory(self):
        """
        Genera l'intera traiettoria tra la posa iniziale corrente e la posa target ricevuta,
        quindi avvia il timer di pubblicazione.
        """
        if self.current_start_pose is None or self.target_pose_stamped is None:
            self.get_logger().error("Cannot generate trajectory: start or target pose is missing. This should not happen here.")
            return

        self.get_logger().info(f"Interpolating from initial pose to the new target pose.")

        # Qui, il tuo 'waypoints_stamped' sarà semplicemente la posa target ricevuta.
        # Se in futuro vuoi più waypoint, dovrai gestire una lista qui.
        self.interpolated_commands = self.interpolate_segment_poses(
            self.current_start_pose,
            self.target_pose_stamped,
            self.interpolation_steps
        )
        if self.interpolated_commands:
            self.interpolated_commands.pop(0)
        self.command_index = 0 # Reset dell'indice per la nuova traiettoria
        self.trajectory_in_progress = True # Imposta il flag di traiettoria in corso

        # Avvia il timer di pubblicazione della traiettoria
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_trajectory)
        self.get_logger().info(f"Trajectory with {len(self.interpolated_commands)} interpolated commands calculated.")
        self.get_logger().info(f"Starting publication of trajectory points at {self.publish_frequency:.2f} Hz for a total duration of {self.total_trajectory_duration_s} seconds.")

    def interpolate_segment_poses(self, start_pose: PoseStamped, end_pose: PoseStamped, steps: int): 
        """
        Genera messaggi PoseStamped interpolati linearmente per posizione
        e orientamenti SLERP tra start_pose ed end_pose.
        """
        self.get_logger().debug("Entering interpolate_segment_poses.")
        segment_poses = []
        
        start_pos = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
        end_pos = np.array([end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z])
        
        start_quat = np.array([start_pose.pose.orientation.x, start_pose.pose.orientation.y, 
                               start_pose.pose.orientation.z, start_pose.pose.orientation.w])
        end_quat = np.array([end_pose.pose.orientation.x, end_pose.pose.orientation.y, 
                             end_pose.pose.orientation.z, end_pose.pose.orientation.w])

        # Aggiungi il punto di partenza come primo punto interpolato
        segment_poses.append(start_pose) 

        for i in range(1, steps + 1): # Modificato per andare da 1 a steps inclusi, per avere steps+1 punti
            t = i / float(steps)
            
            # Interpolazione lineare per la posizione
            interpolated_pos = start_pos + t * (end_pos - start_pos)
            
            # Interpolazione SLERP per l'orientamento
            interpolated_quat = tf_trans.quaternion_slerp(start_quat, end_quat, t)

            # Crea il nuovo messaggio PoseStamped per il punto interpolato
            new_pose_stamped = PoseStamped()
            new_pose_stamped.header.frame_id = start_pose.header.frame_id
            new_pose_stamped.pose.position.x = interpolated_pos[0]
            new_pose_stamped.pose.position.y = interpolated_pos[1]
            new_pose_stamped.pose.position.z = interpolated_pos[2]
            
            new_pose_stamped.pose.orientation.x = interpolated_quat[0]
            new_pose_stamped.pose.orientation.y = interpolated_quat[1]
            new_pose_stamped.pose.orientation.z = interpolated_quat[2]
            new_pose_stamped.pose.orientation.w = interpolated_quat[3]
            
            segment_poses.append(new_pose_stamped)
            
        self.get_logger().debug(f"Exiting interpolate_segment_poses. Generated {len(segment_poses)} poses.")
        return segment_poses

    def publish_trajectory(self):
        """Pubblica il punto successivo della traiettoria interpolata."""
        self.get_logger().debug(f"Entering publish_trajectory. Current command_index: {self.command_index}.")
        if self.command_index < len(self.interpolated_commands):
            msg = self.interpolated_commands[self.command_index] # Ora è PoseStamped
            msg.header.stamp = self.get_clock().now().to_msg() # Imposta il timestamp corrente
            self.publisher_.publish(msg)

            # Logga periodicamente o alla fine per monitorare il progresso
            if self.command_index % (self.interpolation_steps // 10) == 0 or self.command_index == len(self.interpolated_commands) - 1:
                self.get_logger().info(f"Publishing point {self.command_index+1}/{len(self.interpolated_commands)}: "
                                        f"Pos: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), "
                                        f"Ori: ({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})")
            self.command_index += 1
        else:
            self.get_logger().info("Trajectory completed.")
            self.publish_timer.cancel() # Ferma il timer di pubblicazione
            self.trajectory_in_progress = False # Resetta il flag
            self.command_index = 0 # Resetta l'indice per un potenziale riavvio della traiettoria
            self.current_start_pose = None # Resetta la posa di partenza

            # Riattiva il timer di inizializzazione per attendere un nuovo target o una nuova posa iniziale
            self.get_logger().info("Ready for a new target pose.")
            self.initialization_attempt_timer = self.create_timer(1.0, self.try_initialize_trajectory_if_ready)
            self.get_logger().debug("publish_timer cancelled, initialization_attempt_timer restarted.")


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pose Publisher interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
from time import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient  # <--- NUOVO: Import per gestire l'azione Robotiq

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import GripperCommand as GripperSrv
from control_msgs.action import GripperCommand as GripperAction # <--- NUOVO: Tipo di azione ufficiale
# Aggiungi in alto
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from gazebo_msgs.srv import SetEntityState

# Aggiungi in alto
from gazebo_msgs.srv import GetEntityState, SetEntityState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time  # <-- Importato per la gestione del tempo zero

# FIX 3: import QB protetto
try:
    from qb_softhand_industry_srvs.srv import SetCommand
    QB_AVAILABLE = True
except ImportError:
    QB_AVAILABLE = False


RG2_MAX_WIDTH_M  = 0.085
SOFTHAND_MIN_POS = 0
SOFTHAND_MAX_POS = 3500

# NUOVO: Limiti cinematici Robotiq 2F-85 in Gazebo/ROS 2 Control
ROBOTIQ_MIN_POS  = 0.0  # Aperta
ROBOTIQ_MAX_POS  = 0.8  # Chiusa completamente


class GripperManager(Node):

    def __init__(self):
        super().__init__('gripper_manager')
        # Nel __init__ della classe GripperManager

        # Nel __init__ del GripperManager:
        # Nel tuo __init__
        self.gazebo_get_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.gazebo_set_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_object = "cylinder_object"
        self.is_sim = True 
        # FIX DEADLOCK: ReentrantCallbackGroup + MultiThreadedExecutor
        self.cb_group = ReentrantCallbackGroup()

        # --- RG2 publisher ---
        self.rg2_pub = self.create_publisher(
            JointTrajectory,
            '/finger_width_trajectory_controller/joint_trajectory',
            10,
        )

        # --- SoftHand client (solo se pacchetto disponibile) ---
        self.softhand_client = None
        if QB_AVAILABLE:
            self.softhand_client = self.create_client(
                SetCommand,
                '/qb_softhand_industry_communication_handler/set_command',
                callback_group=self.cb_group,
            )
            self.get_logger().info('SoftHand client created.')
        else:
            self.get_logger().warn(
                'qb_softhand_industry_srvs non trovato. SoftHand disabilitato.'
            )

        # --- NUOVO: Robotiq Action Client ---
        self.robotiq_client = ActionClient(
            self,
            GripperAction,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb_group
        )
        self.get_logger().info('Robotiq Action Client inizializzato.')

        # --- Servizio ROS ---
        self.service = self.create_service(
            GripperSrv,
            '/gripper/command',
            self.gripper_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('GripperManager READY')

    # =========================================================
    # MAIN CALLBACK
    # =========================================================

    def gripper_callback(self, request, response):
        cmd          = request.command.strip().lower()
        gripper_type = request.gripper_type.strip().lower() \
                       if request.gripper_type else 'auto'

        position = self._resolve_position(cmd, request.position)

        if position is None:
            response.success = False
            response.message = (
                f"Comando '{cmd}' non valido. "
                "Usa 'open', 'close' oppure 'move' con position in [0.0, 1.0]."
            )
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f"Gripper → cmd={cmd}, position={position:.3f}, type={gripper_type}"
        )

        if gripper_type == 'auto':
            gripper_type = self._auto_select()

        if gripper_type == 'rg2':
            return self._handle_rg2(position, response)
        elif gripper_type == 'softhand':
            return self._handle_softhand(position, response)
        elif gripper_type == 'robotiq':
            return self._handle_robotiq(position, response) # <--- NUOVO indirizzamento
        else:
            response.success = False
            response.message = f"Tipo gripper sconosciuto: '{gripper_type}'"
            self.get_logger().error(response.message)
            return response

    # =========================================================
    # RISOLUZIONE POSIZIONE
    # =========================================================

    def _resolve_position(self, cmd: str, raw_position: float):
        if cmd == 'open':
            return 1.0  # Convenzione logica: 1.0 = Completamente Aperto
        elif cmd == 'close':
            return 0.0  # Convenzione logica: 0.0 = Completamente Chiuso
        elif cmd == 'move':
            return max(0.0, min(1.0, float(raw_position)))
        return None

    # =========================================================
    # RG2
    # =========================================================

    def _handle_rg2(self, position: float, response):
        target_width = RG2_MAX_WIDTH_M * position

        msg = JointTrajectory()
        msg.joint_names = ['finger_width']

        point = JointTrajectoryPoint()
        point.positions = [target_width]
        point.time_from_start.sec = 1
        msg.points = [point]

        self.rg2_pub.publish(msg)

        self.get_logger().info(
            f"[RG2] position={position:.3f} → {target_width*1000:.1f} mm"
        )
        response.success = True
        response.message = (
            f"RG2: position={position:.3f} ({target_width*1000:.1f} mm)"
        )
        return response

    # =========================================================
    # SOFTHAND
    # =========================================================

    def _handle_softhand(self, position: float, response):
        if not QB_AVAILABLE or self.softhand_client is None:
            response.success = False
            response.message = 'SoftHand non disponibile (pacchetto QB non installato).'
            self.get_logger().error(response.message)
            return response

        if not self.softhand_client.service_is_ready():
            response.success = False
            response.message = 'SoftHand service non disponibile.'
            self.get_logger().error(response.message)
            return response

        softhand_cmd = int(
            SOFTHAND_MAX_POS - position * (SOFTHAND_MAX_POS - SOFTHAND_MIN_POS)
        )

        req = self.softhand_client.srv_type.Request()
        req.max_repeats      = 1
        req.set_commands     = True
        req.position_command = softhand_cmd

        future = self.softhand_client.call_async(req)

        import time
        timeout = 5.0
        start   = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = f'Timeout SoftHand ({timeout:.1f} s).'
                self.get_logger().warn(response.message)
                return response

        result = future.result()

        self.get_logger().info(
            f'[SoftHand] position={position:.3f} → cmd={softhand_cmd}'
        )
        response.success = True
        response.message = (
            result.message
            if result
            else f'SoftHand: position={position:.3f} (cmd={softhand_cmd})'
        )
        return response
    def _handle_robotiq(self, position, response):
            # 1. Configurazione del goal
            robotiq_pos = ROBOTIQ_MAX_POS - position * (ROBOTIQ_MAX_POS - ROBOTIQ_MIN_POS)
            goal_msg = GripperAction.Goal()
            goal_msg.command.position = robotiq_pos
            
            # 2. Invio asincrono del goal
            send_goal_future = self.robotiq_client.send_goal_async(goal_msg)
            
            # 3. Aggiungiamo una funzione che verrà chiamata quando l'azione finisce
            send_goal_future.add_done_callback(
                lambda future: self._robotiq_goal_accepted_callback(future, position)
            )

            # 4. RISPONDI SUBITO (NON ASPETTARE)
            response.success = True
            response.message = "Comando Robotiq inviato, esecuzione in background."
            return response

    def _robotiq_goal_accepted_callback(self, future, position):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Robotiq: Goal rifiutato dal controller!")
                return
            self.get_logger().info("Robotiq: Goal accettato, attendo risultato...")
            
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda res: self._robotiq_final_callback(res, position)
            )
        except Exception as e:
            self.get_logger().error(f"Errore nella callback di accettazione: {e}")

    def _robotiq_final_callback(self, future, position):
        self.get_logger().info("Robotiq: Azione terminata, controllo grasping...")
        # Aggiungi questo log per debuggare
        self.get_logger().info(f"DEBUG: is_sim={self.is_sim}, position={position}")
        
        if self.is_sim and position < 0.2:
            if self._check_object_proximity():
                self._perform_finto_grasping()
            else:
                self.get_logger().warn("Robotiq: Chiuso ma oggetto non trovato (distanza eccessiva).")
    def _perform_finto_grasping(self):
        try:
            # 1. Recupera la posa del tool0
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            
            # 2. Chiama il servizio di Gazebo per teletrasportare l'oggetto
            req = SetEntityState.Request()
            req.state.name = self.target_object
            req.state.pose.position.x = trans.transform.translation.x
            req.state.pose.position.y = trans.transform.translation.y
            req.state.pose.position.z = trans.transform.translation.z + 0.05 # Offset Z
            req.state.reference_frame = "base_link"
            
            self.gazebo_client.call_async(req)
            self.get_logger().info("[GRASP SIM] Teletrasporto oggetto riuscito!")
        except Exception as e:
            self.get_logger().error(f"Errore durante finto grasping: {e}")
    def _check_object_proximity(self) -> bool:
        # Verifica veloce senza bloccare il thread
        if not self.gazebo_get_client.service_is_ready():
            self.get_logger().warn("Gazebo servizio non pronto, salto check grasping.")
            return False
            
        req = GetEntityState.Request()
        req.name = self.target_object
        req.reference_frame = "world"
        
        # Chiamata asincrona pura
        future = self.gazebo_get_client.call_async(req)
        
        # Non usare spin_until_future_complete! 
        # Aggiungi un callback che verrà chiamato quando Gazebo risponde
        future.add_done_callback(self._check_grasping_callback)
        return False # Torniamo False subito, la logica avverrà nel callback
        
    def _check_grasping_callback(self, future):
        try:
            res = future.result()
            if res and res.success:
                transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                distanza = ((transform.transform.translation.x - res.state.pose.position.x)**2 + 
                            (transform.transform.translation.y - res.state.pose.position.y)**2 + 
                            (transform.transform.translation.z - res.state.pose.position.z)**2)**0.5
                
                self.get_logger().info(f"Dist. finale: {distanza:.4f}")
                if distanza < 0.8:
                    self._perform_finto_grasping()
        except Exception as e:
            self.get_logger().error(f"Errore nel callback: {e}")
         # =========================================================
    # AUTO SELECT (Ottimizzato per rilevare la Robotiq)
    # =========================================================

    def _auto_select(self) -> str:
        # Se l'action server della Robotiq è visibile sulla rete ROS 2, la usa come scelta primaria
        if self.robotiq_client.server_is_ready():
            return 'robotiq'
        return 'rg2'


def main():
    rclpy.init()
    node = GripperManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
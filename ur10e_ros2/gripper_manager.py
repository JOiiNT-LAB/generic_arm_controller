import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import GripperCommand as GripperSrv

# FIX 3: import QB protetto — il nodo parte anche senza il pacchetto QB
# installato (utile quando enable_qb:=false).
try:
    from qb_softhand_industry_srvs.srv import SetCommand
    QB_AVAILABLE = True
except ImportError:
    QB_AVAILABLE = False


RG2_MAX_WIDTH_M  = 0.085
SOFTHAND_MIN_POS = 0
SOFTHAND_MAX_POS = 3500


class GripperManager(Node):

    def __init__(self):
        super().__init__('gripper_manager')

        # FIX DEADLOCK: ReentrantCallbackGroup + MultiThreadedExecutor
        # permettono al nodo di processare altri callback mentre
        # _handle_softhand attende il future, evitando il deadlock.
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
                'qb_softhand_industry_srvs non trovato. '
                'SoftHand disabilitato.'
            )

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
            return 1.0
        elif cmd == 'close':
            return 0.0
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
    # FIX DEADLOCK: non usiamo più rclpy.spin_until_future_complete(self, ...)
    # che blocca lo spin dall'interno di un callback causando deadlock.
    # Con ReentrantCallbackGroup + MultiThreadedExecutor possiamo usare
    # il future direttamente con un loop non bloccante.
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

        # Attesa non bloccante: cediamo il controllo ogni 10 ms.
        # Questo funziona perché siamo in un ReentrantCallbackGroup
        # con MultiThreadedExecutor — altri callback continuano a girare.
        import time
        timeout = 5.0
        start   = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                response.success = False
                response.message = (
                    f'Timeout SoftHand ({timeout:.1f} s).'
                )
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

    # =========================================================
    # AUTO SELECT
    # =========================================================

    def _auto_select(self) -> str:
        return 'rg2'


def main():
    rclpy.init()
    node = GripperManager()

    # FIX DEADLOCK: MultiThreadedExecutor è obbligatorio quando si usa
    # ReentrantCallbackGroup con wait non bloccante inside callback.
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




























































# import rclpy
# from rclpy.node import Node

# from ur_msgs.srv import GripperCommand as GripperSrv
# from qb_softhand_industry_srvs.srv import SetCommand

# from control_msgs.msg import GripperCommand as RG2Msg
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class GripperManager(Node):

#     def __init__(self):
#         super().__init__('gripper_manager')

#         # =========================
#         # RG2 publisher
#         # =========================
#         self.rg2_pub = self.create_publisher(
#             JointTrajectory,
#             '/finger_width_trajectory_controller/joint_trajectory',
#             10
#         )
#         # =========================
#         # SoftHand client
#         # =========================

#         self.softhand_client = self.create_client(
#             SetCommand,
#             '/qb_softhand_industry_communication_handler/set_command'
#         )

#         # =========================
#         # UNICO SERVICE ROS
#         # =========================
#         self.service = self.create_service(
#             GripperSrv,
#             '/gripper/command',
#             self.gripper_callback
#         )

#         self.get_logger().info("GripperManager READY")

#     # =========================================================
#     # MAIN CALLBACK
#     # =========================================================
#     def gripper_callback(self, request, response):

#         cmd = request.command.lower()

#         # opzionale: puoi estendere il srv con gripper_type
#         gripper_type = getattr(request, "gripper_type", "auto").lower()

#         self.get_logger().info(
#             f"Gripper request → cmd={cmd}, type={gripper_type}"
#         )

#         # =========================
#         # AUTO ROUTING
#         # =========================
#         if gripper_type == "auto":
#             gripper_type = self._auto_select()

#         # =========================
#         # RG2
#         # =========================
#         if gripper_type == "rg2":
#             return self._handle_rg2(cmd, response)

#         # =========================
#         # SOFTHAND
#         # =========================
#         elif gripper_type == "softhand":
#             return self._handle_softhand(cmd, response)

#         else:
#             response.success = False
#             response.message = f"Unknown gripper type: {gripper_type}"
#             return response
        

#     def _handle_rg2(self, cmd, response):

#         msg = JointTrajectory()
#         msg.joint_names = ['finger_width']

#         point = JointTrajectoryPoint()

#         # -------------------------
#         # NORMALIZED POSITION
#         # -------------------------
#         position = None

#         cmd = cmd.lower()

#         if cmd == "open":
#             position = 1.0
#         elif cmd == "close":
#             position = 0.0
#         else:
#             response.success = False
#             response.message = f"Invalid command: {cmd}"
#             return response

#         # clamp
#         position = max(0.0, min(1.0, position))

#         # mapping reale RG2
#         point.positions = [0.085 * position]

#         point.time_from_start.sec = 1
#         msg.points = [point]

#         self.rg2_pub.publish(msg)

#         self.get_logger().info(f"[RG2] position={position:.2f}")

#         response.success = True
#         response.message = f"RG2 executed at {position:.2f}"
#         return response
    
#     def _handle_softhand(self, cmd, response):

#         if not self.softhand_client.service_is_ready():
#             response.success = False
#             response.message = "SoftHand service not available"
#             return response

#         POSITION_MAP = {
#             "open": 0,
#             "close": 3500
#         }

#         cmd = cmd.lower()

#         if cmd not in POSITION_MAP:
#             response.success = False
#             response.message = f"Invalid command: {cmd}"
#             return response

#         position = POSITION_MAP[cmd]

#         req = self.softhand_client.srv_type.Request()
#         req.max_repeats = 1
#         req.set_commands = True
#         req.position_command = position

#         future = self.softhand_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)

#         result = future.result()

#         response.success = True
#         response.message = result.message if result else f"SoftHand {cmd} executed"
#         return response
#     # =========================================================
#     # SIMPLE POLICY
#     # =========================================================
#     def _auto_select(self):
#         """
#         Default policy:
#         - prefer RG2 if system is active
#         - fallback SoftHand
#         """
#         return "rg2"


# def main():
#     rclpy.init()
#     node = GripperManager()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
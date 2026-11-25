"""
ROS side of teach mode (hand-guiding) for task_executor_node_complete.

/set_teach_mode deactivates the profile's teach_mode_controller so the arm can be guided by
hand, and reactivates it; the state is latched on /teach_mode, and the executor refuses to
replay while it is on. An empty teach_mode_controller means the robot has no teach mode.
The decisions (what to refuse, which controllers to switch) are the pure functions in
teach_mode.py; this class only talks to the controller manager.
"""

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from generic_arm_controller import teach_mode
from generic_arm_controller.service_call import call_with_timeout
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

CONTROLLER_MANAGER_TIMEOUT_S = 5.0


class TeachModeService:

    def __init__(self, node, controller, controller_manager, callback_group, exec_lock):
        """
        Create /set_teach_mode and /teach_mode on node.

        exec_lock is the executor's sequence lock: teach mode is refused while a sequence
        runs, and holding the lock during a switch keeps a sequence from starting meanwhile.
        """
        self._node = node
        self._controller = controller
        self._exec_lock = exec_lock
        self._active = False
        self._list_client = node.create_client(
            ListControllers, f'{controller_manager}/list_controllers',
            callback_group=callback_group)
        self._switch_client = node.create_client(
            SwitchController, f'{controller_manager}/switch_controller',
            callback_group=callback_group)
        self._pub = node.create_publisher(
            Bool, 'teach_mode',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._service = node.create_service(
            SetBool, 'set_teach_mode', self._on_request, callback_group=callback_group)
        # The initial state is published once the controller manager answers.
        self._init_timer = node.create_timer(
            2.0, self._init_state, callback_group=callback_group)

    @property
    def active(self):
        """Return True while teach mode is on (the controller is inactive)."""
        return self._active

    def _on_request(self, request, response):
        enable = bool(request.data)
        locked = self._exec_lock.acquire(blocking=False)
        try:
            error = teach_mode.check_request(self._controller, executing=not locked)
            if error is None:
                error = self._switch(enable)
        finally:
            if locked:
                self._exec_lock.release()
        response.success = error is None
        response.message = error or teach_mode.describe(enable)
        if error:
            self._node.get_logger().warn(f"Teach mode request refused: {error}")
        else:
            self._node.get_logger().info(response.message)
        return response

    def _switch(self, enable):
        """Switch the controller for teach mode `enable`; return an error or None."""
        state, error = self._read_state()
        if error:
            return error
        activate, deactivate, error = teach_mode.plan_switch(enable, self._controller, state)
        if error:
            return error
        if activate or deactivate:
            req = SwitchController.Request()
            req.activate_controllers = activate
            req.deactivate_controllers = deactivate
            req.strictness = SwitchController.Request.STRICT
            req.activate_asap = True
            req.timeout = Duration(sec=2)
            res = self._call(self._switch_client, req)
            if res is None:
                return "The controller manager did not answer the switch request."
            if not res.ok:
                return f"The controller manager refused to switch '{self._controller}'."
        self._init_timer.cancel()
        self._set_state(enable)
        return None

    def _read_state(self):
        """Return (state, error) of the controller; state None if not loaded."""
        res = self._call(self._list_client, ListControllers.Request())
        if res is None:
            return None, "The controller manager is not reachable (list_controllers)."
        return teach_mode.controller_state(res.controller, self._controller), None

    def _init_state(self):
        """Publish the initial teach mode state; retried until it is known."""
        if self._controller:
            state, error = self._read_state()
            if error or state is None:
                return  # controller manager or controller not up yet
            self._set_state(state == teach_mode.INACTIVE)
        else:
            self._set_state(False)
        self._init_timer.cancel()

    def _set_state(self, active):
        self._active = active
        self._pub.publish(Bool(data=active))

    @staticmethod
    def _call(client, request):
        """Response, or None if the service is unavailable or doesn't answer in time."""
        if not client.service_is_ready():
            return None
        _, result = call_with_timeout(client, request, CONTROLLER_MANAGER_TIMEOUT_S)
        return result

"""
Pure logic of the teach mode (hand-guiding) switch (no ROS).

Teach mode means the trajectory controller named by the profile field
teach_mode_controller is inactive, so the arm can be guided by hand (Franka:
the robot gravity-compensates while no controller commands it). Kept apart
from task_executor_node_complete so it can be unit tested without a robot.
"""

ACTIVE = 'active'
INACTIVE = 'inactive'


def check_request(controller, executing):
    """
    Return an error message if a teach mode change must be refused, else None.

    controller is the profile's teach_mode_controller: empty means the robot
    has no teach mode (e.g. the Gazebo FR3, whose effort controller would let
    the arm fall when deactivated). executing is True while a sequence runs.
    """
    if not controller:
        return ('Teach mode is not supported for this robot '
                '(no teach_mode_controller in its profile).')
    if executing:
        return 'A sequence is running: wait for it to finish before changing teach mode.'
    return None


def controller_state(controllers, name):
    """Return the state of controller `name` in a ListControllers reply, or None."""
    for controller in controllers:
        if controller.name == name:
            return controller.state
    return None


def plan_switch(enable, controller, state):
    """
    Return (activate, deactivate, error) for a SwitchController request.

    state is the controller's current state (None if it is not loaded). Both
    lists empty and no error means the arm is already in the requested mode.
    """
    if state is None:
        return [], [], f"Controller '{controller}' is not loaded in the controller manager."
    wanted = INACTIVE if enable else ACTIVE
    if state == wanted:
        return [], [], None
    if state not in (ACTIVE, INACTIVE):
        return [], [], (f"Controller '{controller}' is '{state}': "
                        f"expected '{ACTIVE}' or '{INACTIVE}'.")
    if enable:
        return [], [controller], None
    return [controller], [], None


def describe(enable):
    """Human-readable outcome of a successful switch."""
    if enable:
        return 'Teach mode ON: the arm can be guided by hand.'
    return 'Teach mode OFF: the trajectory controller is active again.'

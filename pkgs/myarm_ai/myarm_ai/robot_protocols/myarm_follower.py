import logging
import math

from acton_ai import HelpfulMyArmM, find_myarm_motor
from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol
from .myarm_leader import GRIPPER_MULTIPLER, GRIPPER_OFFSET, reconnect_retry


class MyArmFollowerRobot(BaseRobotProtocol):
    """A robot whose joints constantly move back and forth"""

    class Parameters(BaseModel):
        pass

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()

        self.params = parameters or self.Parameters()
        self._handle = reconnect_retry(self._try_bringup)

    def _try_bringup(self) -> HelpfulMyArmM:
        """This process is fraught with peril, so we need to retry it"""
        handle = find_myarm_motor()
        handle.bring_up_motors()
        handle.prompt_user_to_bring_motors_into_bounds()
        return handle

    def connect(self) -> None:
        """Connects by default on __init__"""

    def deactivate_servos(self) -> None:
        """Nothing to do, no motors on the leader"""
        self._handle.set_servos_enabled(False)

    def read_joints(self) -> list[float]:
        """Move the joints back and forth, and return the new positions"""
        joints_degrees = self._handle.get_joints_angle()
        radians = [math.radians(joint) for joint in joints_degrees[:6]]
        radians.append(joints_degrees[6] * GRIPPER_MULTIPLER)
        return radians

    def write_joints(self, joints: list[float], speed: float) -> None:
        """Nothing to do, no motors on the leader"""
        joints_degrees = [math.degrees(joint) for joint in joints[:6]]
        joints_degrees.append(joints[6] / GRIPPER_MULTIPLER - GRIPPER_OFFSET)
        self._handle.set_joints_from_controller_angles(joints_degrees, int(speed))

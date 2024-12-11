import logging
import math

from acton_ai import find_myarm_motor
from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol
from .myarm_leader import reconnect_retry


class MyArmFollowerRobot(BaseRobotProtocol):
    """A robot whose joints constantly move back and forth"""

    class Parameters(BaseModel):
        pass

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()

        self.params = parameters or self.Parameters()
        self._handle = reconnect_retry(find_myarm_motor)

    def connect(self) -> None:
        """Connects by default on __init__"""
        self._handle.bring_up_motors()
        self._handle.prompt_user_to_bring_motors_into_bounds()

    def deactivate_servos(self) -> None:
        """Nothing to do, no motors on the leader"""
        self._handle.set_servos_enabled(False)

    def read_joints(self) -> list[float]:
        """Move the joints back and forth, and return the new positions"""
        joints_degrees = self._handle.get_joints_angle()
        radians = [math.radians(joint) for joint in joints_degrees]
        return radians

    def write_joints(self, joints: list[float], speed: float) -> None:
        """Nothing to do, no motors on the leader"""
        joints_degrees = [math.degrees(joint) for joint in joints]
        logging.debug(f"Writing joints {joints_degrees} at speed {int(speed)}")
        self._handle.set_joints_from_controller_angles(joints_degrees, int(speed))

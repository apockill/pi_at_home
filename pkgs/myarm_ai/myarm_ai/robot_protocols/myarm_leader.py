import logging
import math
import time
from collections.abc import Callable
from typing import TypeVar

from acton_ai import find_myarm_controller
from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol

T = TypeVar("T")

# Gripper comes out of the API in some insane value. We want a consistent API here,
# so this multiplier converts the gripper value to a linear distance value, compatible
# with the URDF, Isaac Sim, (and real life)
GRIPPER_MULTIPLER = -0.0003389
GRIPPER_OFFSET = -11.76


def reconnect_retry(
    connect_fnc: Callable[[], T], backoff: float = 1.0, max_backoff: float = 15.0
) -> T:
    retries = 0
    while True:
        try:
            return connect_fnc()
        except Exception as e:  # noqa: BLE001
            retries += 1
            logging.error(f"Failed to connect ({e}), retrying in {backoff} seconds")
            time.sleep(backoff)
            backoff += 1
            backoff = min(backoff, max_backoff)


class MyArmLeaderRobot(BaseRobotProtocol):
    """A robot whose joints constantly move back and forth"""

    class Parameters(BaseModel):
        pass

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()

        self.params = parameters or self.Parameters()
        self._handle = reconnect_retry(find_myarm_controller)

    def connect(self) -> None:
        """Connects by default on __init__"""

    def deactivate_servos(self) -> None:
        """Nothing to do, no motors on the leader"""

    def read_joints(self) -> list[float]:
        """Move the joints back and forth, and return the new positions"""
        try:
            joints_degrees = self._handle.get_joint_angles_in_mover_space()
        except TypeError:
            logging.error("Failed to read joints, reconnecting.")
            self._handle = reconnect_retry(find_myarm_controller)
            return self.read_joints()

        radians = [math.radians(joint) for joint in joints_degrees[:6]]
        radians.append((joints_degrees[6] + GRIPPER_OFFSET) * GRIPPER_MULTIPLER)
        return radians

    def write_joints(self, joints: list[float], speed: float) -> None:
        """Nothing to do, no motors on the leader"""

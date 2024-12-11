from abc import ABC, abstractmethod

from node_helpers.parameters import Choosable


class BaseRobotProtocol(Choosable, ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def connect(self) -> None:
        """Connect to the robot, or re-connect if connection has been lost"""

    @abstractmethod
    def deactivate_servos(self) -> None:
        """Safely deactivate all servos on the robot"""

    @abstractmethod
    def read_joints(self) -> list[float]:
        """Read the current joint radians of the robot, in 'leader' space always"""

    @abstractmethod
    def write_joints(self, joints: list[float], speed: float) -> None:
        """Write the joint radians to the robot, in 'leader' space always"""

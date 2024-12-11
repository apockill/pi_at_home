from abc import ABC, abstractmethod

from node_helpers.parameters import Choosable


class BaseRobotProtocol(Choosable, ABC):
    def __init__(self) -> None:
        super().__init__()

        self.connect()

    @abstractmethod
    def connect(self) -> None:
        """Connect to the robot, or re-connect if connection has been lost"""

    @abstractmethod
    def deactivate_servos(self) -> None:
        """Safely deactivate all servos on the robot"""

    @abstractmethod
    def read_joints(self) -> list[float]:
        """Read the current joint angles of the robot, in 'follower' space always"""

    @abstractmethod
    def write_joints(self, joints: list[float], speed: float) -> None:
        """Write the joint angles to the robot, in 'follower' space always"""

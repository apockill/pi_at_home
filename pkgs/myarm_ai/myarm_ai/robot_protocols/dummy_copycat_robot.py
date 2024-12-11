from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol


class DummyCopycatRobot(BaseRobotProtocol):
    """A robot who always sets it's current position to the desired position"""

    class Parameters(BaseModel):
        initial_joint_positions: list[float] = [0.0] * 7

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()
        self.params = parameters or self.Parameters()
        self._joint_positions = self.params.initial_joint_positions

    def connect(self) -> None:
        pass

    def deactivate_servos(self) -> None:
        pass

    def read_joints(self) -> list[float]:
        """Move the joints back and forth, and return the new positions"""
        return self._joint_positions[:]

    def write_joints(self, joints: list[float], speed: float) -> None:
        self._joint_positions = joints

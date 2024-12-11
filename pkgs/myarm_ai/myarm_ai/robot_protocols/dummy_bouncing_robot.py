from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol


class DummyBouncingRobot(BaseRobotProtocol):
    """A robot whose joints constantly move back and forth"""

    class Parameters(BaseModel):
        joint_mins: list[float] = [0.0] * 7
        joint_maxs: list[float] = [0.5] * 7
        joint_speeds: list[float] = [0.01] * 7

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()

        self.params = parameters or self.Parameters()

        self._joint_positions = self.params.joint_mins[:]
        self._joint_goals = self.params.joint_maxs[:]
        self._joint_speeds = self.params.joint_speeds[:]

    def connect(self) -> None:
        pass

    def deactivate_servos(self) -> None:
        self._joint_speeds = [0] * len(self._joint_speeds)

    def read_joints(self) -> list[float]:
        """Move the joints back and forth, and return the new positions"""
        for i in range(len(self._joint_positions)):
            if self._joint_positions[i] >= self._joint_goals[i]:
                self._joint_speeds[i] = -abs(self._joint_speeds[i])
            elif self._joint_positions[i] <= self.params.joint_mins[i]:
                self._joint_speeds[i] = abs(self._joint_speeds[i])

            self._joint_positions[i] += self._joint_speeds[i]

        return self._joint_positions[:]

    def write_joints(self, joints: list[float], speed: float) -> None:
        self._joint_goals = joints
        self._joint_speeds = [speed] * len(joints)

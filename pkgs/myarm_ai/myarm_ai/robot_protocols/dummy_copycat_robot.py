from time import time

from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol


class _MockMotor:
    def __init__(self, position: float = 0.0, speed: float = 0.0):
        self.position = position
        self.speed = speed
        self.target_position = position
        self.last_update = time()

    def set_target_position(self, target_position: float, speed: float) -> None:
        self.target_position = target_position
        self.speed = speed

    def update(self) -> None:
        now = time()
        delta = now - self.last_update
        self.last_update = now

        # Distance the motor would move if it continued at 'speed' for the full delta
        max_move = self.speed * delta
        distance_to_target = self.target_position - self.position

        # If we can reach or surpass the target in 'delta' time,
        # just set the position to the target. Otherwise, move partway there.
        if abs(distance_to_target) <= max_move:
            self.position = self.target_position
        else:
            direction = 1 if distance_to_target > 0 else -1
            self.position += direction * max_move


class DummyCopycatRobot(BaseRobotProtocol):
    """
    A mock robot that moves each joint toward a target position at a specified speed.
    """

    class Parameters(BaseModel):
        speed: float = 0.35  # Radians per second
        n_joints: int = 7

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()
        self.params = parameters or self.Parameters()

        # Create a _MockMotor for each joint, all starting at 0.0
        self._motors = [
            _MockMotor(speed=self.params.speed) for _ in range(self.params.n_joints)
        ]

    def connect(self) -> None:
        pass

    def deactivate_servos(self) -> None:
        pass

    def read_joints(self) -> list[float]:
        """Update each motor's position toward its individual target."""
        for motor in self._motors:
            motor.update()
        return [motor.position for motor in self._motors]

    def write_joints(self, joints: list[float], speed: float) -> None:
        """Set new target positions and speed for each of the joints."""
        if len(joints) != self.params.n_joints:
            raise ValueError(
                f"Expected {self.params.n_joints} joint positions, got {len(joints)}"
            )

        # Update each motor's target and speed
        for motor, target_position in zip(self._motors, joints, strict=False):
            motor.set_target_position(target_position, speed)

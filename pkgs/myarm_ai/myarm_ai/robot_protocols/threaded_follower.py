from threading import Event, RLock, Thread

from pydantic import BaseModel

from .base_robot_protocol import BaseRobotProtocol
from .myarm_follower import MyArmFollowerRobot


class ThreadedReaderWriterRobot(BaseRobotProtocol):
    """An optimized BaseRobotProtocol wrapper that is optimized for fulfulling the role
    of a follower robot, which requires fast 'read' AND 'writes'.

    It has three optimizations:
    - Is constantly calling the underlying 'read_joints' method and caching it
    - Calls write_joints ASAP after receiving a message
    - Avoids calling write_joints if the joints are the same as the last message
    """

    class Parameters(BaseModel):
        robot_protocol: type[BaseRobotProtocol] = MyArmFollowerRobot

    def __init__(self, parameters: Parameters | None = None):
        super().__init__()

        self.params = parameters or self.Parameters()
        self._robot = self.params.robot_protocol()

        # Hold the last known joint positions of this robot
        self._last_joints_read: list[float] = self._robot.read_joints()

        # Hold the last requested position and speed to move to
        self._last_write_request: tuple[list[float], float] | None = None

        # Threading
        self._thread = Thread(target=self._server_loop)
        self._handle_lock = RLock()
        self._exit_event = Event()
        self._thread.start()

    def _server_loop(self) -> None:
        while not self._exit_event.is_set():
            with self._handle_lock:
                self._last_joints_read = self._robot.read_joints()
                if self._last_write_request is not None:
                    joints, speed = self._last_write_request

                    if joints != self._last_joints_read:
                        self._robot.write_joints(joints, speed)
                    self._last_write_request = None

    def connect(self) -> None:
        with self._handle_lock:
            self._robot.connect()

    def deactivate_servos(self) -> None:
        with self._handle_lock:
            self._robot.deactivate_servos()

    def read_joints(self) -> list[float]:
        return self._last_joints_read

    def write_joints(self, joints: list[float], speed: float) -> None:
        self._last_write_request = (joints, speed)

    def close(self) -> None:
        self._exit_event.set()
        self._thread.join()

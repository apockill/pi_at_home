from pydantic import BaseModel


class RobotTimeSample(BaseModel):
    """A recording for a single robot, for a single episode"""

    joint_positions: list[float]
    isaac_time: float
    ros_time: float


class RobotRecording(BaseModel):
    """A recording for a single robot, for a single episode"""

    joint_names: list[str] = []
    time_samples: list[RobotTimeSample] = []
    root_joint: str
    """The prim path of the Ariticulation of the 0th joint in the robot"""


class EpisodeJointsRecording(BaseModel):
    """A recording for a list of robots, for a single episode"""

    robots: dict[str, RobotRecording]


class TrajectoryRecordingMeta(BaseModel):
    """Metadata for an episode"""

    start_time: float = 0.0
    end_time: float


class LeRobotTimestep(BaseModel):
    """Closely mimics one row of a episode *.parquet file from a lerobot dataset"""

    action: list[float]
    state: list[float]
    timestamp: float
    frame_index: int
    episode_index: int
    index: int  # Same as frame_index, but for the entire dataset
    task_index: int

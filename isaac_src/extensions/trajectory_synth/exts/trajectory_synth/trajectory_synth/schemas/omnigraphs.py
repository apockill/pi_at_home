from pydantic import BaseModel

from .constants import RECORDER_ACTION_GRAPH_PATH


class RobotAttributes(BaseModel):
    root_joint: str
    ros_namespace: str
    joint_topic: str = "current_joint_states"

    @property
    def name(self):
        return self.ros_namespace.strip("/")

    @property
    def omnigraph_path(self):
        return f"{RECORDER_ACTION_GRAPH_PATH}/{self.name}"

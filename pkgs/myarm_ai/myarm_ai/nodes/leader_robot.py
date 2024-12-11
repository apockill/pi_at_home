from typing import Any

from node_helpers.spinning import create_spin_function

from myarm_ai.urdfs import MyArmCURDF

from .base_robot_node import BaseRobotNode


class RobotLeaderNode(BaseRobotNode):
    def __init__(self, **kwargs: Any):
        super().__init__("RobotLeaderNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.urdf = MyArmCURDF


main = create_spin_function(RobotLeaderNode)

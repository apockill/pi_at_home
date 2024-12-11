from typing import Any

from node_helpers.spinning import create_spin_function

from myarm_ai.nodes.base_robot_node import BaseRobotNode
from myarm_ai.urdfs import MyArmMURDF


class RobotFollowerNode(BaseRobotNode):
    def __init__(self, **kwargs: Any):
        super().__init__("RobotFollowerNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.urdf = MyArmMURDF


main = create_spin_function(RobotFollowerNode)

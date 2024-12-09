from typing import Any

from node_helpers.nodes import HelpfulNode
from node_helpers.spinning import create_spin_function
from pydantic import BaseModel
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import JointState

from myarm_ai.urdfs import MyArmMURDF


class RobotFollowerNode(HelpfulNode):
    class Parameters(BaseModel):
        pass

    def __init__(self, **kwargs: Any):
        super().__init__("RobotFollowerNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.urdf = MyArmMURDF.with_namespace(self.get_namespace())

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, "desired_joint_states", qos_profile_services_default
        )

    def on_publish_joints(self) -> None:
        pass


main = create_spin_function(RobotFollowerNode)

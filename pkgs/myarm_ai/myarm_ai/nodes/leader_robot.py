from typing import Any

from node_helpers.nodes import HelpfulNode
from node_helpers.spinning import create_spin_function
from pydantic import BaseModel
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import JointState

from myarm_ai.urdfs import MyArmCURDF


class RobotLeaderNode(HelpfulNode):
    class Parameters(BaseModel):
        joint_publish_hz: float = 10.0

    def __init__(self, **kwargs: Any):
        super().__init__("RobotLeaderNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.urdf = MyArmCURDF.with_namespace(self.get_namespace())

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, "desired_joint_states", qos_profile_services_default
        )

        # Create timers
        self.create_timer(1 / self.params.joint_publish_hz, self.on_publish_joints)

        # For kicks, move joints forwards and backwards on repeat
        # Mapping of dict[joint_name, tuple(position, velocity)]
        self.joint_positions = {
            self.urdf.joints.JOINT_1: (0.0, 0.01),
            self.urdf.joints.JOINT_2: (0.0, 0.02),
            self.urdf.joints.JOINT_3: (0.0, 0.03),
            self.urdf.joints.JOINT_4: (0.0, 0.04),
            self.urdf.joints.JOINT_5: (0.0, 0.05),
            self.urdf.joints.JOINT_6: (0.0, 0.06),
            self.urdf.joints.GRIPPER: (0.0, 0.07),
        }

    def on_publish_joints(self) -> None:
        joint_positions = {}

        for joint_name, (position, velocity) in self.joint_positions.items():
            joint_positions[joint_name] = position
            self.joint_positions[joint_name] = (position + velocity, velocity)

            if position > 1.0:
                self.joint_positions[joint_name] = (1.0, -velocity)
            elif position < -1.0:
                self.joint_positions[joint_name] = (-1.0, velocity)

        joint_state = JointState()
        joint_state.name = list(joint_positions.keys())
        joint_state.position = list(joint_positions.values())
        self.joint_state_publisher.publish(joint_state)


main = create_spin_function(RobotLeaderNode)

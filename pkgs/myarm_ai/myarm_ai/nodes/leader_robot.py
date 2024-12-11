from typing import Any

from node_helpers.nodes import HelpfulNode
from node_helpers.spinning import create_spin_function
from pydantic import BaseModel
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import JointState

from myarm_ai.robot_protocols import BaseRobotProtocol
from myarm_ai.urdfs import MyArmCURDF


class RobotLeaderNode(HelpfulNode):
    class Parameters(BaseModel):
        joint_publish_hz: float = 10.0
        robot_protocol: type[BaseRobotProtocol]

    def __init__(self, **kwargs: Any):
        super().__init__("RobotLeaderNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.urdf = MyArmCURDF
        self.robot_protocol = self.params.robot_protocol()

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, "desired_joint_states", qos_profile_services_default
        )

        # Create timers
        self.create_timer(1 / self.params.joint_publish_hz, self.on_publish_joints)

    def on_publish_joints(self) -> None:
        # Map the joint names to the joint positions from the robot protocol
        joint_readings = self.robot_protocol.read_joints()
        joint_positions = {
            self.urdf.joints.JOINT_1: joint_readings[0],
            self.urdf.joints.JOINT_2: joint_readings[1],
            self.urdf.joints.JOINT_3: joint_readings[2],
            self.urdf.joints.JOINT_4: joint_readings[3],
            self.urdf.joints.JOINT_5: joint_readings[4],
            self.urdf.joints.JOINT_6: joint_readings[5],
            self.urdf.joints.GRIPPER: joint_readings[6],
        }

        joint_state = JointState()
        joint_state.name = list(joint_positions.keys())
        joint_state.position = list(joint_positions.values())
        self.joint_state_publisher.publish(joint_state)


main = create_spin_function(RobotLeaderNode)

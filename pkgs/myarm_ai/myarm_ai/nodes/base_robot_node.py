from abc import ABC
from typing import Any

from node_helpers.nodes import HelpfulNode
from pydantic import BaseModel
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import JointState

from myarm_ai.robot_protocols import BaseRobotProtocol
from myarm_ai.urdfs import MyArmCURDF, MyArmMURDF


class BaseRobotNode(HelpfulNode, ABC):
    """A base class creating ROS nodes that control a robot"""

    urdf: type[MyArmCURDF] | type[MyArmMURDF]

    class Parameters(BaseModel):
        joint_publish_hz: float = 10.0
        robot_protocol: type[BaseRobotProtocol]

        # In joint units
        move_speed: float

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.robot_protocol = self.params.robot_protocol()
        self.robot_protocol.connect()

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, "desired_joint_states", qos_profile_services_default
        )
        # Set motor states and speed
        self.motor_state_subscriber = self.create_subscription(
            JointState,
            "motor_commands",
            self.on_motor_states,
            qos_profile=qos_profile_sensor_data,
        )

        # Create timers
        self.create_timer(1 / self.params.joint_publish_hz, self.on_publish_joints)

    def on_motor_states(self, msg: JointState) -> None:
        self.robot_protocol.write_joints(msg.position, self.params.move_speed)

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

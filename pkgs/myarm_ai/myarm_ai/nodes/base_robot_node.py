from abc import ABC
from typing import Any

from node_helpers.nodes import HelpfulNode
from pydantic import BaseModel
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState

from myarm_ai.robot_protocols import BaseRobotProtocol
from myarm_ai.urdfs import MyArmCURDF, MyArmMURDF

last_pos = [0, 0, 0, 0, 0, 0, 0]


class BaseRobotNode(HelpfulNode, ABC):
    """A base class creating ROS nodes that control a robot"""

    urdf: type[MyArmCURDF] | type[MyArmMURDF]

    class Parameters(BaseModel):
        # This will run faster than the read/write speed
        joint_publish_hz: float = 100.0
        robot_protocol: type[BaseRobotProtocol]

        # In joint units
        move_speed: float

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.robot_protocol = self.params.robot_protocol()
        self.robot_protocol.connect()

        # Create publishers
        reliable_latest_receiver = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, "current_joint_states", reliable_latest_receiver
        )
        # Set motor states and speed
        self.motor_state_subscriber = self.create_subscription(
            JointState, "motor_commands", self.on_motor_states, reliable_latest_receiver
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
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(joint_positions.keys())
        joint_state.position = list(joint_positions.values())
        self.joint_state_publisher.publish(joint_state)

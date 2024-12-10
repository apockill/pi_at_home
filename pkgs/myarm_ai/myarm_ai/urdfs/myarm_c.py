"""This module describes the URDF for the MyArm C650 robot arm."""

from typing import NamedTuple

from node_helpers.urdfs import URDFConstants


class MyArmCJoints(NamedTuple):
    JOINT_1: str = "leader_joint1"
    JOINT_2: str = "leader_joint2"
    JOINT_3: str = "leader_joint3"
    JOINT_4: str = "leader_joint4"
    JOINT_5: str = "leader_joint5"
    JOINT_6: str = "leader_joint6"

    GRIPPER: str = "leader_gripper"


class MyArmMFrames(NamedTuple):
    BASE_LINK: str = "leader_base"


MyArmCURDF = URDFConstants[MyArmCJoints, MyArmMFrames](
    from_package="myarm_ai",
    registration_name="myarm_c",
    urdf_paths=[(None, "urdfs/myarm_c/myarm_c650.urdf")],
    joints=MyArmCJoints(),
    frames=MyArmMFrames(),
)

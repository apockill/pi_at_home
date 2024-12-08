"""This module describes the URDF for the MyArm M robot arm."""

from typing import NamedTuple

from node_helpers.urdfs import URDFConstants


class MyArmMJoints(NamedTuple):
    JOINT_1: str = "joint1"
    JOINT_2: str = "joint2"
    JOINT_3: str = "joint3"
    JOINT_4: str = "joint4"
    JOINT_5: str = "joint5"
    JOINT_6: str = "joint6"

    GRIPPER: str = "gripper"


class MyArmMFrames(NamedTuple):
    BASE_LINK: str = "base"


MyArmMURDF = URDFConstants[MyArmMJoints, MyArmMFrames](
    from_package="myarm_ai",
    registration_name="myarm_m",
    urdf_paths=[(None, "urdfs/myarm_m/myarm_m.urdf")],
    joints=MyArmMJoints(),
    frames=MyArmMFrames(),
)

"""This module describes the URDF for the MyArm M robot arm."""

from typing import NamedTuple

from node_helpers.urdfs import URDFConstants


class MyArmMJoints(NamedTuple):
    JOINT_1: str = "follower_joint1"
    JOINT_2: str = "follower_joint2"
    JOINT_3: str = "follower_joint3"
    JOINT_4: str = "follower_joint4"
    JOINT_5: str = "follower_joint5"
    JOINT_6: str = "follower_joint6"

    GRIPPER: str = "follower_gripper"


class MyArmMFrames(NamedTuple):
    BASE_LINK: str = "follower_base"


MyArmMURDF = URDFConstants[MyArmMJoints, MyArmMFrames](
    from_package="myarm_ai",
    registration_name="myarm_m",
    urdf_paths=[(None, "urdfs/myarm_m/myarm_m.urdf")],
    joints=MyArmMJoints(),
    frames=MyArmMFrames(),
)

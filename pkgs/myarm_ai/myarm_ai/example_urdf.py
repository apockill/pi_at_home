"""This module demonstrates how to define a `node_helpers` URDFConstants object for a
basic URDF, specify necessary joint and frame names, and register the URDF with the
`node_helpers` package.

By registering it, the URDF can be accessed _by name_ in configuration files.
"""

from typing import NamedTuple

from node_helpers.urdfs import URDFConstants


class ForkliftJoints(NamedTuple):
    FORKS: str = "forks"
    FORKS_PARENT_DATUM: str = "forks_parent_datum"


class ForkliftFrames(NamedTuple):
    BASE_LINK: str = "forklift_body"

    # Joint tracking
    FORKS_ORIGIN: str = "forks_origin"
    FORKS: str = "forks"

ForkliftURDF = URDFConstants[ForkliftJoints, ForkliftFrames](
    from_package="node_helpers",
    registration_name="forklift",
    urdf_paths=[(None, "sample_urdfs/forklift/robot.urdf")],
    joints=ForkliftJoints(),
    frames=ForkliftFrames(),
)

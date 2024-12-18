"""Launch nodes for this launch profile."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

# Import the URDF module so it registers all available URDFs with the URDFConstants
from myarm_ai import urdfs  # noqa: F401
from node_helpers import launching
from node_helpers.parameters import ParameterLoader
from pydantic import BaseModel


class MetaParameters(BaseModel):
    """This is a great place to put parameters that affect the generation of the launch
    file. Don't put node-specific configuration in here, rather, put configuration for
    what nodes you want to be created in the first place.

    Read more about this functionality under docs/parameters.rst
    """


def generate_launch_description() -> LaunchDescription:
    # Create a parameter loader to parse all yaml files in the launch-profile/parameters
    # directory, and then apply overrides from the override file, if one exists.
    param_loader: ParameterLoader[MetaParameters] = ParameterLoader(
        parameters_directory=Path("/robot/launch-profile/parameters/"),
        override_file=Path("/robot/launch-profile/parameters.override.yaml"),
        meta_parameters_schema=MetaParameters,
    )


    launch_description = [
        Node(
            package="myarm_ai",
            executable="follower_robot",
            parameters=[param_loader.ros_parameters_file],
            namespace="follower",
            remappings=[
                ("motor_commands", "/leader/current_joint_states"),
            ],
        ),
        Node(
            package="myarm_ai",
            executable="leader_robot",
            parameters=[param_loader.ros_parameters_file],
            namespace="leader",
        ),
    ]
    return LaunchDescription(launch_description)

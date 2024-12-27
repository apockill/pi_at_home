import logging
import random
from pathlib import Path

import omni.ext
import omni.graph.core as og
import omni.graph.core.types as ot
import omni.kit.commands
from omni import ui

from . import path_utils, schemas
from .episode import EpisodeRecording


class TrajectoryRecorderExtension(omni.ext.IExt):
    # State Variables
    recording: bool
    current_episode: EpisodeRecording | None
    current_joint_recording: schemas.EpisodeJointsRecording | None

    def on_startup(self, ext_id):
        logging.info("[trajectory_synth] trajectory_synth startup")

        # State Variables
        self.recording = False
        self.recordings_dir = schemas.DEFAULT_RECORDINGS_DIR

        # Set in 'start_recording'
        self.current_episode = None
        self.current_joint_recording = None

        # UI Window
        self._window = ui.Window("Trajectory Recorder", width=400, height=400)
        with self._window.frame, ui.VStack(spacing=10):
            # Recordings Directory Input
            with ui.HStack(spacing=10):
                ui.Label("Recordings Directory:", width=150)
                self.directory_field = ui.StringField()
                self.directory_field.model.set_value(str(self.recordings_dir))

            # Add Robot Attributes UI for Leader and Follower Robots
            self.leader_robot_attributes = self._add_robot_attributes(
                "Leader Robot",
                default_root_joint="/World/leader/root_joint",
                default_ros_namespace="/leader/",
            )
            self.follower_robot_attributes = self._add_robot_attributes(
                "Follower Robot",
                default_root_joint="/World/follower/root_joint",
                default_ros_namespace="/follower/",
            )

            # Status Label
            self.status_label = ui.Label("Ready", alignment=ui.Alignment.CENTER)

            # Single Toggle Button for Start/Stop Recording
            self.toggle_recording_button = ui.Button(
                "Start Recording", clicked_fn=self.toggle_recording
            )

    def _add_robot_attributes(
        self, robot_name: str, default_root_joint: str, default_ros_namespace: str
    ):
        """Dynamically add UI fields for a robot's attributes."""
        section_name = f"{robot_name} Controller Attributes"
        robot_attrs = schemas.RobotAttributes(
            root_joint=default_root_joint, ros_namespace=default_ros_namespace
        )
        with ui.CollapsableFrame(section_name, height=200), ui.VStack(spacing=5):
            # Root Joint
            with ui.HStack(spacing=10):
                ui.Label("Root Joint:", width=150)
                root_joint_field = ui.StringField()
                root_joint_field.model.set_value(robot_attrs.root_joint)

            # ROS Namespace
            with ui.HStack(spacing=10):
                ui.Label("ROS Namespace:", width=150)
                ros_namespace_field = ui.StringField()
                ros_namespace_field.model.set_value(robot_attrs.ros_namespace)

            # Joint Topic
            with ui.HStack(spacing=10):
                ui.Label("Joint Topic:", width=150)
                joint_topic_field = ui.StringField()
                joint_topic_field.model.set_value(robot_attrs.joint_topic)

            # Add callbacks to update RobotAttributes
            root_joint_field.model.add_value_changed_fn(
                lambda model, r_name=robot_name: setattr(
                    robot_attrs, "root_joint", model.get_value_as_string()
                )
            )
            ros_namespace_field.model.add_value_changed_fn(
                lambda model, r_name=robot_name: setattr(
                    robot_attrs, "ros_namespace", model.get_value_as_string()
                )
            )
            joint_topic_field.model.add_value_changed_fn(
                lambda model, r_name=robot_name: setattr(
                    robot_attrs, "joint_topic", model.get_value_as_string()
                )
            )

        return robot_attrs

    def toggle_recording(self):
        try:
            if self.recording:
                self.stop_recording()
            else:
                self.start_recording()
        except Exception as e:
            self.update_status(f"Error: {type(e).__class__.__name__}: {e}")
            raise

    def start_recording(self):
        # TODO: Prevent this being called multiple times while recording

        # Reset the timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        timeline.set_current_time(0)

        # Get directory from the user input
        self.recordings_dir = Path(self.directory_field.model.get_value_as_string())
        self.recordings_dir.mkdir(parents=True, exist_ok=True)

        # Create a new episode directory
        self.current_episode = EpisodeRecording(
            path_utils.get_next_numbered_dir(self.recordings_dir, "episode"),
            expect_exists=False,
        )
        self.current_joint_recording = schemas.EpisodeJointsRecording(
            robots={
                self.leader_robot_attributes.name: schemas.RobotRecording(
                    root_joint=self.leader_robot_attributes.root_joint
                ),
                self.follower_robot_attributes.name: schemas.RobotRecording(
                    root_joint=self.follower_robot_attributes.root_joint
                ),
            }
        )

        # Save the current scene, so the replaying code knows the exact scene used
        # Always delete the existing omnigraphs, since we don't want the saved scene to
        # include the ROS joint -> articulation controller graph
        self.clear_omni_graphs()
        stage = omni.usd.get_context().get_stage()
        scene_file_path = self.current_episode.scene_path
        stage.GetRootLayer().Export(str(scene_file_path))
        logging.info(f"Scene saved to {scene_file_path}")

        # Generate OmniGraph
        self.set_up_omnigraph(self.leader_robot_attributes)
        self.set_up_omnigraph(self.follower_robot_attributes)

        # Start the recording
        omni.kit.commands.execute(
            "StartRecording",
            target_paths=[("/World", True)],
            live_mode=False,
            use_frame_range=False,
            start_frame=0,
            end_frame=0,
            use_preroll=False,
            preroll_frame=0,
            record_to="FILE",
            fps=0,
            apply_root_anim=False,
            increment_name=False,
            record_folder=str(self.current_episode.directory),
            take_name=schemas.TIMESTEPS_FILENAME,
        )

        self.recording = True
        self.toggle_recording_button.text = "Stop Recording"
        self.update_status("Recording started...")
        timeline.play()

    def stop_recording(self):
        # Stop the recording
        omni.kit.commands.execute("StopRecording")

        # Stop the timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()

        self.recording = False
        self.toggle_recording_button.text = "Start Recording"
        self.update_status("Recording stopped.")

        # Write the metadata file
        metadata_file_path = self.current_episode.metadata_path
        metadata = schemas.TrajectoryRecordingMeta(end_time=timeline.get_current_time())
        metadata_file_path.write_text(metadata.model_dump_json())

        # Write the joint recordings file
        self.current_episode.joints_recording_path.write_text(
            self.current_joint_recording.model_dump_json()
        )

        # Create the 'renders' directory, then validate the recording
        self.current_episode.renders_dir.mkdir(parents=True, exist_ok=False)

        # Validate the recording
        if not self.current_episode.validate():
            self.update_status("Recording failed validation. Somethings wrong!")
            return

    def clear_omni_graphs(self):
        """Remove existing OmniGraph if it exists"""
        stage = omni.usd.get_context().get_stage()

        for graph_path in [
            self.leader_robot_attributes.omnigraph_path,
            self.follower_robot_attributes.omnigraph_path,
        ]:
            existing_graph = stage.GetPrimAtPath(graph_path)
            if existing_graph and existing_graph.IsValid():
                logging.info(f"Removing existing OmniGraph at {graph_path}")
                omni.kit.commands.execute("DeletePrims", paths=[graph_path])

    def set_up_omnigraph(self, robot_attributes: schemas.RobotAttributes):

        # Create a custom node to process joint state data with a unique name,
        # so we can reinitialize the extension without conflicts
        recorder_name = f"JointRecorder_{random.randint(0, 10000000)}"

        @og.create_node_type(unique_name=recorder_name)
        def custom_joint_state_processor(
            input_joint_names: ot.tokenarray,
            input_positions: ot.doublearray,
            timestamp: ot.double,
            ros_timestamp: ot.double,
            exec_in: ot.execution,
        ) -> ot.string:
            """Custom node to process joint state data.

            In the future this could be used to record joint state data to a file.
            Currently it's a proof of concept that doesn't do anything, but is hooked up
            to the graph.
            """
            robot_recording = self.current_joint_recording.robots[robot_attributes.name]
            robot_recording.joint_names = input_joint_names
            robot_recording.time_samples.append(
                schemas.RobotTimeSample(
                    joint_positions=input_positions,
                    isaac_time=timestamp,
                    ros_time=ros_timestamp,
                )
            )
            return ""

        # Create and configure the graph
        graph = {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (
                    "SubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("ProcessJointState", recorder_name),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ProcessJointState.inputs:exec_in"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ProcessJointState.inputs:input_joint_names",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ProcessJointState.inputs:input_positions",
                ),
                ("SubscribeJointState.outputs:timeStamp", "ProcessJointState.inputs:ros_timestamp"),
                (
                    "OnPlaybackTick.outputs:time",
                    "ProcessJointState.inputs:timestamp",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "ArticulationController.inputs:targetPrim",
                    robot_attributes.root_joint,
                ),
                (
                    "SubscribeJointState.inputs:nodeNamespace",
                    robot_attributes.ros_namespace,
                ),
                ("SubscribeJointState.inputs:topicName", robot_attributes.joint_topic),
            ],
        }

        og.Controller.edit(
            {
                "graph_path": robot_attributes.omnigraph_path,
                "evaluator_name": "execution",
            },
            graph,
        )

    def update_status(self, message):
        self.status_label.text = message
        logging.info(f"[trajectory_synth] Status: {message}")

    def on_shutdown(self):
        logging.info("[trajectory_synth] trajectory_synth shutdown")

from pathlib import Path

import omni.ext
import omni.kit.commands
from omni import ui
from pxr import Sdf

from . import path_utils, schema


class TrajectoryRecorderExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_synth startup")

        # State Variables
        self.recording = False
        self.scene_loaded = False
        self.mover_omnigraph_path = (
            "/robot/isaac_src/assets/omnigraphs/mover_actiongraph.usd"
        )
        self.recordings_dir = schema.DEFAULT_RECORDINGS_DIR

        # UI Window
        self._window = ui.Window("Trajectory Recorder", width=400, height=200)
        with self._window.frame, ui.VStack(spacing=10):
            # Mover Omnigraph Path Input
            with ui.HStack(spacing=10):
                ui.Label("Mover Omnigraph Path:", width=150)
                self.mover_omnigraph_field = ui.StringField()
                self.mover_omnigraph_field.model.set_value(self.mover_omnigraph_path)

            # Recordings Directory Input
            with ui.HStack(spacing=10):
                ui.Label("Recordings Directory:", width=150)
                self.directory_field = ui.StringField()
                self.directory_field.model.set_value(str(self.recordings_dir))

            # Status Label
            self.status_label = ui.Label("Ready", alignment=ui.Alignment.CENTER)

            # Single Toggle Button for Start/Stop Recording
            self.toggle_recording_button = ui.Button(
                "Start Recording", clicked_fn=self.toggle_recording
            )

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

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
        self.current_episode = schema.TrajectoryRecording(
            path_utils.get_next_numbered_dir(self.recordings_dir, "episode"),
            expect_exists=False,
        )

        # Save the current scene without the omnigraph
        self.ensure_omnigraph_loaded(loaded=False)
        stage = omni.usd.get_context().get_stage()
        scene_file_path = self.current_episode.scene_path
        stage.GetRootLayer().Export(str(scene_file_path))
        print(f"Scene saved to {scene_file_path}")

        # Ensure the omnigraph is loaded
        self.ensure_omnigraph_loaded(loaded=True)

        # Start the recording
        omni.kit.commands.execute(
            "StartRecording",
            target_paths=[("/World", True)],  # Adjust the target paths as needed
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
            take_name=schema.TIMESTEPS_FILENAME,
        )

        self.recording = True
        self.toggle_recording_button.text = "Stop Recording"
        self.update_status("Recording started...")
        timeline.play()  # Start the timeline playback

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
        metadata = schema.TrajectoryRecordingMeta(end_time=timeline.get_current_time())
        metadata_file_path.write_text(metadata.model_dump_json())

        # Create the 'renders' directory, then validate the recording
        self.current_episode.renders_dir.mkdir(parents=True, exist_ok=False)

        # Validate the recording
        if not self.current_episode.validate():
            self.update_status("Recording failed validation. Somethings wrong!")
            return

    def ensure_omnigraph_loaded(self, loaded: bool) -> None:
        # Check or remove the omnigraph as needed
        stage = omni.usd.get_context().get_stage()
        omnigraph_prim = stage.GetPrimAtPath("/World/MoverOmnigraph")
        if loaded:
            if not omnigraph_prim or not omnigraph_prim.IsValid():
                print(f"Loading mover omnigraph: {self.mover_omnigraph_path}")
                omni.kit.commands.execute(
                    "CreateReferenceCommand",
                    usd_context=omni.usd.get_context(),
                    path_to=Sdf.Path("/World/MoverOmnigraph"),
                    asset_path=self.mover_omnigraph_path,
                    prim_path=Sdf.Path(),
                    instanceable=False,
                    select_prim=False,
                )
                self.update_status("Mover omnigraph loaded.")
            else:
                print("Mover omnigraph already loaded.")
        elif omnigraph_prim and omnigraph_prim.IsValid():
            print("Removing existing mover omnigraph.")
            omni.kit.commands.execute("DeletePrims", paths=["/World/MoverOmnigraph"])
            self.update_status("Mover omnigraph removed.")

    def update_status(self, message):
        # Update the status label
        self.status_label.text = message

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_synth shutdown")

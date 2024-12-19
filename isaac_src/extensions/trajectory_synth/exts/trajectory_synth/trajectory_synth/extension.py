import omni.ext
import omni.ui as ui
import omni.kit.commands
import os
import re


class TrajectorySynthExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_synth startup")

        # State Variables
        self.recording = False
        self.recordings_dir = "/robot/synthetic-output/recordings/"  # Default directory

        # UI Window
        self._window = ui.Window("Trajectory Synth", width=400, height=200)
        with self._window.frame:
            with ui.VStack(spacing=10):
                # Recordings Directory Input
                with ui.HStack(spacing=10):
                    ui.Label("Recordings Directory:", width=150)
                    self.directory_field = ui.StringField()
                    self.directory_field.model.set_value(self.recordings_dir)

                # Status Label
                self.status_label = ui.Label("Ready", alignment=ui.Alignment.CENTER)

                # Single Toggle Button for Start/Stop Recording
                self.toggle_recording_button = ui.Button("Start Recording", clicked_fn=self.toggle_recording)

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        # Get directory from the user input
        self.recordings_dir = self.directory_field.model.get_value_as_string()
        if not os.path.exists(self.recordings_dir):
            os.makedirs(self.recordings_dir)

        # Define the recording file path
        take_name = f"episode"

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
            record_to="NEW_LAYER",
            fps=0,
            apply_root_anim=False,
            increment_name=True,
            record_folder=self.recordings_dir,
            take_name=take_name,
        )

        self.recording = True
        self.toggle_recording_button.text = "Stop Recording"
        self.update_status("Recording started...")

    def stop_recording(self):
        # Stop the recording
        omni.kit.commands.execute("StopRecording")

        self.recording = False
        self.toggle_recording_button.text = "Start Recording"
        self.update_status("Recording stopped.")

    def update_status(self, message):
        # Update the status label
        self.status_label.text = message

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_synth shutdown")

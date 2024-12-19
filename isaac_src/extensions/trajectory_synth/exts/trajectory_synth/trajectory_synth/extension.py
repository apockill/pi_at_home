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
        self._window = ui.Window("Trajectory Synth", width=300, height=150)
        with self._window.frame:
            with ui.VStack():
                # Recordings Directory Input
                with ui.HStack():
                    ui.Label("Recordings Directory:")
                    self.directory_field = ui.StringField()
                    self.directory_field.model.set_value(self.recordings_dir)

                # Status Label
                self.status_label = ui.Label("")
                self.update_status("Ready")

                # Buttons for Start and Stop Recording
                with ui.HStack():
                    ui.Button("Start Recording", clicked_fn=self.start_recording)
                    ui.Button("Stop Recording", clicked_fn=self.stop_recording)

    def start_recording(self):
        if self.recording:
            self.update_status("Recording already in progress.")
            return

        # Get directory from the user input
        self.recordings_dir = self.directory_field.model.get_value_as_string()
        if not os.path.exists(self.recordings_dir):
            os.makedirs(self.recordings_dir)


        # Define the recording file path
        take_name = f"episode_"

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
            increment_name=True,
            record_folder=self.recordings_dir,
            take_name=take_name,
        )

        self.recording = True
        self.update_status(f"Recording episode...")

    def stop_recording(self):
        if not self.recording:
            self.update_status("No recording in progress.")
            return

        # Stop the recording
        omni.kit.commands.execute("StopRecording")

        self.recording = False
        self.update_status("Recording stopped.")

    def get_next_episode_number(self, directory):
        # Scan the directory for existing episode files
        episodes = []
        if os.path.exists(directory):
            for filename in os.listdir(directory):
                match = re.match(r"episode_(\d+)", filename)
                if match:
                    episodes.append(int(match.group(1)))

        # Return the next episode number
        return max(episodes, default=0) + 1

    def update_status(self, message):
        # Update the status label
        self.status_label.text = message

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_synth shutdown")

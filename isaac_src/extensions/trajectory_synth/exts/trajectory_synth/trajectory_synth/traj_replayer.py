import os
from pathlib import Path

import omni.ext
import omni.kit.commands
import omni.usd
from omni import ui
from pxr import Sdf

from .schema import DEFAULT_RECORDINGS_DIR


class TrajectoryReplayerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_replayer startup")
        # UI Window
        self._window = ui.Window("Trajectory Replayer", width=400, height=200)
        with self._window.frame:
            with ui.VStack(spacing=10):
                # Recordings Directory Input
                with ui.HStack(spacing=10):
                    ui.Label("Recordings Directory:", width=150)
                    self.recordings_dir_field = ui.StringField()
                    self.recordings_dir_field.model.set_value(DEFAULT_RECORDINGS_DIR)

                # Episode Number Input
                with ui.HStack(spacing=10):
                    ui.Label("Episode Number:", width=150)
                    self.episode_number_field = ui.IntField()
                    self.episode_number_field.model.set_value(0)

                # Replay Button
                self.replay_button = ui.Button("Replay", clicked_fn=self.replay_episode)

    def replay_episode(self):
        # Get user inputs
        recordings_dir = self.recordings_dir_field.model.get_value_as_string()
        episode_number = self.episode_number_field.model.get_value_as_int()

        if not Path(recordings_dir).is_dir():
            print("Please provide a valid recordings directory and episode number.")
            return

        # Construct paths
        episode_dir = os.path.join(recordings_dir, f"episode_{episode_number:04d}")
        scene_usd_path = os.path.join(episode_dir, "scene.usd")
        timesteps_usd_path = os.path.join(episode_dir, "timesteps.usd")

        # Validate paths
        if not os.path.exists(scene_usd_path):
            print(f"Scene file not found: {scene_usd_path}")
            return

        if not os.path.exists(timesteps_usd_path):
            print(f"Timesteps file not found: {timesteps_usd_path}")
            return

        # Step 1: New Scene
        omni.usd.get_context().new_stage()
        print("New scene created.")

        # Step 2: Insert timesteps.usd and scene.usd
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_layer.subLayerPaths.append(timesteps_usd_path)
        root_layer.subLayerPaths.append(scene_usd_path)

        # Step 4: Set scene.usd as the authoring layer, without this replay won't work
        scene_layer = Sdf.Layer.FindOrOpen(scene_usd_path)
        if scene_layer:
            stage.SetEditTarget(scene_layer)
            print(f"Set {scene_usd_path} as the authoring layer.")
        else:
            print(f"Failed to set {scene_usd_path} as the authoring layer.")

        # Step 5: Start the timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print("Timeline started.")

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_replayer shutdown")

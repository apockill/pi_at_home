from pathlib import Path

import omni.ext
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from omni import ui
from pxr import Sdf

from . import path_utils
from .schema import DEFAULT_RECORDINGS_DIR


class TrajectoryReplayerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_replayer startup")
        self._build_ui()

    def _build_ui(self):
        self._window = ui.Window("Trajectory Replayer", width=400, height=300)
        with self._window.frame, ui.VStack(spacing=10):
            # Recordings Directory Input
            with ui.HStack(spacing=10):
                ui.Label("Recordings Directory:", width=150)
                self.recordings_dir_field = ui.StringField()
                self.recordings_dir_field.model.set_value(DEFAULT_RECORDINGS_DIR)

            # Episode Number Input
            with ui.HStack(spacing=10):
                ui.Label("Episode Number:", width=150)
                self.episode_number_field = ui.IntField()
                self.episode_number_field.model.set_value(1)

            # Camera Selection
            with ui.HStack(spacing=10):
                ui.Label("Select Cameras:", width=150)
                self.camera_selector = ui.StringField()
                self.camera_selector.model.set_value("top,wrist")
                self.camera_selector.tooltip = (
                    "Enter a comma-separated list of camera names, referenced by "
                    "their prim name"
                )

            # Resolution Width Input
            with ui.HStack(spacing=10):
                ui.Label("Resolution Width:", width=150)
                self.resolution_width_field = ui.IntField()
                self.resolution_width_field.model.set_value(320)

                ui.Label("Resolution Height:", width=150)
                self.resolution_height_field = ui.IntField()
                self.resolution_height_field.model.set_value(240)

            # Replay Button
            self.replay_button = ui.Button("Render", clicked_fn=self.replay_episode)
            self.status_label = ui.Label("", alignment=ui.Alignment.CENTER)

    def update_status(self, message):
        self.status_label.text = message

    def replay_episode(self):
        # Get user inputs
        recordings_dir = Path(self.recordings_dir_field.model.get_value_as_string())
        episode_number = self.episode_number_field.model.get_value_as_int()
        resolution_width = self.resolution_width_field.model.get_value_as_int()
        resolution_height = self.resolution_height_field.model.get_value_as_int()
        selected_camera_names = [
            name.strip()
            for name in self.camera_selector.model.get_value_as_string().split(",")
        ]

        episode_path = recordings_dir / f"episode_{episode_number:03d}"
        if not episode_path.is_dir():
            self.update_status("Invalid recordings directory or episode number.")
            return

        # Construct paths
        scene_usd_path = episode_path / "scene.usd"
        timesteps_usd_path = episode_path / "timesteps.usd"

        # Validate paths
        if not scene_usd_path.exists():
            self.update_status(f"Scene file not found: {scene_usd_path}")
            return

        if not timesteps_usd_path.exists():
            self.update_status(f"Timesteps file not found: {timesteps_usd_path}")
            return

        # Step 1: New Scene
        omni.usd.get_context().new_stage()
        self.update_status("New scene created.")

        # Step 2: Insert timesteps.usd and scene.usd
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_layer.subLayerPaths.append(str(timesteps_usd_path))
        root_layer.subLayerPaths.append(str(scene_usd_path))

        # Step 3: Set scene.usd as the authoring layer
        scene_layer = Sdf.Layer.FindOrOpen(str(scene_usd_path))
        if scene_layer:
            stage.SetEditTarget(scene_layer)
            self.update_status(f"Set {scene_usd_path} as the authoring layer.")
        else:
            self.update_status(
                f"Failed to set {scene_usd_path} as the authoring layer."
            )
            return

        # Step 4: Validate Cameras
        selected_cameras = self.get_cameras_from_names(selected_camera_names)
        if len(selected_cameras) != len(selected_camera_names):
            return

        # Step 5: Initialize Replicator and Create Render Products
        render_products = []

        for camera in selected_cameras:
            resolution = (resolution_width, resolution_height)

            render_product = rep.create.render_product(
                str(camera.GetPath()), resolution
            )
            render_products.append((camera.GetName(), render_product))

        # Step 6: Attach Writer to Render Products
        renders_dir = episode_path / "renders"
        render_output_path = path_utils.get_next_numbered_dir(renders_dir, "render")
        for camera_name, render_product in render_products:
            camera_output_path = render_output_path / camera_name
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=str(camera_output_path), rgb=True)
            writer.attach([render_product])

        # Step 7: Trigger Rendering on Each Frame
        timeline = omni.timeline.get_timeline_interface()
        num_frames = int(
            timeline.get_end_time() * timeline.get_time_codes_per_seconds()
        )
        with rep.trigger.on_frame(num_frames=num_frames):
            pass  # The writer will handle capturing frames

        # Step 8: Start the timeline
        timeline.play()
        self.update_status("Timeline started and rendering frames.")

    def get_cameras_from_names(self, selected_camera_names: list[str]):
        available_cameras = self.get_scene_cameras()
        available_camera_names = [camera.GetName() for camera in available_cameras]

        invalid_cameras = [
            name for name in selected_camera_names if name not in available_camera_names
        ]
        if invalid_cameras:
            self.update_status(
                f"Invalid cameras: {', '.join(invalid_cameras)}. "
                f"Available cameras: {', '.join(available_camera_names)}"
            )
            return []
        return [
            camera
            for camera in available_cameras
            if camera.GetName() in selected_camera_names
        ]

    def get_scene_cameras(self):
        stage = omni.usd.get_context().get_stage()
        return [
            prim
            for prim in stage.Traverse()
            if prim.GetTypeName() == "Camera" and prim.GetName()
        ]

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_replayer shutdown")

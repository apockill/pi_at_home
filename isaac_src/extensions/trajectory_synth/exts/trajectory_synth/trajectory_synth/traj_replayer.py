import asyncio
from pathlib import Path

import omni.ext
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from omni import ui
from pxr import Sdf

from . import path_utils, schema


class TrajectoryReplayerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_replayer startup")
        self.randomization_config = DomainRandomization()
        self._build_ui()

    def _build_ui(self):
        self._window = ui.Window("Trajectory Replayer", width=400, height=400)
        with self._window.frame, ui.VStack(spacing=10):
            # Recordings Directory Input
            with ui.HStack(spacing=10):
                ui.Label("Recordings Directory:", width=150)
                self.recordings_dir_field = ui.StringField()
                self.recordings_dir_field.model.set_value(schema.DEFAULT_RECORDINGS_DIR)

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

            # Resolution Input
            with ui.HStack(spacing=10):
                ui.Label("Resolution Width:", width=150)
                self.resolution_width_field = ui.IntField()
                self.resolution_width_field.model.set_value(320)

                ui.Label("Resolution Height:", width=150)
                self.resolution_height_field = ui.IntField()
                self.resolution_height_field.model.set_value(240)

            # Number of Renders Input
            with ui.HStack(spacing=10):
                ui.Label("Number of Renders:", width=150)
                self.num_renders_field = ui.IntField()
                self.num_renders_field.model.set_value(1)

            # Render Button
            self.render_button = ui.Button(
                "Render",
                clicked_fn=lambda: asyncio.ensure_future(self.render_multiple_times()),
            )
            self.status_label = ui.Label("", alignment=ui.Alignment.CENTER)

    def update_status(self, message):
        self.status_label.text = message

    async def render_multiple_times(self) -> None:
        self.render_button.enabled = False
        num_renders = self.num_renders_field.model.get_value_as_int()
        for i in range(num_renders):
            self.update_status(f"Starting render {i + 1} of {num_renders}...")
            await self.replay_episode(render_index=i)
        self.update_status("All renders completed.")
        self.render_button.enabled = True

    async def replay_episode(self, render_index: int) -> None:
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
        try:
            traj_recording = schema.TrajectoryRecording(episode_path)
        except FileNotFoundError:
            self.update_status("Invalid trajectory recording, file missing.")
            return

        # Step 1: New Scene
        omni.usd.get_context().new_stage()
        self.update_status("New scene created.")

        # Step 2: Insert timesteps.usd and scene.usd
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_layer.subLayerPaths.append(str(traj_recording.timesteps_path))
        root_layer.subLayerPaths.append(str(traj_recording.scene_path))

        # Step 3: Set scene.usd as the authoring layer
        scene_layer = Sdf.Layer.FindOrOpen(str(traj_recording.scene_path))
        if scene_layer:
            stage.SetEditTarget(scene_layer)
            self.update_status(
                f"Set {traj_recording.scene_path} as the authoring layer."
            )
        else:
            self.update_status(
                f"Failed to set {traj_recording.scene_path} as the authoring layer."
            )
            return

        # Step 4: Validate Cameras
        selected_cameras = self.get_cameras_from_names(selected_camera_names)
        if len(selected_cameras) != len(selected_camera_names):
            return

        # Step 5: Initialize Replicator and Create Render Products
        render_products = []

        for camera in selected_cameras:
            camera_name = camera.GetName()
            resolution = (resolution_width, resolution_height)
            randomization_config = self.randomization_config.camera_params[camera_name]

            # Apply randomization using Replicator
            camera_path = str(camera.GetPath())
            camera_prim = rep.get.prims(camera_path)
            with camera_prim:
                rep.modify.pose(
                    position=randomization_config.position_distribution,
                    rotation=randomization_config.rotation_distribution,
                )

            render_product = rep.create.render_product(camera_path, resolution)
            render_products.append((camera.GetName(), render_product))

        # Step 6: Attach Writer to Render Products
        renders_dir = episode_path / "renders"
        render_output_path = path_utils.get_next_numbered_dir(renders_dir, "render")
        for camera_name, render_product in render_products:
            camera_output_path = render_output_path / camera_name
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=str(camera_output_path), rgb=True)
            writer.attach([render_product])

        # Step 7: Determine Frame Range from Timesteps USD
        timesteps_layer = Sdf.Layer.FindOrOpen(str(traj_recording.timesteps_path))
        if not timesteps_layer:
            self.update_status(f"Failed to load {traj_recording.timesteps_path}.")
            return

        assert timesteps_layer.HasEndTimeCode()

        time_codes = timesteps_layer.ListAllTimeSamples()
        if not time_codes:
            self.update_status(
                f"No time samples found in {traj_recording.timesteps_path}."
            )
            return

        # Step 8: Control Timeline Manually
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_current_time(0.0)
        await omni.kit.app.get_app().next_update_async()

        framerate = 30  # TODO: Make framerate randomized
        end_time = traj_recording.metadata.end_time
        final_frame = int(round(end_time * framerate))

        for frame in range(final_frame):
            timeline.set_current_time(frame / framerate)
            timeline.forward_one_frame()
            await omni.kit.app.get_app().next_update_async()
            self.update_status(
                f"Rendering frame {frame}/{final_frame} of render {render_index + 1}"
            )

        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self.update_status(f"Render {render_index + 1} completed.")

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

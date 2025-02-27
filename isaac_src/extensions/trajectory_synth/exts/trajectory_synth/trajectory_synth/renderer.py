import asyncio
import random
from pathlib import Path
from typing import Literal

import omni.ext
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import yaml
from omni import ui
from pxr import Sdf
from pydantic import ValidationError

from . import path_utils, schemas
from .episode import EpisodeRecording
from .lerobot_episode import LeRobotEpisodeRecording


class TrajectoryRendererExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[trajectory_synth] trajectory_renderer startup")
        self._build_ui()

    def _build_ui(self):
        self._window = ui.Window("Trajectory Renderer", width=400, height=500)
        with self._window.frame, ui.VStack(spacing=10):
            # Recordings Directory Input
            with ui.HStack(spacing=10):
                ui.Label("Recordings Directory:", width=150)
                self.recordings_dir_field = ui.StringField()
                self.recordings_dir_field.model.set_value(
                    str(schemas.DEFAULT_RECORDINGS_DIR)
                )

            with ui.HStack(spacing=10):
                ui.Label("Mesh Textures Directory:", width=150)
                self.mesh_textures_dir_field = ui.StringField()
                self.mesh_textures_dir_field.model.set_value(
                    str(schemas.DEFAULT_MESH_TEXTURES_DIR)
                )
                self.skybox_textures_dir_field = ui.StringField()
                self.skybox_textures_dir_field.model.set_value(
                    str(schemas.DEFAULT_SKYBOX_TEXTURES_DIR)
                )

            # Episode Number Input
            with ui.HStack(spacing=10):
                ui.Label("Episode Number:", width=150)
                self.episode_number_field = ui.IntField()
                self.episode_number_field.model.set_value(-1)

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
                ui.Label("Target Number of Renders:", width=150)
                self.num_renders_field = ui.IntField()
                self.num_renders_field.model.set_value(1)

            # Target FPS
            with ui.HStack(spacing=10):
                ui.Label("Target FPS", width=150)
                self.target_fps_field = ui.IntField()
                self.target_fps_field.model.set_value(30)

            # Render Settings
            with ui.CollapsableFrame("Randomization Settings", height=200):
                self.render_settings_field = ui.StringField(multiline=True)
                default_randomization = (
                    schemas.RandomizationDistributions().model_dump()
                )
                self.render_settings_field.model.set_value(
                    yaml.safe_dump(default_randomization, sort_keys=False)
                )

            # Render Button
            self.render_button = ui.Button(
                "Render",
                clicked_fn=lambda: asyncio.ensure_future(self.render_multiple_times()),
            )
            self.status_label = ui.Label("", alignment=ui.Alignment.CENTER)

    def update_status(self, message):
        print(f"[trajectory_synth] Status: {message}")
        self.status_label.text = message

    async def render_multiple_times(self) -> None:
        self.render_button.enabled = False
        num_renders = self.num_renders_field.model.get_value_as_int()

        # Load DomainRandomization from UI
        try:
            randomization_config = schemas.RandomizationDistributions(
                **yaml.safe_load(self.render_settings_field.model.get_value_as_string())
            )
        except ValidationError as e:
            self.update_status(f"Invalid Render Settings: {e}")
            self.render_button.enabled = True
            return

        # If the chosen episode == -1, select all episodes
        episode_chosen = self.episode_number_field.model.get_value_as_int()
        if episode_chosen < 0:
            episodes = [
                int(str(p.name).split("_")[-1])
                for p in Path(
                    self.recordings_dir_field.model.get_value_as_string()
                ).iterdir()
            ]
        else:
            episodes = [episode_chosen]

        # For each episode, render the episode num_renders times
        for i in range(num_renders):
            for episode_number in episodes:
                self.update_status(f"Starting render {i + 1} of {num_renders}...")
                recordings_dir = Path(
                    self.recordings_dir_field.model.get_value_as_string()
                )
                episode_path = recordings_dir / f"episode_{episode_number:03d}"
                traj_recording = EpisodeRecording(episode_path)

                next_render_number = path_utils.get_next_number_in_dir(
                    traj_recording.renders_dir, "render"
                )
                if next_render_number > i + 1:
                    self.update_status(
                        f"Skipping episode {episode_number} in favor of other episodes."
                    )
                    continue

                render_path = path_utils.get_next_numbered_dir(
                    traj_recording.renders_dir, "render"
                )

                try:
                    await self.replay_episode(
                        traj_recording=traj_recording,
                        render_path=render_path,
                        render_index=i,
                        randomization_config=randomization_config,
                    )
                except Exception as e:
                    self.update_status(f"Failed to render: {e}")
                    raise
                finally:
                    self.render_button.enabled = True

        self.update_status("All renders completed.")

    async def replay_episode(
        self,
        traj_recording: EpisodeRecording,
        render_path: Path,
        render_index: int,
        randomization_config: schemas.RandomizationDistributions,
    ) -> None:
        # Get user inputs
        resolution_width = self.resolution_width_field.model.get_value_as_int()
        resolution_height = self.resolution_height_field.model.get_value_as_int()

        # Create paths and recording objects

        joints_recording = LeRobotEpisodeRecording(
            render_dir=render_path,
            recording=traj_recording.joints_recording,
            target_fps=self.target_fps_field.model.get_value_as_int(),
        )
        selected_camera_names = [
            name.strip()
            for name in self.camera_selector.model.get_value_as_string().split(",")
        ]

        # Step 1: New Scene
        omni.usd.get_context().new_stage()
        self.update_status("New scene created.")

        # Step 2: Insert timesteps.usd and scene.usd
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_layer.subLayerPaths.append(str(traj_recording.timesteps_path))
        root_layer.subLayerPaths.append(str(traj_recording.scene_path))

        # Step 3: Set scene.usd as the authoring layer
        #         Without this, timesteps don't seem to play when the timeline starts
        scene_layer = Sdf.Layer.FindOrOpen(str(traj_recording.scene_path))
        stage.SetEditTarget(scene_layer)
        self.update_status(f"Set {traj_recording.scene_path} as the authoring layer.")

        # Step 4: Validate Cameras
        selected_cameras = self.get_cameras_from_names(selected_camera_names)
        if len(selected_cameras) != len(selected_camera_names):
            return

        # Step 5: Initialize Replicator and Create Render Products
        self.update_status("Initializing replicator configuration...")
        self._set_up_replicator(
            config=randomization_config,
            camera_names=selected_cameras,
            render_resolution=(resolution_width, resolution_height),
            render_path=render_path,
            mesh_textures_path=Path(
                self.mesh_textures_dir_field.model.get_value_as_string()
            ),
            skylight_textures_path=Path(
                self.skybox_textures_dir_field.model.get_value_as_string()
            ),
        )

        # Step 6: Control Timeline Manually
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_current_time(0.0)
        await omni.kit.app.get_app().next_update_async()

        framerate = random.randint(
            randomization_config.min_render_fps, randomization_config.max_render_fps
        )
        end_time = traj_recording.metadata.end_time
        final_frame = int(round(end_time * framerate))

        for frame_index in range(final_frame):
            current_time = frame_index / framerate
            timeline.set_current_time(current_time)
            timeline.forward_one_frame()
            await rep.orchestrator.step_async()
            await omni.kit.app.get_app().next_update_async()

            joints_recording.add_timestep(current_time)
            self.update_status(
                f"Rendering frame {frame_index}/{final_frame}. Render: {render_index} "
                f"Episode: {traj_recording.directory.name}"
            )

        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        joints_recording.save()

        self.update_status(f"Render {render_index} completed.")

    def get_cameras_from_names(self, selected_camera_names: list[str]):
        available_cameras = self.get_stage_prims("Camera")
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

    def get_stage_prims(self, type_name: Literal["Camera"], keep_matches: bool = True):
        stage = omni.usd.get_context().get_stage()
        return [
            prim
            for prim in stage.Traverse()
            if (prim.GetTypeName() == type_name) is keep_matches
        ]

    def on_shutdown(self):
        print("[trajectory_synth] trajectory_renderer shutdown")

    @staticmethod
    def _set_up_replicator(
        config: schemas.RandomizationDistributions,
        render_path: Path,
        mesh_textures_path: Path,
        skylight_textures_path: Path,
        camera_names: list[str],
        render_resolution: tuple[int, int],
    ):
        # Create cameras and randomize their positions
        # TODO: Consider using custom events for more explicit triggering
        #       https://forums.developer.nvidia.com/t/replicator-and-rep-orchestrator-step/314079
        render_products = []
        rep.set_global_seed(random.randint(0, 10000))  # Not strictly necessary
        for camera in camera_names:
            camera_name = camera.GetName()
            camera_configs = config.camera_params[camera_name]
            chosen_for_camera = random.choice(camera_configs)

            # Apply randomization using Replicator
            camera_path = str(camera.GetPath())
            camera_prim = rep.get.prims(camera_path)
            with camera_prim:
                rep.modify.pose(
                    position=chosen_for_camera.position_distribution,
                    rotation=chosen_for_camera.rotation_distribution,
                )

            render_product = rep.create.render_product(camera_path, render_resolution)
            render_products.append((camera.GetName(), render_product))

        # Randomize textures of all prims. Since project_uvw has a drastic effect on
        # the appearance of the textures, we set randomize the project_uvw flag as well.
        mesh_textures = [
            str(t) for t in path_utils.get_all_textures_in_dir(mesh_textures_path)
        ]
        prims_to_apply_materials = rep.get.mesh()
        materials = [
            rep.create.material_omnipbr(
                # Create random material properties
                diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
                roughness=rep.distribution.uniform(0, 1),
                metallic=rep.distribution.choice([0, 1]),
                emissive_color=rep.distribution.uniform((0, 0, 0.5), (0, 0, 1)),
                emissive_intensity=rep.distribution.uniform(0, 1000),
                # Texturize the material properties
                diffuse_texture=rep.distribution.choice(mesh_textures),
                roughness_texture=rep.distribution.choice(mesh_textures),
                metallic_texture=rep.distribution.choice(mesh_textures),
                emissive_texture=rep.distribution.choice(mesh_textures),
                count=random.randint(config.min_materials, config.max_materials),
                project_uvw=project_uvw,
            )
            for project_uvw in [True, False]
        ]
        rep.randomizer.materials(
            materials=materials, input_prims=prims_to_apply_materials
        )

        # Randomize positions of preconfigured distractor prims
        for prim_path, distractor_config in config.distractor_params.items():
            distractor_prims = rep.get.prim_at_path(prim_path)
            print(f"{prim_path=} {distractor_prims=}")
            with distractor_prims:
                rep.modify.pose(
                    position=distractor_config.position_distribution,
                    rotation=distractor_config.rotation_distribution,
                    scale=distractor_config.scale_distribution,
                )

        # Create a skybox with a texture
        skylight_textures = [
            str(t) for t in path_utils.get_all_textures_in_dir(skylight_textures_path)
        ]
        rep.create.light(
            rotation=rep.distribution.uniform((0, -180, -180), (0, 180, 180)),
            intensity=rep.distribution.uniform(50, 1200),
            temperature=rep.distribution.normal(6500, 1000),
            light_type="dome",
            texture=rep.distribution.choice(skylight_textures),
            name="Skybox",
        )
        with rep.get.light():
            rep.modify.pose(
                position=config.light_pos_offset,
                rotation=config.light_rot_offset,
            )

        # Create output directory for renders
        for camera_name, render_product in render_products:
            camera_output_path = render_path / camera_name
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=str(camera_output_path), rgb=True)
            writer.attach([render_product])

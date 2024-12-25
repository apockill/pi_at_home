from pathlib import Path

import omni.replicator.core as rep
from pydantic import BaseModel

TIMESTEPS_FILENAME = "timesteps"  # usd
SCENE_FILENAME = "scene"  # usd
METADATA_FILENAME = "metadata.json"
DEFAULT_MESH_TEXTURES_DIR = Path("/robot/isaac_src/assets/textures/random")
DEFAULT_SKYBOX_TEXTURES_DIR = Path("/robot/isaac_src/assets/textures/skyboxes")
DEFAULT_RECORDINGS_DIR = Path("/robot/synthetic-output/recordings/")

BOUND = tuple[float, float, float]


class TrajectoryRecordingMeta(BaseModel):
    start_time: float = 0.0
    end_time: float


class TrajectoryRecording:
    def __init__(self, directory: Path, expect_exists: bool = True):
        self.directory = directory
        self.renders_dir = directory / "renders"
        self.metadata_path = directory / METADATA_FILENAME
        self.scene_path = directory / f"{SCENE_FILENAME}.usd"
        self.timesteps_path = directory / f"{TIMESTEPS_FILENAME}.usd"

        if expect_exists and not self.validate():
            raise FileNotFoundError("Incomplete trajectory recording!")

    def validate(self):
        return all(
            [
                self.metadata_path.is_file(),
                self.scene_path.is_file(),
                self.timesteps_path.is_file(),
                self.renders_dir.is_dir(),
            ]
        )

    @property
    def metadata(self) -> TrajectoryRecordingMeta:
        return TrajectoryRecordingMeta.model_validate_json(
            self.metadata_path.read_text()
        )


class CameraDistribution(BaseModel):
    pos_offset_max: tuple[float, float, float]
    pos_offset_min: tuple[float, float, float] | None = None

    rot_offset_max: tuple[float, float, float]
    rot_offset_min: tuple[float, float, float] | None = None

    @property
    def position_distribution(self):
        return self._uniform(self.pos_offset_max, self.pos_offset_min)

    @property
    def rotation_distribution(self):
        return self._uniform(self.rot_offset_max, self.rot_offset_min)

    def _uniform(self, max_bound: BOUND, min_bound: BOUND | None):
        if min_bound is None:
            min_bound = [-n for n in max_bound]
        # Sanity check
        assert all(min_bound[i] <= max_bound[i] for i in range(3))
        return rep.distribution.uniform(min_bound, max_bound)


class RandomizationDistributions(BaseModel):
    # Choose how fast or slow the robot moves. This allows you to "stretch" or
    # "squish" the trajectory in time.
    min_render_fps: int = 10
    max_render_fps: int = 50

    # Choose number of materials to randomize
    min_materials: int = 1
    max_materials: int = 50

    # Choose how much to modify lighting position
    light_pos_offset: tuple[float, float, float] = (0.1, 0.1, 0.1)
    light_rot_offset: tuple[float, float, float] = (45, 45, 45)

    # Camera name -> randomization params
    # One is randomly chosen per full trajectory run
    camera_params: dict[str, list[CameraDistribution]] = {
        "top": [
            # Vary rotation as much as possible
            CameraDistribution(
                pos_offset_min=(0, 0, 0),
                pos_offset_max=(0, 0, 0),
                rot_offset_min=(-10.0, -6.0, -5.0),  # Validated
                rot_offset_max=(1.0, 6.0, 5.0),  # Validated
            ),
            # Vary position around within a small range
            CameraDistribution(
                pos_offset_min=(-0.08, -0.05, -0.025),  # Validated
                pos_offset_max=(0.08, 0.02, 0.25),  # Validated
                rot_offset_min=(0, 0, 0),
                rot_offset_max=(0, 0, 0),
            ),
            # Keep the Z far from the normal position, allowing for drastic rotations
            # around the Z axis
            CameraDistribution(
                pos_offset_min=(0, 0, 0.15),
                pos_offset_max=(0, 0, 0.35),
                rot_offset_min=(0.0, 0, -50),
                rot_offset_max=(0.0, 0, 50),
            ),
        ],
        "wrist": [
            # Strategy: Set rot offset to 10, and maximize the position offset while
            #           keeping the picker tip in view
            CameraDistribution(
                pos_offset_max=(0.01, 0.005, 0.015),  # Validated
                rot_offset_min=[-15, -10, -10],  # Validated
                rot_offset_max=[7, 10, 10],  # Validated
            )
        ],
    }

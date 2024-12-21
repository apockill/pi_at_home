from pathlib import Path

from pydantic import BaseModel

TIMESTEPS_FILENAME = "timesteps"  # usd
SCENE_FILENAME = "scene"  # usd
METADATA_FILENAME = "metadata.json"
DEFAULT_RECORDINGS_DIR = "/robot/synthetic-output/recordings/"


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

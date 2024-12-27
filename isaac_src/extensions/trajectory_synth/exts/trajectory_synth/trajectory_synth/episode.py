from pathlib import Path

from . import schema


class EpisodeRecording:
    def __init__(self, directory: Path, expect_exists: bool = True):
        self.directory = directory
        self.renders_dir = directory / "renders"
        self.metadata_path = directory / schema.METADATA_FILENAME
        self.scene_path = directory / f"{schema.SCENE_FILENAME}.usd"
        self.timesteps_path = directory / f"{schema.TIMESTEPS_FILENAME}.usd"

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
    def metadata(self) -> schema.TrajectoryRecordingMeta:
        return schema.TrajectoryRecordingMeta.model_validate_json(
            self.metadata_path.read_text()
        )

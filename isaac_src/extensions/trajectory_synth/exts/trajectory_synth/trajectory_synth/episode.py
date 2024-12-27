from pathlib import Path

from . import schemas


class EpisodeRecording:
    def __init__(self, directory: Path, expect_exists: bool = True):
        self.directory = directory
        self.renders_dir = directory / "renders"
        self.metadata_path = directory / schemas.METADATA_FILENAME
        self.scene_path = directory / f"{schemas.SCENE_FILENAME}.usd"
        self.timesteps_path = directory / f"{schemas.TIMESTEPS_FILENAME}.usd"
        self.joints_recording_path = directory / "joints_recording.json"

        if expect_exists and not self.validate():
            raise FileNotFoundError("Incomplete trajectory recording!")

    def validate(self):
        return all(
            [
                self.metadata_path.is_file(),
                self.scene_path.is_file(),
                self.timesteps_path.is_file(),
                self.renders_dir.is_dir(),
                self.joints_recording_path.is_file(),
            ]
        )

    @property
    def metadata(self) -> schemas.TrajectoryRecordingMeta:
        return schemas.TrajectoryRecordingMeta.model_validate_json(
            self.metadata_path.read_text()
        )

    @property
    def joints_recording(self) -> schemas.EpisodeJointsRecording:
        return schemas.EpisodeJointsRecording.model_validate_json(
            self.joints_recording_path.read_text()
        )

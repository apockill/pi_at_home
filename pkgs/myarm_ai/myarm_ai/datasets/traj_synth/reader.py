import json
from collections.abc import Generator
from pathlib import Path
from typing import Any

import cv2
from pydantic import BaseModel, Field


class Timestep(BaseModel):
    """Hold all information for a single 'frame' in time for a given render"""

    images: dict[str, Path]
    """A list of paths to *.png files holding observations"""

    action: list[float]
    state: list[float] = Field(alias="observation.state")
    timestamp: float

    class Config:
        populate_by_name = True

    def to_lerobot_frame_dict(self) -> dict[str, Any]:
        """Turn this into something that can be fed to lerobot_datset.add_frame"""
        # Convert the timestep to a Lerobot frame dict
        frame_params = self.model_dump()

        # Convert the image paths to actual images
        images = frame_params.pop("images")
        for obs_name, image_path in images.items():
            lerobot_feature = f"observation.images.{obs_name}"
            image = cv2.imread(str(image_path))
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            frame_params[lerobot_feature] = image_rgb

        # Convert 'state' to 'observation.state'
        frame_params["observation.state"] = frame_params.pop("state")

        return frame_params


class RenderReader:
    def __init__(self, render_path: Path):
        """A path to a single Render"""
        self.render_path = render_path
        self.episode_json_path = render_path / "episode.json"

        if not self.render_path.is_dir():
            raise FileNotFoundError(f"Render {render_path} does not exist")
        if not self.episode_json_path.exists():
            raise FileNotFoundError(
                f"Render {render_path} does not contain an 'episode.json' file"
            )

    @property
    def _observation_paths(self) -> Generator[Path, None, None]:
        """Return directories in the render directory, all are assumed to be cameras"""
        for dir in self.render_path.iterdir():
            if dir.is_dir():
                yield dir

    def _image_for_observation(self, frame_idx: int, observation_dir: Path) -> Path:
        """Return the path to the image file for a given observation"""

        # Images are of format rgb_####.png
        image_path = observation_dir / f"rgb_{frame_idx:04d}.png"
        if not image_path.exists():
            raise FileNotFoundError(
                f"Observation {observation_dir} does not contain an '{image_path}' file"
            )
        return image_path

    @property
    def timesteps(self) -> Generator[Timestep, None, None]:
        json_lines = self.episode_json_path.read_text().splitlines()

        for line in json_lines:
            step = json.loads(line)
            images = {
                obs_dir.name: self._image_for_observation(step["frame_index"], obs_dir)
                for obs_dir in self._observation_paths
            }
            yield Timestep(
                images=images,
                action=step["action"],
                state=step["state"],  # type: ignore
                timestamp=step["timestamp"],
            )


class EpisodeReader:
    def __init__(self, episode_path: Path):
        """A path to a single Episode"""
        self.episode_path = episode_path
        self.renders_path = episode_path / "renders"

        if not self.episode_path.is_dir():
            raise FileNotFoundError(f"Episode {episode_path} does not exist")
        if not self.renders_path.exists():
            raise FileNotFoundError(
                f"Episode {episode_path} does not contain a 'renders' directory"
            )

    @property
    def renders(self) -> Generator[RenderReader, None, None]:
        for render_path in self.renders_path.iterdir():
            if render_path.is_dir():
                yield RenderReader(render_path)


class TrajectorySynthDatasetReader:
    def __init__(self, dataset_path: Path):
        """A path to a directory containing 'episode_####' directories"""
        self.dataset_path = dataset_path

        if not self.dataset_path.is_dir():
            raise FileNotFoundError(f"Dataset {dataset_path} does not exist")

    @property
    def episodes(self) -> Generator[EpisodeReader, None, None]:
        for episode_path in self.dataset_path.iterdir():
            if episode_path.is_dir():
                yield EpisodeReader(episode_path)

    @property
    def renders(self) -> Generator[RenderReader, None, None]:
        for episode in self.episodes:
            yield from episode.renders

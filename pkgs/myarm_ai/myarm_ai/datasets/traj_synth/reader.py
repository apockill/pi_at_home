from collections.abc import Generator
from pathlib import Path

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
    def renders(self) -> Generator[Path, None, None]:
        for episode in self.episodes:
            for render in episode.renders:
                yield render
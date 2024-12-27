from argparse import ArgumentParser
from pathlib import Path

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

from myarm_ai.datasets import TrajectorySynthDatasetReader


def convert(input_dataset: Path, output_dir: Path, fps: int) -> None:
    traj_dataset = TrajectorySynthDatasetReader(input_dataset)

    LeRobotDataset.create(
        repo_id="pi_at_home",
        root=output_dir,
        fps=fps
    )

    for episode in traj_dataset.episodes:
        print(episode)


def main() -> None:
    parser = ArgumentParser(
        description="Convert datasets created by this repositories 'Trajectory Synth' "
        "extension to the Lerobot dataset format"
    )
    parser.add_argument(
        "-i",
        "--episodes_dir",
        type=Path,
        help="Location of directory containing episode_####/ directories",
    )
    parser.add_argument("--fps", default=30, type=int, help="Target FPS for the dataset.")

    parser.add_argument(
        "-o", "--output_dir", type=Path, help="Where to write the Lerobot dataset"
    )
    args = parser.parse_args()

    convert(args.episodes_dir, args.output_dir, args.fps)


if __name__ == "__main__":
    pass

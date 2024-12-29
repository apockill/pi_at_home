import random
from argparse import ArgumentParser
from pathlib import Path

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

from myarm_ai.datasets import TrajectorySynthDatasetReader
from myarm_ai.datasets.lerobot import DEFAULT_MYARM_FEATURES


def convert(
    input_dataset: Path, output_dir: Path, fps: int, task: str
) -> LeRobotDataset:
    traj_dataset = TrajectorySynthDatasetReader(input_dataset)

    dataset = LeRobotDataset.create(
        repo_id="pi_at_home",
        root=output_dir,
        fps=fps,
        features=DEFAULT_MYARM_FEATURES,
    )

    shuffled_renders = list(traj_dataset.renders)
    random.shuffle(shuffled_renders)
    for render in shuffled_renders:
        for timestep in render.timesteps:
            frame_dict = timestep.to_lerobot_frame_dict()
            dataset.add_frame(frame_dict)

        dataset.save_episode(task=task)

    dataset.consolidate(run_compute_stats=True)
    return dataset


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
    parser.add_argument(
        "--fps", default=30, type=int, help="Target FPS for the dataset."
    )
    parser.add_argument("--task", type=str, help="Task name for the dataset.")
    parser.add_argument(
        "-o", "--output_dir", type=Path, help="Where to write the Lerobot dataset"
    )
    args = parser.parse_args()

    convert(
        input_dataset=args.episodes_dir,
        output_dir=args.output_dir,
        fps=args.fps,
        task=args.task,
    )


if __name__ == "__main__":
    pass

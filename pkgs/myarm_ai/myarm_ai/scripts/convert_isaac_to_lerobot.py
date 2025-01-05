import logging
import random
from argparse import ArgumentParser
from pathlib import Path

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

from myarm_ai.datasets import TrajectorySynthDatasetReader
from myarm_ai.datasets.lerobot import DEFAULT_MYARM_FEATURES


def convert(input_dataset: Path, repo_id: str, fps: int, task: str) -> LeRobotDataset:
    traj_dataset = TrajectorySynthDatasetReader(input_dataset)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=DEFAULT_MYARM_FEATURES,
    )

    shuffled_renders = list(traj_dataset.renders)

    # Deterministic shuffle
    shuffled_renders.sort()
    random.seed(1337)
    random.shuffle(shuffled_renders)

    # Add all frames to the dataset
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
        "-r", "--repo-id", type=str, help="The 'user/dataset' huggingface repo id."
    )
    parser.add_argument(
        "-u",
        "--upload",
        action="store_true",
        default=False,
        help="Upload the dataset to the huggingface dataset hub.",
    )
    args = parser.parse_args()

    dataset = convert(
        input_dataset=args.episodes_dir,
        repo_id=args.repo_id,
        fps=args.fps,
        task=args.task,
    )

    if args.upload:
        dataset.push_to_hub()

    logging.info(f"Dataset finished saving to {dataset.root}")


if __name__ == "__main__":
    pass

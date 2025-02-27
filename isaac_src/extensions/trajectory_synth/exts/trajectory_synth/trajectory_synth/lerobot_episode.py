from pathlib import Path
from typing import Literal

from . import schemas


class LeRobotEpisodeRecording:
    """Helper class for saving joint information in the same format as the lerobot
    episode_#####.parquet files, as CSVs.

    Holds similar information to the EpisodeJointsRecording. The 'rendering' references
    the fact that at this point in the pipeline, there is now a 'frame ID' associated
    with each sample; furthermore, each sample is now interpolated so that for each
    leader position there is a corresponding follower position.
    """

    def __init__(
        self,
        render_dir: Path,
        recording: schemas.EpisodeJointsRecording,
        target_fps: int,
        episode_index: int = 0,
        task_index: int = 0,
    ):
        self.recording = recording
        self.timesteps: list[schemas.LeRobotTimestep] = []
        self.save_path = render_dir / "episode.json"
        self.target_fps = target_fps
        self.episode_index = episode_index
        self.task_index = task_index

        self.leader_steps = recording.robots["leader"].time_samples
        self.follower_steps = recording.robots["follower"].time_samples

    def add_timestep(self, isaac_time: float):
        """Finds or interpolates the joint positions for each robot in at the given
        isaac_time and appends it to the current timesteps.
        """
        frame_index = (
            0 if len(self.timesteps) == 0 else self.timesteps[-1].frame_index + 1
        )

        # Interpolate joints
        leader_ros_joints = self._interpolate_joints(
            isaac_time, self.leader_steps, "real"
        )
        leader_sim_joints = self._interpolate_joints(
            isaac_time, self.leader_steps, "sim"
        )
        follower_ros_joints = self._interpolate_joints(
            isaac_time, self.follower_steps, "real"
        )
        follower_sim_joints = self._interpolate_joints(
            isaac_time, self.follower_steps, "sim"
        )

        # Modify isaac_time so it stretches or squishes such that 'target_fps' is hit
        # TODO: I need to look into this further to validate it's working as intended
        timestamp = frame_index / self.target_fps

        # Create a LeRobotTimestep
        # (Here we store leader joints in .action, follower in .state)
        new_timestep = schemas.LeRobotTimestep(
            ros_action=leader_ros_joints,
            ros_state=follower_ros_joints,
            sim_action=leader_sim_joints,
            sim_state=follower_sim_joints,
            timestamp=timestamp,
            frame_index=frame_index,
            episode_index=self.episode_index,
            index=frame_index,  # Sometimes index == frame_index
            task_index=self.task_index,
        )

        self.timesteps.append(new_timestep)

    def _interpolate_joints(
        self,
        time: float,
        time_samples: list[schemas.RobotTimeSample],
        joints_source: Literal["real", "sim"],
    ) -> list[float]:
        """
        Given a time and a particular robot_key (e.g. 'leader' or 'follower'),
        linearly interpolate that robot's joint positions at the requested time.

        Parameters
        ----------
        time : float
            The requested interpolation time
        robot_key : str
            The key for the robot in `self.recording.robots` whose joints we want

        Returns
        -------
        list[float]
            A list of interpolated joint positions for that robot
        """

        # Sort by isaac_time just in case they are somehow out of order
        time_samples.sort(key=lambda s: s.isaac_time)

        # Validate time is in-range
        if time <= time_samples[0].isaac_time:
            return time_samples[0].from_source(joints_source)
        if time >= time_samples[-1].isaac_time:
            raise ValueError(f"Time {time} is after the last sample")

        # Otherwise, find the two samples that bracket `time`
        for i in range(len(time_samples) - 1):
            t0 = time_samples[i].isaac_time
            t1 = time_samples[i + 1].isaac_time
            if t0 <= time <= t1:
                # Interpolate linearly
                ratio = (time - t0) / (t1 - t0)

                joints0 = time_samples[i].from_source(joints_source)
                joints1 = time_samples[i + 1].from_source(joints_source)
                interp_joints = [
                    j0 + ratio * (j1 - j0)
                    for j0, j1 in zip(joints0, joints1, strict=True)
                ]
                return interp_joints

        # Fallback (should not happen if times cover the entire range)
        return time_samples[-1].from_source(joints_source)

    def save(self):
        """Save the list as json lines"""
        with self.save_path.open("w") as f:
            for timestep in self.timesteps:
                f.write(timestep.model_dump_json() + "\n")

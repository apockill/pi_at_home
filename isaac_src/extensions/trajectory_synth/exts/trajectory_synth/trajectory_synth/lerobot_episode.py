from pathlib import Path

from . import schemas


class LeRobotEpisodeRecording:
    """Helper class for saving joint information in the same format as the lerobot
    episode_#####.parquet files, as CSVs.

    Holds similar information to the EpisodeJointsRecording. The 'rendering' references
    the fact that at this point in the pipeline, there is now a 'frame ID' associated
    with each sample; furthermore, each sample is now interpolated so that for each
    leader position there is a corresponding follower position.
    """

    def __init__(self, render_dir: Path, recording: schemas.EpisodeJointsRecording):
        self.recording = recording
        self.timesteps: list[schemas.LeRobotTimestep] = []
        self.save_path = render_dir / "episode.json"

        self.leader_steps = recording.robots["leader"].time_samples
        self.follower_steps = recording.robots["follower"].time_samples

    def add_timestep(
        self,
        isaac_time: float,
        frame_index: int | None = None,
        episode_index: int = 0,
        task_index: int = 0,
    ):
        """Finds or interpolates the joint positions for each robot in at the given
        isaac_time and appends it to the current timesteps.
        """
        frame_index = (
            0 if len(self.timesteps) == 0 else self.timesteps[-1].frame_index + 1
        )

        # Interpolate joints
        leader_joints = self._interpolate_joints(isaac_time, self.leader_steps)
        follower_joints = self._interpolate_joints(isaac_time, self.follower_steps)

        # Create a LeRobotTimestep
        # (Here we store leader joints in .action, follower in .state)
        new_timestep = schemas.LeRobotTimestep(
            action=leader_joints,
            state=follower_joints,
            timestamp=isaac_time,
            frame_index=frame_index,
            episode_index=episode_index,
            index=frame_index,  # Sometimes index == frame_index
            task_index=task_index,
        )

        self.timesteps.append(new_timestep)

    def _interpolate_joints(
        self, time: float, time_samples: list[schemas.RobotTimeSample]
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
            return time_samples[0].joint_positions
        if time >= time_samples[-1].isaac_time:
            raise ValueError(f"Time {time} is after the last sample")

        # Otherwise, find the two samples that bracket `time`
        for i in range(len(time_samples) - 1):
            t0 = time_samples[i].isaac_time
            t1 = time_samples[i + 1].isaac_time
            if t0 <= time <= t1:
                # Interpolate linearly
                ratio = (time - t0) / (t1 - t0)

                joints0 = time_samples[i].joint_positions
                joints1 = time_samples[i + 1].joint_positions
                interp_joints = [
                    j0 + ratio * (j1 - j0)
                    for j0, j1 in zip(joints0, joints1, strict=True)
                ]
                return interp_joints

        # Fallback (should not happen if times cover the entire range)
        return time_samples[-1].joint_positions

    def save(self):
        """Save the list as json lines"""
        with self.save_path.open("w") as f:
            for timestep in self.timesteps:
                f.write(timestep.model_dump_json() + "\n")

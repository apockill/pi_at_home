# pi_at_home

---
[![Test Status](https://github.com/apockill/pi_at_home/workflows/Test/badge.svg)](https://github.com/apockill/pi_at_home/actions?query=workflow%3ATest)
[![Lint Status](https://github.com/apockill/pi_at_home/workflows/Lint/badge.svg)](https://github.com/apockill/pi_at_home/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/apockill/pi_at_home/branch/main/graph/badge.svg)](https://codecov.io/gh/apockill/pi_at_home)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros-%230A0FF9.svg?logo=ros&logoColor=white)
---

This is my productionized deployment environment for my real world AI robot policies, 
and a place to create training data for reinforcement learning and imitation learning.

It's also the first project I've built using [create-ros-app](https://github.com/urbanmachine/create-ros-app),
a template I'm developing to make it easier for anyone to create and deploy production 
ready ROS2 applications.

## Project Roadmap
### Phase 1
Phase 1 focuses on leveraging imitation learning + synthetic data to improve the robustness
of imitation-learning datasets with regards to lighting, camera position, and other environmental
factors.

- [x] Develop a decent teleoperation interface for the MyArm M&C robot leader/follower arms
- [x] Add isaac-sim support for visualizing above arms
- [ ] **In Progress**: Learn how to use Replicator to multiplex trajectories of human demonstrations of robot tasks
      done in-simulation
- [ ] Create dataset collection tools based on lerobot dataset format
  - [x] Play around, train, and test lerobot policies.
        Done: [Now available via this fork of lerobot](https://github.com/huggingface/lerobot/pull/506)
  - [ ] Create ROS service for starting/stopping data collection and serializing datasets
- [ ] Create easy workflows for:
  - [ ] Record demonstrations with **real leader arm** and **simulation follower arm**
  - [x] Multiplex demonstrations using domain randomization, leveraging Replicator learnings above
  - [ ] Training models with mix of real and simulated data
- [ ] Benchmark the sim2real gap with this project, publicize results to open source community
- [ ] Add support for Koch arm and other open-source robot arm

### Phase 2
Add Reinforcement learning pipelines with ROS2 and Isaac-Sim support

### Phase 3
Support long-horizon tasks involving multiple policies, with a focus on VLMs and 
language-grounded interaction with the robot.

---

## Capabilities

### Simple Teleoperation
The simplest thing you can do is teleoperation with your robot arm copied in Isaac Sim.
To do that, run: 
```shell
docker/launch teleop_myarm
```
This will automatically build and run everything needed for the project.
Then, open http://localhost/ on your browser to view the project logs.

For in-depth documentation on the repository features, read the [About Template](docs/about_template.md) documentation.

### Synthetic Data Generation

There are three steps to synthetic data generation. First, open up the `synthetic_generation` launch profile:

```shell
docker/launch synthetic_generation
```

This will spin up Isaac Sim. Go to Extensions->3rd Party and turn on the "Trajectory Synth" extension.
This extension is custom built, and stored in `isaac_src/extensions`. Next, plug in your robot. When using a real world robot, you will need to add a `parameters.overrides.yaml` file to the 
root of `launch-profiles/synthetic_generation` to configure your specific robots drivers.

Next, record an episode! Configure the "Trajectory Recorder" window, and click "Start Recording":
![traj_recording.png](docs/static/traj_recording.png)

Once you've recorded an episode, you can re-render with domain randomization using the "Trajectory Renderer" extension:
![traj_replay.png](docs/static/traj_replay.png)

This will generate a configurable number of episodes while varying everything possible-
including joint interpolation to move the robot faster or slower than the original demonstration.

Finally, convert the rendered episodes to a dataset:
```shell
docker/run convert_isaac_to_lerobot \
  --episodes_dir /robot/synthetic-output/recordings \
  --output_dir /robot/synthetic-output/lerobot-fmt
```

### Dependencies

- [Docker](https://docs.docker.com/get-docker/), and optionally [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for hardware acceleration.
- [Poetry](https://python-poetry.org/docs/), in order to use linting tooling.

---
This repository was initialized by the [create-ros-app](https://github.com/UrbanMachine/create-ros-app) template. Contributions are welcome!

# pi_at_home
This is my productionized deployment environment for my real world AI robot policies, 
and a place to create training data for reinforcement learning and imitation learning.

It's also the first project I've built using [create-ros-app](https://github.com/urbanmachine/create-ros-app),
a template I'm developing to make it easier for anyone to create and deploy production 
ready ROS2 applications.

---
[![Test Status](https://github.com/apockill/pi_at_home/workflows/test.yaml/badge.svg)](https://github.com/apockill/pi_at_home/actions?query=workflow%3ATest)
[![Lint Status](https://github.com/apockill/pi_at_home/workflows/lint.yaml/badge.svg)](https://github.com/apockill/pi_at_home/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/apockill/pi_at_home/branch/main/graph/badge.svg)](https://codecov.io/gh/apockill/pi_at_home)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros-%230A0FF9.svg?logo=ros&logoColor=white)

---

## Running This Project

To run the project, use the following command:

```shell
docker/launch teleop_myarm
```
Then, open http://localhost/ on your browser to view the project logs.

For in-depth documentation on the repository features, read the [About Template](docs/about_template.md) documentation.

### Dependencies

- [Docker](https://docs.docker.com/get-docker/), and optionally [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for hardware acceleration.
- [Poetry](https://python-poetry.org/docs/), in order to use linting tooling.

---
This repository was initialized by the [create-ros-app](https://github.com/UrbanMachine/create-ros-app) template. Contributions are welcome!

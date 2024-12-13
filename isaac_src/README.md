# isaac_src

This directory is mounted into the isaac-sim container, and is the location for storing scripts,
libraries, and tooling that I use in this project.


### Pycharm Setup

To get good code completion, add a `Docker compose interpreter`, then
point it towards `/isaac-sim/python.sh`. That should
work, and pycharm should discover all the relevant libraries.

![img.png](docs/img.png)
# docker/utils

These scripts are intended to assist in creating and using the robot Docker
container. They are installed in `/usr/local/bin` for easy availability.

Scripts in `environment/` assist in installing and setting up dependencies
in the image. These are introduced into the image early on and can be used
anytime during the build process or afterward.

Scripts in `runtime/` are for starting tasks in the image after it has been
built. These are introduced into the image once everything else has been set
up.

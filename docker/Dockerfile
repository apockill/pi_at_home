ARG BASE_IMAGE="this_variable_is_set_in_the_.env_file
FROM ${BASE_IMAGE}

# Set configurable environment variables
ARG ROS2_DISTRO=jazzy
ARG POETRY_VERSION=1.8.4
ARG COLCON_POETRY_ROS_VERSION=0.9.0
ARG NODE_HELPERS_VERSION=v0.5.1

# Set variables for basic local and config, to make building easier
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHON_VERSION=python3.12
ENV PYTHONDONTWRITEBYTECODE=1

# Set variables to allow Docker to display windows on the host machine
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Set variables for caches
ENV APT_CACHE=/var/cache/apt
ENV PIP_CACHE=/root/.cache/pip
ENV BUILD_CACHE=/colcon-build-cache/pi_at_home/

# TODO: was opengl necessary?

# Copy in useful scripts for use during the build, such as `add-apt-repo` and `install-packages`
COPY docker/utils/environment/* /usr/local/bin/

# Add the ROS2 repo
RUN --mount=type=cache,target="${APT_CACHE}" \
    add-apt-repo \
    "deb [signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"

######################################################################
########## NOTE TO TEMPLATE USERS: This is a good place to add any additional apt packages you need
# Install ROS2 and other necessary packages
RUN --mount=type=cache,target="${APT_CACHE}" \
    install-packages \
    # ROS2
    ros-${ROS2_DISTRO}-ros-base \
    ros-${ROS2_DISTRO}-rosbridge-suite \
    ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
    # Build tools
    build-essential \
    git \
    git-lfs \
    # Complementary ROS2 Python tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    # Poetry dependencies \
    python3-pip \
    python-is-python3 \
    python3-venv
######################################################################

# Install colcon-poetry-ros, a tool by Urban Machine for building ROS2 packages with Poetry
RUN --mount=type=cache,target="${PIP_CACHE}" \
    pip3 install --break-system-packages "colcon-poetry-ros==${COLCON_POETRY_ROS_VERSION}"
RUN curl -fsSL https://install.python-poetry.org --output /tmp/install-poetry.py \
    && POETRY_HOME=/usr/local python3 /tmp/install-poetry.py --version "${POETRY_VERSION}"
RUN poetry self add poetry-plugin-bundle

# Add the ROS core setup script to the 'workspaces' command
RUN add-workspace /opt/ros/${ROS2_DISTRO}/setup.bash

# Set up colcon so it can install mixins. This is required for `colcon-poetry-ros`
RUN colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
        https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Initialize Rosdep for automatically downloading dependencies from `package.xml` in pkgs
WORKDIR /robot
RUN rosdep init
RUN rosdep update --rosdistro ${ROS2_DISTRO}

######################################################################
########## Add package.xml's of each package
########## NOTE TO TEMPLATE USERS: When adding a new package, add a new line here copying the package.xml
COPY pkgs/myarm_ai/package.xml pkgs/myarm_ai/package.xml
COPY pkgs/myarm_ai_msgs/package.xml pkgs/myarm_ai_msgs/package.xml
######################################################################

# Install rosdep dependencies for each package
RUN --mount=type=cache,target="${PIP_CACHE}" \
    --mount=type=cache,target="${APT_CACHE}" \
    with-package-list \
    rosdep install -i --from-path pkgs --rosdistro "${ROS2_DISTRO}" -y

######################################################################
########## Add package.xml's of each package
########## NOTE TO TEMPLATE USERS: When adding a new package, add a new line here copying the poetry.lock and pyproject.toml files
COPY pkgs/myarm_ai/poetry.lock pkgs/myarm_ai/poetry.lock
COPY pkgs/myarm_ai/pyproject.toml pkgs/myarm_ai/pyproject.toml
######################################################################

######################################################################
########## Add Git ROS2 Packages
########## NOTE TO TEMPLATE USERS: If you need to depend on a package that is not in the ROS2 distro, you can add it here
WORKDIR /ros-git-deps/
RUN --mount=type=cache,target="${PIP_CACHE}" \
    --mount=type=cache,target="${APT_CACHE}" \
    install-ros-package-from-git \
      https://github.com/UrbanMachine/node_helpers.git $NODE_HELPERS_VERSION pkgs && \
    ##################### Add your packages here!
    ########### install-ros-package-from-git {URL} {BRANCH} {PKGS PATH IN REPO}
    echo "Done installing ROS2 packages from git"
######################################################################

# Install Poetry dependencies for each package in this repo
WORKDIR /robot
RUN --mount=type=cache,target="${PIP_CACHE}" \
    python3 -m colcon_poetry_ros.dependencies.install --base-paths pkgs

## Add ROS2 libraries to Python's path ahead of enter-workspaces to help developer tools
## like PyCharm or VS Code understand where packages are. It also breaks dependency isolation a bit,
## but it's a necessary evil for now.
RUN echo "/opt/ros/${ROS2_DISTRO}/lib/${PYTHON_VERSION}/site-packages" >> /usr/local/lib/${PYTHON_VERSION}/dist-packages/ros2.pth
RUN echo "/opt/ros/${ROS2_DISTRO}/local/lib/${PYTHON_VERSION}/dist-packages" >> /usr/local/lib/${PYTHON_VERSION}/dist-packages/ros2.pth
RUN make-pth-file-from-workspace "$(pwd)/install" /usr/local/lib/${PYTHON_VERSION}/dist-packages/robot.pth

# Move the build cache from a Docker cache mount to a place where our build
# system can see it. This helps make `colcon build` faster between runs.
RUN --mount=type=cache,target="${BUILD_CACHE}" restore-build-cache

# Build all packages, copying project files under a root `/robot/` directory in the container
COPY ../launch-profiles /robot/launch-profiles
COPY ../pkgs /robot/pkgs
RUN enter-workspaces colcon build --packages-select $(ls pkgs)
RUN add-workspace /robot/install/setup.bash

# Move the build cache back into the Docker cache mount so it can be reused on
# future runs
RUN --mount=type=cache,target="${BUILD_CACHE}" save-build-cache

# Make ROS tools available by default
ENTRYPOINT [ "enter-workspaces" ]
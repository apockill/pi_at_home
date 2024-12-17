#!/usr/bin/env bash
# Shared functions used in multiple deployment scripts

source .env

# Builds our Docker images
function build_images {
  echo "Building project images. This may take a few minutes on the first run..." >&2
  docker compose build --build-arg BASE_IMAGE="${BASE_IMAGE}"
}

# Removes an existing Docker Compose deployment, if one exists
function undeploy {
  if is_stack_running; then
    echo "" >&2
    echo 'Waiting for robot stack to come down' >&2
    docker compose down
    while is_stack_running; do
      sleep 0.1
    done
    echo " done!" >&2
  fi
}

# Pulls necessary images for offline building and deployment
function pull_images {
  echo "Pulling necessary docker images. This may take a few minutes on the first run..." >&2

  # Pull all the non-locally-built images
  docker compose pull grafana loki promtail

  # Pull the base images for (for image building purposes)
  # Doing this explicitly tags the base
  # image, preventing `docker build` from checking for updates itself. That
  # way, if the user chooses to skip this step, builds can be done offline.
  docker pull "${BASE_IMAGE}"
}

# Enables display passthrough of windows. Other passthrough (such as pulseaudio) can also
# be configured in this function.
function enable_display_passthrough {
  xhost +local:root
}

# This is automatically called when the user presses Ctrl+C. It will bring down the stack.
function _catch_sigint {
  trap '' INT
  undeploy
}

# Checks if the stack is running
function is_stack_running() {
    [ -n "$(docker compose ps --services)" ]
}

# Starts a Docker Compose deployment, and waits until the user presses Ctrl+C.
# Then, the stack will be brought down.
function deploy_and_wait {
  # Start the stack non-blockingly, because logs are best accessed via the web interface
  export LAUNCH_PROFILE="${1}"
  docker compose up --detach

  trap _catch_sigint INT

  echo "" >&2
  echo "Deployment of '${PROJECT_NAME}' has started! Press Ctrl+C to stop." >&2
  echo "Logs are viewable at http://localhost/" >&2

  while is_stack_running; do
    sleep 1
  done
  echo "Another process brought down the stack." >&2
  return 1
}

# Shows the user the available launch profiles
function launch_profiles_helper_msg {
  echo "Available launch profiles are:" >&2
   # shellcheck disable=SC2012
  ls -1 launch-profiles/ | sed 's/^/      - /' >&2
  echo "" >&2
  echo "Read more about 'launch-profiles' under 'docs/about_template.md'" >&2
  exit 1
}

# Inform the user that the chosen launch profile is invalid if it is not a directory
function validate_launch_profile {
  local chosen_profile
  chosen_profile="$1"

  # Check if the chosen profile is a directory under 'launch-profiles'
  if [[ ! -d "launch-profiles/${chosen_profile}" ]]; then
    echo "Error: '${chosen_profile}' is not a valid launch profile." >&2
    echo "It should be a directory under 'launch-profiles/'." >&2
    launch_profiles_helper_msg
  fi
}
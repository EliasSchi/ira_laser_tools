#!/bin/bash
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh
DOCKER_DIR="${ROOT}/../docker"

function usage() {
    print_info "Usage: run_dev.sh"
    print_info "Copyright (c) 2024, enabl Technology."
}

# prefix for container name
BASE_NAME="scan-merge"
# path to the base of workspace
PACKAGE_DIR="$HOME/test_ws_elias"
# base of workspace in container
WORKSPACE_DIR="/colcon_ws"

ON_EXIT=()
function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command
    done
}
trap cleanup EXIT

pushd . >/dev/null
ON_EXIT+=("popd")

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

PLATFORM="$(uname -m)"
IMAGE_NAME="${BASE_NAME}-${PLATFORM}"
CONTAINER_NAME="${IMAGE_NAME}-container"

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    docker exec -i -t -u ros --workdir $WORKSPACE_DIR $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Build image
if [[ ! "$(docker images --quiet $CONTAINER_NAME)" ]]; then 
    print_info "No image with container name: $CONTAINER_NAME exists."
    print_info "Building container with name $CONTAINER_NAME for target platform $PLATFORM."

    docker build $DOCKER_DIR --platform $PLATFORM -t $CONTAINER_NAME
fi

print_info "Start container $CONTAINER_NAME."
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/FastDDSProfile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")

# Run container from image
docker run -it --rm \
    --privileged \
    --network host \
    --name "$CONTAINER_NAME" \
    ${DOCKER_ARGS[@]} \
    -v $PACKAGE_DIR:$WORKSPACE_DIR \
    -v /dev/shm:/dev/shm \
    --workdir $WORKSPACE_DIR \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    $CONTAINER_NAME \
    /bin/bash
    bash -c "ros2 launch ira_laser_tools laserscn_multi_merger_launch.py"

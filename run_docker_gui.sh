#!/bin/env bash
ROS_DISTRO="jazzy"
TEST=false
NO_CACHE=""

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros_distro)
      ROS_DISTRO="${2:-}"
      shift 2
      ;;
    --ros_distro=*)
      ROS_DISTRO="${1#*=}"
      shift 1
      ;;
    --test)
      TEST="true"
      shift 1
      ;;
    --no-cache)
      NO_CACHE="--no-cache"
      shift 1
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

docker build $NO_CACHE --build-arg RUN_TESTS=${TEST} --build-arg ROS_DISTRO=${ROS_DISTRO} -t "mujoco_ros2_control:${ROS_DISTRO}" .

docker network create ros
# give permissions to use X11 with docker 
xhost +local:docker
# starts the container with the franka example
docker run \
    --network="ros" \
    --gpus="all" \
    --device="/dev/dri:/dev/dri" \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.Xdocker \
    --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it mujoco_ros2_control:${ROS_DISTRO} bash
xhost -local:docker

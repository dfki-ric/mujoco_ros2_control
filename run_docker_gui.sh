#!/bin/env bash
if [[ $1 == "test" ]]; then
    docker build --build-arg RUN_TESTS=true -t "mujoco_ros2_control:jazzy-citest" .
else
    docker build -t "mujoco_ros2_control:jazzy" .
fi

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
    -it mujoco_ros2_control:jazzy bash
xhost -local:docker

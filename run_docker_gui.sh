#!/bin/env bash
# Build the container (pass "test" to also run the tests during the build)
if [[ $1 == "test" ]]; then
    docker build --build-arg ROS_DISTRO=humble --build-arg RUN_TESTS=true -t "mujoco_ros2_control:humble-citest" .
else
    docker build --build-arg ROS_DISTRO=humble -t "mujoco_ros2_control" .
fi
# create the network for the container (it prints a error when the network already exist)
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
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it mujoco_ros2_control bash
# remove the permissions to use X11 with docker 
xhost -local:docker

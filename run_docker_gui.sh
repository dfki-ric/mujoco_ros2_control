#!/bin/env bash
# Build the container
docker build -t "mujoco_ros2_control" .
# create the network for the container (it prints a error when the network already exist)
docker network create ros
# give permissions to use X11 with docker 
xhost +local:docker
# starts the container with the franka example
docker run \
    --network="ros" \
    --device="/dev/dri:/dev/dri" \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.Xdocker \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it mujoco_ros2_control bash
# remove the permissions to use X11 with docker 
xhost -local:docker

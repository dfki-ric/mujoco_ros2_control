#!/bin/env bash
docker build -t "mujoco_ros2_control" .

docker network create ros
xhost +local:docker
docker run \
    --network="ros" \
    --device="/dev/dri:/dev/dri" \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.Xdocker \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it mujoco_ros2_control bash
xhost -local:docker

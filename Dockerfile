FROM ros:humble AS base

RUN apt-get update && apt-get install -y \
    git \
    libglfw3-dev \
    libx11-dev \
    xorg-dev \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-pcl-ros \
    ros-humble-perception-pcl \
    ros-humble-urdfdom-py \
    libopencv-dev \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    libpcl-dev \
    python3-scipy

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN colcon build

RUN rm -rf /git

RUN echo source /ros2_ws/install/setup.bash > /root/.bashrc

COPY mujoco_ros2_control /ros2_ws/src/mujoco_ros2_control

WORKDIR /ros2_ws
#RUN rosdep init ||
#RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro humble -y
#RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-select mujoco_ros2_control_simulate_gui
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

FROM base AS demo
RUN apt-get update && apt-get install -y \
    ros-humble-franka-description
COPY examples /ros2_ws/src/mujoco_ros2_control_examples
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

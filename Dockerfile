FROM ros:jazzy AS base

RUN apt-get update && apt-get install -y \
    git \
    libglfw3-dev \
    libx11-dev \
    xorg-dev \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-pcl-ros \
    ros-jazzy-perception-pcl \
    ros-jazzy-urdfdom-py \
    libopencv-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge \
    libpcl-dev \
    python3-scipy

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN colcon build


RUN echo source /ros2_ws/install/setup.bash > /root/.bashrc

COPY mujoco_ros2_control /ros2_ws/src/mujoco_ros2_control

WORKDIR /ros2_ws
#RUN rosdep init ||
RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-select mujoco_ros2_control_simulate_gui
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

FROM base AS demo
# RUN apt-get update && apt-get install -y \
#     ros-jazzy-franka-description
COPY examples /ros2_ws/src/mujoco_ros2_control_examples
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-skip franka_mujoco
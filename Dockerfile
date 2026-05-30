FROM ros:humble AS base

RUN apt-get update && apt-get install -y \
    git \
    libglfw3-dev \
    libx11-dev \
    xorg-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    extra-cmake-modules \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-urdfdom-py \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    libopencv-dev \
    libpcl-dev \
    python3-scipy

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN colcon build


RUN echo source /ros2_ws/install/setup.bash > /root/.bashrc

COPY mujoco_ros2_control /ros2_ws/src/mujoco_ros2_control

WORKDIR /ros2_ws
#RUN rosdep init ||
RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

FROM base AS demo
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-franka-description
COPY examples /ros2_ws/src/mujoco_ros2_control_examples
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

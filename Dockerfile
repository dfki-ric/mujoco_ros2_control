FROM ros:jazzy AS base

RUN apt-get update && apt-get install -y \
    git \
    libx11-dev \
    xorg-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    extra-cmake-modules

WORKDIR /ros2_ws
COPY mujoco_ros2_control /ros2_ws/src/mujoco_ros2_control

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-select mujoco_ros2_control

# Optionally run the library tests at build time
ARG RUN_TESTS=false
RUN if [ "$RUN_TESTS" = "true" ]; then \
      . /opt/ros/$ROS_DISTRO/setup.sh && . install/setup.sh && \
      DISABLE_OPENGL=1 colcon test --packages-select mujoco_ros2_control \
        --event-handlers console_direct+ && \
      colcon test-result --test-result-base build/mujoco_ros2_control/test_results --verbose; \
    fi

RUN echo "source /ros2_ws/install/setup.bash" > /root/.bashrc


FROM base AS demo

# franka_description has no binary package on jazzy, so rosdep cannot resolve
# it -> build it from source.
RUN git clone -b jazzy https://github.com/frankarobotics/franka_description.git \
    /ros2_ws/src/franka_description
COPY mujoco_ros2_control_examples /ros2_ws/src/mujoco_ros2_control_examples

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-up-to mujoco_ros2_control_examples

ARG RUN_TESTS=false
RUN if [ "$RUN_TESTS" = "true" ]; then \
      . /opt/ros/$ROS_DISTRO/setup.sh && . install/setup.sh && \
      DISABLE_OPENGL=1 colcon test --packages-select mujoco_ros2_control_examples \
        --event-handlers console_direct+ && \
      colcon test-result --test-result-base build/mujoco_ros2_control_examples/test_results --verbose; \
    fi
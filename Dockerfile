ARG ROS_DISTRO="humble"

FROM ros:${ROS_DISTRO} AS base

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
COPY mujoco_ros2_control_examples /ros2_ws/src/mujoco_ros2_control_examples
# franka_description and ur_description are declared in the examples package.xml
# and available as binaries on humble, so rosdep resolves them (no git clone).
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

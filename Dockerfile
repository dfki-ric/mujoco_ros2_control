ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS base

# Build-time deps rosdep cannot resolve: the MuJoCo simulate GUI build headers
# (X11/Wayland/cmake-modules) plus the EGL headers the offscreen camera and
# lidar sensors compile against.
RUN apt-get update && apt-get install -y \
    git \
    libegl-dev \
    libx11-dev \
    xorg-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    extra-cmake-modules

# Mesa software rendering, so the camera and lidar sensors can get an EGL
# context with no GPU and no display. Mirrors .github/workflows/{humble,jazzy}-core.yml; without
# it their tests can only be skipped. A container with a real GPU passed through
# uses that instead - these are the fallback, not a restriction.
RUN apt-get install -y \
    libegl1 \
    libglvnd0 \
    libopengl0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    mesa-utils-extra

# The test stages below run with OpenGL on, so a GL test reporting "skipped"
# means the image quietly lost EGL and the run proved less than it claims.
# Same guard as the CI workflows.
RUN printf '%s\n' \
    'check_gl_ran() {' \
    '  if grep -rl "OpenGL disabled" "$1" 2>/dev/null | grep -q .; then' \
    '    echo "ERROR: GL tests were skipped, but this image tests with OpenGL enabled" >&2' \
    '    grep -rn "OpenGL disabled" "$1" >&2 || true' \
    '    return 1' \
    '  fi' \
    '  echo "No GL test was skipped."' \
    '}' \
    > /usr/local/bin/check_gl_ran.sh

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
      . /usr/local/bin/check_gl_ran.sh && \
      DISABLE_OPENGL=0 EGL_PLATFORM=surfaceless \
        colcon test --packages-select mujoco_ros2_control \
        --event-handlers console_direct+ && \
      colcon test-result --test-result-base build/mujoco_ros2_control/test_results --verbose && \
      check_gl_ran build/mujoco_ros2_control/test_results; \
    fi

RUN echo "source /ros2_ws/install/setup.bash" > /root/.bashrc


FROM base AS demo

COPY mujoco_ros2_control_examples /ros2_ws/src/mujoco_ros2_control_examples

RUN vcs import --input src/mujoco_ros2_control_examples/.repos-${ROS_DISTRO} src/ \
    && mv src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg src/unitree_hg \
    && rm -rf src/unitree_ros2


RUN sed -i '/<build_depend>rosidl_default_generators<\/build_depend>/a\  <build_depend>rosidl_generator_dds_idl</build_depend>' \
    src/unitree_hg/package.xml

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
RUN PY_VER=$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])') && \
    uv venv /ros2_ws/.venv --python "$PY_VER" && \
    uv pip install --python /ros2_ws/.venv/bin/python "numpy<2" coacd trimesh && \
    ln -s "python${PY_VER}" /ros2_ws/.venv/lib/python-site
ENV PYTHONPATH="/ros2_ws/.venv/lib/python-site/site-packages:${PYTHONPATH}"

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-up-to mujoco_ros2_control_examples

ARG RUN_TESTS=false
RUN if [ "$RUN_TESTS" = "true" ]; then \
      . /opt/ros/$ROS_DISTRO/setup.sh && . install/setup.sh && \
      . /usr/local/bin/check_gl_ran.sh && \
      DISABLE_OPENGL=0 EGL_PLATFORM=surfaceless \
        colcon test --packages-select mujoco_ros2_control_examples \
        --event-handlers console_direct+ && \
      colcon test-result --test-result-base build/mujoco_ros2_control_examples/test_results --verbose && \
      check_gl_ran build/mujoco_ros2_control_examples/test_results; \
    fi
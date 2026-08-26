FROM ros:jazzy AS base

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
# context with no GPU and no display. Mirrors .github/workflows/ci.yml; without
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

RUN vcs import --input src/mujoco_ros2_control_examples/.repos src/ \
    && mv src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg src/unitree_hg \
    && rm -rf src/unitree_ros2

# coacd/trimesh, for the Franka example's decompose_industreal_peg_trays. They
# have no apt/rosdep package, and this image's system Python is PEP 668
# externally-managed, so `pip install --system --break-system-packages` is the
# obvious move -- and the wrong one: coacd/trimesh pull in an unpinned numpy
# that would shadow the apt-built python3-numpy for every process in the image,
# and trimesh unconditionally imports the apt-built python3-scipy (compiled
# against numpy 1.x's ABI) at import time, crashing the moment numpy floats to
# 2.x. A venv bridged onto PYTHONPATH keeps these out of the files apt manages;
# ${PYTHONPATH} still has to carry it, because run_coacd.py is launched as
# `sys.executable` from whichever Python is already running -- the ROS-sourced
# system one, not a venv's.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
RUN uv venv /ros2_ws/.venv --python 3.12 && \
    uv pip install --python /ros2_ws/.venv/bin/python "numpy<2" coacd trimesh
ENV PYTHONPATH="/ros2_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH}"

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
# syntax=docker/dockerfile:1
ARG ROS_BASE=ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
FROM ghcr.io/astral-sh/uv:0.12.17@sha256:10787c682e4184e4f290de1171fd4703dc63de99221f10fe1c99002ce7fa9acc AS uv
FROM ${ROS_BASE} AS base_inputs
FROM base_inputs AS common_build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
COPY docker/preserve-apt.sh /usr/local/bin/preserve-apt
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-downloads
COPY locks/geographiclib.sha256 /opt/solo/geographiclib.sha256
# APT provides build/runtime dependencies; the pinned source overlay below is MAVROS 2.15.1.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-eigen3-cmake-module \
      ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rosbag2-storage-mcap \
      python3-yaml python3-venv python3-pytest python3-colcon-common-extensions \
      geographiclib-tools curl ca-certificates git iproute2 procps tmux \
      build-essential cmake pkg-config python3-dev libgmp-dev \
      libasio-dev libgeographiclib-dev libconsole-bridge-dev libeigen3-dev && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    sha256sum -c /opt/solo/geographiclib.sha256 && \
    preserve-apt
# Binary packages omit build-only dependencies (e.g. angles); use Debian source metadata.
RUN apt-get update && apt-get build-dep -y \
      ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs ros-jazzy-libmavconn && \
    preserve-apt
ARG MAVROS_COMMIT=22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e
RUN git clone --branch 2.15.1 --depth 1 https://github.com/mavlink/mavros.git /opt/mavros_ws/src/mavros && \
    test "$(git -C /opt/mavros_ws/src/mavros rev-parse HEAD)" = "$MAVROS_COMMIT" && \
    source /opt/ros/jazzy/setup.bash && cd /opt/mavros_ws && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]

# Keep application changes after expensive dependency/compiler layers.
FROM common_build AS application_build
COPY ros_ws/src /opt/solo_ws/src
RUN source /opt/ros/jazzy/setup.bash && source /opt/mavros_ws/install/setup.bash && cd /opt/solo_ws && \
    colcon build --event-handlers console_direct+
ARG SOURCE_REVISION=unknown
RUN printf '%s\n' "$SOURCE_REVISION" > /opt/solo/build-inputs/source-revision
# Keep this workspace and its build inputs in a separate archival target.
COPY config /workspace/config
COPY scripts /workspace/scripts
COPY compose.release.yaml .env.release.example scripts/release scripts/doctor.py /opt/solo/deploy/
COPY locks /opt/solo/locks
COPY locks/versions.json /opt/solo/versions.json
COPY docker/entrypoint.sh /usr/local/bin/solo-entrypoint

FROM application_build AS companion_inputs
COPY . /opt/solo/source-checkout

FROM common_build AS simulation_dependencies
ARG ARDUPILOT_COMMIT=ee14ff3822161368ec403700bf96d02b84def06c
ARG GAZEBO_PLUGIN_COMMIT=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
ARG BUILD_JOBS=4
COPY --from=uv /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/python UV_CACHE_DIR=/opt/solo/build-inputs/uv-cache
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      libegl1-mesa-dev libgl1-mesa-dri mesa-utils gawk ccache \
      libtool libtool-bin libxml2-dev libxslt1-dev && \
    preserve-apt
COPY locks/ardupilot-py311.lock /opt/solo/ardupilot-py311.lock
COPY locks/build-backend.lock /opt/solo/build-backend.lock
RUN uv python install 3.11.16 && \
    uv venv --python 3.11.16 /opt/venvs/ardupilot && \
    uv pip install --python /opt/venvs/ardupilot/bin/python --require-hashes \
      -r /opt/solo/build-backend.lock && \
    uv pip install --python /opt/venvs/ardupilot/bin/python --require-hashes \
      --no-build-isolation -r /opt/solo/ardupilot-py311.lock
RUN git clone --branch Copter-4.5.3 --depth 1 https://github.com/ArduPilot/ardupilot.git /opt/ardupilot && \
    test "$(git -C /opt/ardupilot rev-parse HEAD)" = "$ARDUPILOT_COMMIT" && \
    git -C /opt/ardupilot submodule update --init --recursive --depth 1 && \
    cd /opt/ardupilot && /opt/venvs/ardupilot/bin/python ./waf configure --board sitl && \
    /opt/venvs/ardupilot/bin/python ./waf copter -j"$BUILD_JOBS"
# gz-transport's exported target needs zmq.hpp; it is only an APT recommendation.
RUN apt-get -o Acquire::Retries=3 update --error-on=any && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends cppzmq-dev && \
    preserve-apt && \
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo && \
    git -C /opt/ardupilot_gazebo checkout "$GAZEBO_PLUGIN_COMMIT" && \
    source /opt/ros/jazzy/setup.bash && export GZ_VERSION=harmonic && \
    cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3 && \
    cmake --build /opt/ardupilot_gazebo/build -j"$BUILD_JOBS"
# Gazebo's Qt GUI uses SVG icons; --no-install-recommends does not pull this in.
RUN apt-get -o Acquire::Retries=3 update --error-on=any && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends libqt5svg5 && \
    preserve-apt
RUN dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock && \
    git -C /opt/ardupilot submodule status --recursive > /opt/solo/submodules.lock && \
    sha256sum /usr/share/GeographicLib/geoids/egm96-5.pgm > /opt/solo/geoid.sha256 && \
    test -x /opt/ardupilot/build/sitl/bin/arducopter && \
    test -f /opt/ardupilot_gazebo/build/libArduPilotPlugin.so && \
    mkdir -p /opt/solo/licenses && \
    find /opt/ardupilot -type f \
      \( -iname 'license*' -o -iname 'copying*' -o -iname 'notice*' \) \
      -exec cp --parents -t /opt/solo/licenses {} +
RUN printf 'BUILD_JOBS=%s\nARDUPILOT_COMMIT=%s\nGAZEBO_PLUGIN_COMMIT=%s\n' \
    "$BUILD_JOBS" "$ARDUPILOT_COMMIT" "$GAZEBO_PLUGIN_COMMIT" > /opt/solo/build-inputs/simulation-build-args
FROM simulation_dependencies AS simulation_mpc_dependencies
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-bridge python3-opencv && preserve-apt
COPY locks/mpc-requirements.txt /opt/solo/mpc-requirements.txt
COPY locks/mpc-py312.lock /opt/solo/mpc-py312.lock
RUN python3 -m venv --system-site-packages /opt/venvs/mpc && \
    uv pip install --python /opt/venvs/mpc/bin/python --require-hashes -r /opt/solo/mpc-py312.lock
ENV PYTHONPATH=/opt/venvs/mpc/lib/python3.12/site-packages \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1
# Source, submodules, Python downloads, APT debs/indexes, datasets and build flags
# are retained here for archival, never copied wholesale into runtime images.
FROM simulation_mpc_dependencies AS simulation_inputs
COPY --from=application_build /opt/solo/build-inputs/source-revision /opt/solo/build-inputs/source-revision
COPY --from=application_build /opt/solo_ws /opt/solo_ws
COPY --from=application_build /workspace /workspace
COPY --from=application_build /opt/solo/locks /opt/solo/locks
COPY --from=application_build /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application_build /opt/solo/deploy /opt/solo/deploy
COPY --from=application_build /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
COPY . /opt/solo/source-checkout

FROM base_inputs AS runtime_common
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive HOME=/tmp/solo-home \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
# Install from the exact debs used by the compiler stage, with no live APT access.
COPY docker/install-runtime.sh /usr/local/bin/install-runtime
RUN --network=none --mount=from=common_build,source=/opt/solo/build-inputs/apt,target=/inputs,ro \
    install-runtime ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge ros-jazzy-image-transport \
      ros-jazzy-rosbag2-storage-mcap python3-yaml python3-pytest iproute2 procps
COPY --from=common_build /opt/mavros_ws/install /opt/mavros_ws/install
COPY --from=common_build /opt/mavros_ws/src/mavros/LICENSE* /opt/solo/licenses/mavros/
COPY --from=common_build /usr/share/GeographicLib /usr/share/GeographicLib
COPY --from=application_build /opt/solo_ws/install /opt/solo_ws/install
COPY --from=application_build /workspace /workspace
COPY --from=application_build /opt/solo/locks /opt/solo/locks
COPY --from=application_build /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application_build /opt/solo/deploy /opt/solo/deploy
COPY --from=application_build /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
COPY docker/prune-runtime.sh /usr/local/bin/prune-runtime
RUN mkdir -p /workspace/runs /workspace/artifacts && \
    chmod 755 /usr/local/bin/solo-entrypoint && \
    cp /opt/solo/locks/geographiclib.sha256 /opt/solo/geoid.sha256

FROM runtime_common AS companion_root
RUN prune-runtime

# Flatten the prepared root filesystem so removed toolchains occupy no layers.
FROM scratch AS companion
COPY --from=companion_root / /
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
    ROS_DISTRO=jazzy PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]
CMD ["ros2", "launch", "solo_bringup", "hardware.launch.py"]

FROM runtime_common AS simulation_root
RUN --network=none --mount=from=simulation_mpc_dependencies,source=/opt/solo/build-inputs/apt,target=/inputs,ro \
    install-runtime gz-harmonic ros-jazzy-ros-gz-bridge python3-opencv \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      libgl1-mesa-dri libegl1 mesa-utils libqt5svg5
COPY --from=simulation_dependencies /opt/ardupilot/build/sitl/bin/arducopter /opt/ardupilot/build/sitl/bin/arducopter
COPY --from=simulation_dependencies /opt/ardupilot/Tools/autotest/default_params /opt/ardupilot/Tools/autotest/default_params
COPY --from=simulation_dependencies /opt/ardupilot/COPYING.txt /opt/ardupilot/COPYING.txt
COPY --from=simulation_dependencies /opt/ardupilot_gazebo/build/*.so /opt/ardupilot_gazebo/build/
COPY --from=simulation_dependencies /opt/ardupilot_gazebo/models /opt/ardupilot_gazebo/models
COPY --from=simulation_dependencies /opt/ardupilot_gazebo/worlds /opt/ardupilot_gazebo/worlds
COPY --from=simulation_dependencies /opt/ardupilot_gazebo/config /opt/ardupilot_gazebo/config
COPY --from=simulation_dependencies /opt/ardupilot_gazebo/LICENSE.md /opt/ardupilot_gazebo/LICENSE.md
COPY --from=simulation_dependencies /opt/solo/submodules.lock /opt/solo/submodules.lock
COPY --from=simulation_dependencies /opt/solo/licenses /opt/solo/licenses
COPY --from=simulation_mpc_dependencies /opt/venvs/mpc /opt/venvs/mpc
RUN strip --strip-unneeded /opt/ardupilot/build/sitl/bin/arducopter \
      /opt/ardupilot_gazebo/build/*.so && prune-runtime

FROM scratch AS simulation
COPY --from=simulation_root / /
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
    ROS_DISTRO=jazzy PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    PYTHONPATH=/opt/venvs/mpc/lib/python3.12/site-packages \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/numba-cache
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]
CMD ["ros2", "launch", "solo_bringup", "simulation.launch.py"]

FROM simulation AS simulation_mpc
CMD ["ros2", "launch", "solo_mpc", "mpc.launch.py"]

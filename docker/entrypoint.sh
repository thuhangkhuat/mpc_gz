#!/usr/bin/env bash
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/mavros_ws/install/setup.bash
# shellcheck disable=SC1091
source /opt/solo_ws/install/setup.bash
# Only the development Compose service opts in to checkout imports.
if [[ "${SOLO_DEV:-0}" == 1 ]]; then
  export PYTHONPATH="/workspace/ros_ws/src/solo_mpc:/workspace/ros_ws/src/solo_bringup:${PYTHONPATH:-}"
fi
mkdir -p "${HOME}" /tmp/solo-runtime
chmod 700 /tmp/solo-runtime
export XDG_RUNTIME_DIR=/tmp/solo-runtime
if [[ -d /opt/ardupilot_gazebo ]]; then
  export GZ_VERSION=harmonic
  # ros_gz vendor hooks replace this path with bridge-only CLI definitions.
  # Retain the native Harmonic commands (including gz sim) alongside them.
  export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
  export SDF_PATH="/opt/ardupilot_gazebo/models:${SDF_PATH:-}"
fi
exec "$@"

#!/usr/bin/env bash
set -eo pipefail

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_distro_name="${ROS_DISTRO_NAME:-humble}"
ros_setup="/opt/ros/${ros_distro_name}/setup.bash"

if [[ ! -r "${ros_setup}" ]]; then
  echo "Error: ROS 2 setup file does not exist: ${ros_setup}" >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "Error: colcon is not installed." >&2
  exit 1
fi

# Do not inherit another ROS workspace. The only underlay is /opt/ros; every
# non-system package used by this application is bundled below src/.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH CPATH CPLUS_INCLUDE_PATH LIBRARY_PATH
# shellcheck disable=SC1090
source "${ros_setup}"

cd "${kit_root}"
colcon build \
  --base-paths "${kit_root}/src" \
  --symlink-install \
  --executor sequential \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DFPSDK_BUILD_TESTING=OFF \
    -DFPSDK_USE_PROJ=OFF \
    -DCMAKE_BUILD_TYPE=Release \
  "$@"

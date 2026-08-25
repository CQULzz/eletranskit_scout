#!/usr/bin/env bash
set -eo pipefail

KIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"

source_required() {
  local setup_file="$1"
  local component="$2"

  if [[ ! -r "$setup_file" ]]; then
    echo "Error: ${component} setup file does not exist: ${setup_file}" >&2
    return 1
  fi

  # shellcheck disable=SC1090
  source "$setup_file"
}

# Drop ROS package-search state inherited from another sourced workspace. The
# only underlay allowed here is the system ROS installation.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH CPATH CPLUS_INCLUDE_PATH LIBRARY_PATH
source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" "ROS 2"

required_installs=(
  "${KIT_ROOT}/install/ele_scout_kit/share/ele_scout_kit/package.xml"
  "${KIT_ROOT}/install/fpsdk_common/lib/libfpsdk_common.so"
  "${KIT_ROOT}/install/fixposition_driver_lib/lib/libfixposition_driver_lib.so"
  "${KIT_ROOT}/install/fixposition_driver_ros2/share/fixposition_driver_ros2/package.xml"
  "${KIT_ROOT}/install/rslidar_msg/share/rslidar_msg/package.xml"
  "${KIT_ROOT}/install/rslidar_sdk/share/rslidar_sdk/package.xml"
  "${KIT_ROOT}/install/elevation_map_msgs/share/elevation_map_msgs/package.xml"
  "${KIT_ROOT}/install/ros2_numpy/share/ros2_numpy/package.xml"
  "${KIT_ROOT}/install/elevation_mapping_cupy/share/elevation_mapping_cupy/package.xml"
)
for required_install in "${required_installs[@]}"; do
  if [[ ! -r "${required_install}" ]]; then
    echo "首次运行：正在从本仓库 src/ 构建全部程序……"
    "${KIT_ROOT}/build_workspace.sh"
    break
  fi
done

source_required "${KIT_ROOT}/install/local_setup.bash" "standalone ele_scout_kit workspace"

for package_name in fixposition_driver_ros2 rslidar_msg rslidar_sdk elevation_mapping_cupy; do
  package_prefix="$(ros2 pkg prefix "${package_name}")"
  if [[ "${package_prefix}" != "${KIT_ROOT}/install/"* ]]; then
    echo "Error: ${package_name} resolved outside this repository: ${package_prefix}" >&2
    exit 1
  fi
done

if ! python3 - <<'PY'
import importlib

required = ("cupy", "cv2", "numpy", "scipy", "shapely", "ruamel.yaml", "simple_parsing")
missing = []
for module in required:
    try:
        importlib.import_module(module)
    except Exception as exc:  # Report broken binary installs as well as missing modules.
        missing.append(f"{module} ({exc})")

if missing:
    raise SystemExit("Missing Python runtime dependencies: " + ", ".join(missing))
PY
then
  echo "Error: install requirements-runtime.txt and a CUDA-compatible CuPy build; see README.md." >&2
  exit 1
fi

# ROS environment hooks are not nounset-safe; enable it only after sourcing.
set -u

exec ros2 launch ele_scout_kit real_car.launch.py "$@"

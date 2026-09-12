#!/usr/bin/env bash
# Humble port of FG-static/small_point_lio_lc/quick_source.sh
# Must be sourced, not executed:
#   source /home/nav/nav2_test/quick_source.sh
#
# Upstream script sourced:
#   /opt/ros/jazzy/setup.bash
#   /home/goose/fastlio/livox_ws/install/setup.bash
#   <that repo>/install/setup.bash
# This machine is Humble; the Livox driver lives in a sibling workspace so
# livox_ros_driver2/build.sh cannot wipe nav2_test's install/.
# Do not source bievr_ws here: its Ceres 2.2 overlay conflicts with Nav2.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Please run: source $0"
  exit 1
fi

_NAV2_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_ROS_SETUP="/opt/ros/humble/setup.bash"
_LIVOX_WS="${LIVOX_WS:-/home/nav/livox_ws}"
_LIVOX_SETUP="${_LIVOX_WS}/install/setup.bash"
_LOCAL_SETUP="${_NAV2_WS}/install/setup.bash"

# Humble's rclpy is Python 3.10. Drop conda so `python3` is not Anaconda 3.13.
if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda deactivate >/dev/null 2>&1 || true
fi
_stripped_path=""
_old_ifs="$IFS"
IFS=':'
for _p in $PATH; do
  case "$_p" in
    *miniconda*|*anaconda*|*conda/envs*) ;;
    *) _stripped_path="${_stripped_path:+$_stripped_path:}$_p" ;;
  esac
done
IFS="$_old_ifs"
export PATH="$_stripped_path"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
unset _stripped_path _old_ifs _p

if [[ ! -f "${_ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${_ROS_SETUP}"
  unset _NAV2_WS _ROS_SETUP _LIVOX_WS _LIVOX_SETUP _LOCAL_SETUP
  return 1
fi

if [[ ! -f "${_LIVOX_SETUP}" ]]; then
  echo "Missing Livox workspace setup: ${_LIVOX_SETUP}"
  echo "Build first:"
  echo "  mkdir -p ${_LIVOX_WS}/src"
  echo "  git clone https://github.com/Livox-SDK/livox_ros_driver2.git ${_LIVOX_WS}/src/livox_ros_driver2"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  ${_LIVOX_WS}/src/livox_ros_driver2/build.sh humble"
  unset _NAV2_WS _ROS_SETUP _LIVOX_WS _LIVOX_SETUP _LOCAL_SETUP
  return 1
fi

if [[ ! -f "${_LOCAL_SETUP}" ]]; then
  echo "Missing local workspace setup: ${_LOCAL_SETUP}"
  echo "Build first: cd ${_NAV2_WS} && colcon build --symlink-install"
  unset _NAV2_WS _ROS_SETUP _LIVOX_WS _LIVOX_SETUP _LOCAL_SETUP
  return 1
fi

source "${_ROS_SETUP}"
source "${_LIVOX_SETUP}"
source "${_LOCAL_SETUP}"

echo "Sourced:"
echo "  ${_ROS_SETUP}"
echo "  ${_LIVOX_SETUP}"
echo "  ${_LOCAL_SETUP}"
echo "  python3=$(command -v python3) ($("$(command -v python3)" --version 2>/dev/null))"
unset _NAV2_WS _ROS_SETUP _LIVOX_WS _LIVOX_SETUP _LOCAL_SETUP

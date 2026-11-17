#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
overlay_setup="${ENGINEAI_ROS2_OVERLAY:-${repo_root}/build/x86_64/ros2_env/install/setup.bash}"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 setup not found: ${ros_setup}" >&2
  exit 2
fi

# ROS 2 setup scripts use several optional variables without defaults. Load
# them outside nounset, then restore the recorder's strict-shell mode.
set +u
source "${ros_setup}"
if [[ -f "${overlay_setup}" ]]; then
  source "${overlay_setup}"
else
  echo "SDK ROS2 overlay not found: ${overlay_setup}" >&2
  echo "Build the interface overlay or set ENGINEAI_ROS2_OVERLAY to its setup.bash." >&2
  exit 2
fi
set -u

output_args=()
if [[ $# -gt 0 && "$1" != -* ]]; then
  output_args=(--output "$1")
  shift
fi

exec python3 "${repo_root}/tools/safety_monitor/record_safety.py" "${output_args[@]}" "$@"

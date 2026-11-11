#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Load ROS in this launcher only, just as the duel entrypoint does.
for single_ros_distro in "${ROS_DISTRO:-humble}" humble jazzy; do
  single_ros_setup="/opt/ros/${single_ros_distro}/setup.bash"
  if [[ -f "$single_ros_setup" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$single_ros_setup"
    set -u
    break
  fi
done

cd "$script_dir/.."
exec python3 -B -m tools.duel_sim.single_launcher "$@"

#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Match the SDK runtime environment, including ROS's transitive libraries.
for duel_ros_distro in "${ROS_DISTRO:-humble}" humble jazzy; do
  duel_ros_setup="/opt/ros/${duel_ros_distro}/setup.bash"
  if [[ -f "$duel_ros_setup" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$duel_ros_setup"
    set -u
    break
  fi
done

exec python3 "$script_dir/../tools/duel_sim/launcher.py" "$@"

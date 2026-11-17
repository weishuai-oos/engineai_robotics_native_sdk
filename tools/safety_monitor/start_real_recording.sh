#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
container_name="${ENGINEAI_RECORDING_CONTAINER:-engineai_robotics_env}"
container_user="${ENGINEAI_RECORDING_CONTAINER_USER:-ds}"
ros_domain="${ROS_DOMAIN_ID:-69}"
rmw_implementation="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
duration=""
run_name=""
skip_preflight=false
recorder_extra_args=()

usage() {
  cat <<EOF
Usage:
  $0 [run_name] [options]

Start the read-only real-robot recorder from the host or the SDK container.

Options:
  --duration SEC       Stop automatically after SEC seconds.
  --ros-domain ID      ROS 2 domain, default ${ros_domain}.
  --rmw NAME           RMW implementation, default ${rmw_implementation}.
  --skip-preflight     Skip the ROS 2 topic check.
  --                  Pass remaining arguments to record_safety.py.
  -h, --help           Show this help.

The default run directory is records/safety_monitor/real_t800_<timestamp>.
The recorder uses the real-robot LCM configuration and keeps the controller
untouched.
EOF
}

if [[ $# -gt 0 && "$1" != -* ]]; then
  run_name="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    --ros-domain)
      [[ $# -ge 2 ]] || { echo "--ros-domain requires a value" >&2; exit 2; }
      ros_domain="$2"
      shift 2
      ;;
    --rmw)
      [[ $# -ge 2 ]] || { echo "--rmw requires a value" >&2; exit 2; }
      rmw_implementation="$2"
      shift 2
      ;;
    --skip-preflight)
      skip_preflight=true
      shift
      ;;
    --)
      shift
      recorder_extra_args=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Use --help for usage." >&2
      exit 2
      ;;
  esac
done

if [[ -z "$run_name" ]]; then
  run_name="real_t800_$(date +%Y%m%d_%H%M%S)"
fi
if [[ ! "$run_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Run name may contain only letters, numbers, '.', '_' and '-'." >&2
  exit 2
fi
if [[ -n "$duration" && ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Duration must be a positive number of seconds." >&2
  exit 2
fi

records_root="${repo_root}/records/safety_monitor"
output_path="${records_root}/${run_name}"
if [[ -e "$output_path" ]]; then
  echo "Output directory already exists: ${output_path}" >&2
  exit 2
fi

in_container=false
if [[ -f /.dockerenv ]]; then
  in_container=true
fi

if [[ "$in_container" != true ]]; then
  command -v docker >/dev/null 2>&1 || {
    echo "Docker is required. Run this command inside engineai_robotics_env or install Docker." >&2
    exit 2
  }

  compose_dir="${repo_root}/docker/dockers/engineai_robotics_env"
  container_running="$(docker inspect -f '{{.State.Running}}' "$container_name" 2>/dev/null || true)"
  if [[ "$container_running" != true ]]; then
    if [[ ! -f "${compose_dir}/docker-compose.yml" ]]; then
      echo "Generated Docker compose files are missing: ${compose_dir}" >&2
      echo "Run ./docker/generate.sh first." >&2
      exit 2
    fi
    echo "Starting ${container_name}..."
    (cd "$compose_dir" && docker compose up -d)
  fi
fi

overlay_setup="${ENGINEAI_ROS2_OVERLAY:-${repo_root}/build/x86_64/ros2_env/install/setup.bash}"
if [[ ! -f "$overlay_setup" ]]; then
  echo "SDK ROS2 overlay not found: ${overlay_setup}" >&2
  echo "Build the interface overlay or set ENGINEAI_ROS2_OVERLAY." >&2
  exit 2
fi

run_ros2_topic_list() {
  local ros_distro="${ROS_DISTRO:-humble}"
  if [[ "$in_container" == true ]]; then
    ROS_DOMAIN_ID="$ros_domain" \
      ROS_LOCALHOST_ONLY=0 \
      RMW_IMPLEMENTATION="$rmw_implementation" \
      bash -c "set +u; source /opt/ros/${ros_distro}/setup.bash; source '${overlay_setup}'; set -u; export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; ros2 topic list --no-daemon"
  else
    docker exec --user "$container_user" \
      -e "ROS_DOMAIN_ID=${ros_domain}" \
      -e ROS_LOCALHOST_ONLY=0 \
      -e "RMW_IMPLEMENTATION=${rmw_implementation}" \
      "$container_name" \
      bash -lc "set +u; source /opt/ros/${ros_distro}/setup.bash; source '${overlay_setup}'; set -u; export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; ros2 topic list --no-daemon"
  fi
}

if [[ "$skip_preflight" != true ]]; then
  echo "Checking ROS 2 topics with ROS_DOMAIN_ID=${ros_domain}, RMW=${rmw_implementation}..."
  topic_list="$(run_ros2_topic_list 2>&1 || true)"
  if ! grep -Fxq "/hardware/joint_state" <<<"$topic_list"; then
    echo "No /hardware/joint_state publisher was discovered." >&2
    echo "Check the robot process, ROS_DOMAIN_ID, network interface and RMW implementation." >&2
    exit 3
  fi
  if grep -Fxq "/hardware/rc02_diagnostics" <<<"$topic_list"; then
    echo "Found /hardware/joint_state and /hardware/rc02_diagnostics."
  else
    echo "Found /hardware/joint_state, but /hardware/rc02_diagnostics is unavailable." >&2
    echo "Recording will continue without RC02 raw-frame diagnostics." >&2
  fi
fi

record_args=(
  "$run_name"
  --config assets/config/t800/mode.yaml
  --lcm-config assets/config/t800/lcm/robot.yaml
)
if [[ -n "$duration" ]]; then
  record_args+=(--duration "$duration")
fi
record_args+=("${recorder_extra_args[@]}")

echo "Recording to ${output_path}"
echo "Press Ctrl-C to stop recording and flush metadata.json."

if [[ "$in_container" == true ]]; then
  exec env \
    ROS_DOMAIN_ID="$ros_domain" \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION="$rmw_implementation" \
    "$repo_root/tools/safety_monitor/start_recording.sh" "${record_args[@]}"
else
  docker_exec_io=(-i)
  if [[ -t 0 && -t 1 ]]; then
    docker_exec_io=(-it)
  fi
  exec docker exec "${docker_exec_io[@]}" --user "$container_user" \
    -e "ROS_DOMAIN_ID=${ros_domain}" \
    -e ROS_LOCALHOST_ONLY=0 \
    -e "RMW_IMPLEMENTATION=${rmw_implementation}" \
    -w "$repo_root" \
    "$container_name" \
    "$repo_root/tools/safety_monitor/start_recording.sh" "${record_args[@]}"
fi

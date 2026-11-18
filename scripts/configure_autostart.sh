#!/bin/bash

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    exec sudo -- "$0" "$@"
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [ -f "$script_dir/systemd/engineai-native-sdk.service" ]; then
    project_dir="$script_dir"
elif [ -f "$script_dir/../systemd/engineai-native-sdk.service" ]; then
    project_dir=$(cd -- "$script_dir/.." && pwd)
else
    echo "Cannot locate the SDK project directory from $script_dir" >&2
    exit 1
fi

unit_source="$project_dir/systemd/engineai-native-sdk.service"
unit_target="/etc/systemd/system/engineai-native-sdk.service"

if [ ! -f "$unit_source" ]; then
    echo "Missing service unit: $unit_source" >&2
    exit 1
fi

if systemctl is-active --quiet robotics.service; then
    echo "robotics.service is active. It will remain running until the next reboot."
fi

install -m 0644 "$unit_source" "$unit_target"
systemctl daemon-reload
systemctl disable robotics.service
systemctl enable engineai-native-sdk.service

echo "Enabled engineai-native-sdk.service."
echo "robotics.service is disabled."
echo "The custom service was not started."

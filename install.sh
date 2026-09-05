#!/bin/bash

# Usage: ./install.sh <product_name> <mode> [arch]
# Example: 
#    ./install.sh pm_v2 robot x86_64
#    ./install.sh pm_v2 robot aarch64
# Exits on error
set -e

# Native SDK runs on the T800 Nezha Robot PC, not the Jetson Orin AI PC.
# The current T800 hardware reports aarch64 on both boards, so use the
# confirmed Nezha address and the platform check below to select the target.
# Replace remote_host with the confirmed Nezha address when using Ethernet.
remote_user="user"
remote_host="192.168.0.163"
remote_dir="~/projects/engineai_robotics"

# Sets the remote destination
remote_dest="${remote_user}@${remote_host}"

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)
local_scripts=("$source_dir/scripts/run_robot.sh" "$source_dir/scripts/set_imu_tty.sh")
local_assets_dir="$source_dir/assets"

usage() {
    echo "Usage: $0 <product_name> <mode> [arch]"
    echo "  product_name: the name of the product"
    echo "  mode: the mode of the product"
    echo "  arch: the architecture of the product (x86_64 or aarch64, default: x86_64)"
    echo "Example:"
    echo "    $0 pm_v2 robot"
    echo "    $0 pm_v2 robot x86_64"
    echo "    $0 pm_v2 robot aarch64"
    exit 0
}

# Requires product_name and mode; prints red error and exits if missing
if [ $# -lt 2 ] || [ -z "$1" ] || [ -z "$2" ]; then
    echo -e "\033[31mERROR: product name and mode are required.\033[0m"
    usage
    exit 1
fi
product_name="$1"
active_mode="$2"

if [ $# -ge 3 ]; then
    arch="$3"
    if [ "$arch" != "x86_64" ] && [ "$arch" != "aarch64" ]; then
        echo -e "\033[31mERROR: invalid arch: $arch\033[0m"
        usage
        exit 1
    fi
else
    arch="x86_64"
fi

local_install_dir="$source_dir/build/${arch}/_install"
echo "Installing for product: $product_name with mode: $active_mode and arch: $arch"

temp_dir=$(mktemp -d)
ssh_ctl_dir=$(mktemp -d)
ssh_control_path="${ssh_ctl_dir}/ssh_%r_%h_%p"
trap "ssh -o ControlPath=${ssh_control_path} -O exit ${remote_dest} 2>/dev/null || true; rm -rf $temp_dir $ssh_ctl_dir" EXIT

echo "temp_dir: ${temp_dir}"
rsync -av "${local_install_dir}" "${temp_dir}"
rsync -av "${local_scripts[@]}" "${temp_dir}"

# Copies assets for the given product
if [ -d "${local_assets_dir}/config/$product_name" ]; then
    echo "Copying assets for $product_name..."
    mkdir -p "${temp_dir}/assets/config"
    rsync -av "${local_assets_dir}/config/$product_name" "${temp_dir}/assets/config/"

    # Copy the whole robot resource directory so URDF, meshes and auxiliary XML stay in sync.
    mkdir -p "${temp_dir}/assets/resource/robot"
    if [ -d "${local_assets_dir}/resource/robot/$product_name" ]; then
        rsync -av "${local_assets_dir}/resource/robot/$product_name" "${temp_dir}/assets/resource/robot/"
    fi

    # Copy optional top-level mujoco xml for this product when available.
    mkdir -p "${temp_dir}/assets/resource"
    if [ -f "${local_assets_dir}/resource/${product_name}.xml" ]; then
        rsync -av "${local_assets_dir}/resource/${product_name}.xml" "${temp_dir}/assets/resource/"
    fi
else
    echo "Error: Product directory ${local_assets_dir}/config/$product_name does not exist"
    exit 1
fi

# Add git information if available (兼容普通仓库和 worktree)
git_dir=$( [ -d "$source_dir/.git" ] && echo "$source_dir/.git" || ( [ -f "$source_dir/.git" ] && sed 's/gitdir: //' "$source_dir/.git" ) )
if [ -n "$git_dir" ]; then
    echo "release_note.txt" > "${temp_dir}/release_note.txt"
    git_commit_id=$(git rev-parse HEAD)
    git_branch=$(git rev-parse --abbrev-ref HEAD)
    git_date=$(date "+%Y-%m-%d %H:%M:%S")
    echo "commit_id: ${git_commit_id}" >> "${temp_dir}/release_note.txt"
    echo "branch: ${git_branch}" >> "${temp_dir}/release_note.txt"
    echo "date: ${git_date}" >> "${temp_dir}/release_note.txt"
fi

# Opens a single SSH connection (password prompted once); mkdir and rsync reuse it
echo "Connecting to ${remote_dest} (enter password once)..."
ssh -o ControlMaster=yes -o ControlPath="${ssh_control_path}" -o ControlPersist=60 "${remote_dest}" "echo 'Connected.'"

# Refuse to copy an artifact to a board with a different CPU architecture.
remote_arch=$(ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" "uname -m" | tr -d '\r')
case "${arch}:${remote_arch}" in
    x86_64:x86_64|aarch64:aarch64|aarch64:arm64)
        ;;
    *)
        echo "ERROR: build architecture '${arch}' does not match remote '${remote_arch}' at ${remote_dest}." >&2
        echo "Native SDK real-robot deployment must target the confirmed Nezha Robot PC." >&2
        exit 1
        ;;
esac
echo "Remote architecture: ${remote_arch}"

# T800's Nezha and Jetson boards are both aarch64. Reject Jetson by its
# kernel/device-tree markers so a valid ARM64 artifact cannot go to the AI PC.
if [ "${product_name}" = "t800" ]; then
    remote_platform=$(ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" \
        "uname -r; tr -d '\\0' < /proc/device-tree/model 2>/dev/null || true")
    if printf '%s\n' "${remote_platform}" | grep -Eiq 'tegra|jetson|orin|nvidia'; then
        echo "ERROR: ${remote_dest} appears to be the Jetson/Orin AI PC." >&2
        echo "Deploy the T800 Native SDK to the Nezha Robot PC instead." >&2
        exit 1
    fi
    echo "Remote platform check passed for the Native SDK target."
fi

# Ensures the remote directory exists (reuses connection, no password)
ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" "mkdir -p $remote_dir"

rsync -avz --delete -e "ssh -o ControlPath=${ssh_control_path}" "${temp_dir}/" "${remote_dest}:$remote_dir"

# Updates active_mode in remote mode.yaml (same logic as in scripts/package.sh)
remote_mode_yaml="${remote_dir}/assets/config/${product_name}/mode.yaml"
ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" "sed -i 's/^active_mode:.*$/active_mode: $active_mode/' $remote_mode_yaml"
echo "Updated active_mode to '$active_mode' in remote $remote_mode_yaml"

echo "Congratulations! Installed to ${remote_user}@${remote_host}:${remote_dir}"

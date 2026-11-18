# EngineAI Native SDK: Native Control Framework for Humanoid Robots

[中文](README_CN.md) | English

**Documentation:** [EngineAI Native SDK Documentation](https://dx3a2bminsq.feishu.cn/wiki/KyD9wDc4mi03uXkTVuAc5LQan4C)

## Overview

This repository provides the EngineAI Native Control SDK, designed for humanoid robot application development and system integration. It delivers a lightweight, easily deployable control and task execution framework with strong extensibility. Through standardized interfaces and modular architecture, the SDK significantly reduces the complexity of secondary development and functional integration, allowing developers to focus on algorithm research and application implementation.

To support robot algorithm development, simulation verification, and real-robot deployment, the repository provides a complete runtime framework and simulation-deployment toolchain. Core modules include:

- **High-Performance Precompiled Scheduling Framework** — Provides a high-performance scheduling mechanism for real-time robot control and inference tasks. Precompilation optimizations reduce runtime overhead and improve overall system execution efficiency and stability.
- **Configuration-Driven Task Orchestration System** — Employs a configuration-based task orchestration approach for flexible organization of algorithm modules, control flows, and data pipelines, enabling rapid adjustment of experiment workflows and system operating strategies.
- **Extensible Model and Parameter Management System** — Provides unified interfaces for model loading, version management, and parameter configuration, facilitating centralized management and iteration of algorithm models, control parameters, and experiment configurations.
- **Modular Business Plugin Mechanism** — Based on a plugin architecture that supports flexible extension and decoupling of perception, planning, and control modules, enabling rapid integration of new algorithms and features.
- **Mujoco Simulation and Real-Robot Deployment Toolchain** — Provides a complete Mujoco simulation environment and real-robot deployment scripts, supporting fast migration from simulation verification to real robot systems.

Additionally, the repository includes URDF and other robot model files that are consistent with the real robot hardware structure, used for simulation environment construction, kinematics/dynamics computation, and algorithm verification, ensuring good consistency between simulation results and real systems.

## Repository Structure

```
native_sdk/
├── assets/              # Resource files (models, configs, etc.)
│   └── config/          # Robot-specific configuration files
├── core/                # Core framework library
├── docker/              # Container environment scripts
│   └── generate.sh      # Generate container dev environment
├── scripts/             # Utility scripts (simulation build/run, cross-compile, etc.)
│   └── cross_compile_env.sh  # Cross-compile container launcher
├── simulation/          # Mujoco simulation module
├── src/                 # Application source code
│   ├── runner/          # Motion control modules (Runner plugins)
│   ├── executor/        # Executor module
│   └── data/            # Data processing module
├── build.sh             # Build script (supports native and aarch64 cross-compile)
├── run.sh               # Run script
└── install.sh           # Real-robot deployment script
```

---

## 1. Development Environment & Quick Start

### 1.1 Container Environment

- **Install Docker and Docker Compose**  
  Docker provides an automated setup script that supports installation on Debian, RHEL, SUSE and their derivatives.  
  Note: Docker does not recommend using this script to install Docker CE in production.

  ```bash
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh get-docker.sh

  # Users in China may use a mirror (e.g. Tsinghua mirror)
  export DOWNLOAD_URL="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh get-docker.sh
  ```

- **Allow running Docker without sudo**

  ```bash
  sudo usermod -aG docker $USER
  ```

  **After running this, you must log out of your desktop session and log back in (or reboot) for the group change to take effect.**

- **Verify installation**

  ```bash
  docker --version
  ```

  Example output:

  ```plain
  Docker version 26.1.1
  ```

- **Verify Docker Compose**

  ```bash
  docker compose version
  ```

  Example output:

  ```plain
  Docker Compose version v2.27.0
  ```

- **Generate the container development environment.** Upon completion, a shortcut entry `engineai_robotics_env_1116` will be created:

```bash
cd native_sdk
./docker/generate.sh
```
![Container development environment generation](docs/generate_docker.png)

This command creates a container and mounts the current repository into it, so you can build and run the program inside the container.

Open a new terminal and enter the development environment using the shortcut command:

```bash
engineai_robotics_env_1116
```
![Inside the container development environment](docs/inside_docker.png)

### 1.2 Build

```bash
# Enter the container
engineai_robotics_env_1116

# Build (native x86_64, ROS 2 Humble by default)
./build.sh

# Optional: specify ROS 2 distro / build type
./build.sh -r humble -t release
```

Common `build.sh` options:

| Option | Description |
|:------:|:------------|
| `-a` | Architecture: `x86_64` (default) or `aarch64` |
| `-r` | ROS 2 distro: `humble` (default) or `jazzy` |
| `-t` | Build type: `release`, `debug`, `releasewithdebinfo` (default) |
| `-j` | Number of parallel compile jobs |
| `-h` | Show help |

#### 1.2.1 Cross Compilation (aarch64)

Some robots use an **aarch64** onboard computer. You can cross-compile binaries for the real robot on an x86_64 host. When `-a aarch64` is specified, `./build.sh` automatically launches the cross-compile Docker image via `scripts/cross_compile_env.sh` — run this from the **host machine** (not inside `engineai_robotics_env_1116`).

**Supported ROS 2 + architecture combinations:**

| ROS 2 | Architecture | Notes |
|:-----:|:------------:|:------|
| humble | x86_64 | Native build (default) |
| humble | aarch64 | Cross-compile |
| jazzy | aarch64 | Cross-compile |

```bash
# Optionally pull cross-compile images in advance
./scripts/cross_compile_env.sh pull

# Cross-compile for aarch64 (ROS 2 Humble by default)
./build.sh -a aarch64

# Cross-compile with ROS 2 Jazzy
./build.sh -a aarch64 -r jazzy
```

Artifacts are installed to `build/aarch64/_install`.

Images used:

- Humble: `ghcr.io/engineai-robotics/cross_compile_env:latest`
- Jazzy: `ghcr.io/engineai-robotics/cross_compile_env_jazzy:latest`

Interactive cross-compile environment (optional):

```bash
# Enter an interactive shell in the cross-compile container (Humble)
./scripts/cross_compile_env.sh shell

# Or use the Jazzy image
./scripts/cross_compile_env.sh --ros jazzy shell

# Run a one-off command in the container
./scripts/cross_compile_env.sh run "./build.sh -a aarch64"
```

### 1.3 Run

```bash
# Enter the container
engineai_robotics_env_1116

# Run with default robot model
./run.sh

# Run with a specific robot model
./run.sh pm01_edu
```

### 1.4 State Switching

After program startup, the robot switches between different motion states via remote controller commands. The Native SDK uses a **Finite State Machine (FSM) mechanism**:

- Each motion state defines explicit entry conditions and allowed state transitions. The system only permits state switching when conditions are met, ensuring the safety and stability of robot motion control.
#### Remote Controller

##### Physical Gamepad (Logitech F710)
- Use Logitech Wireless Gamepad F710 (Xbox mode)
- Automatically recognized after inserting the USB receiver
- All state transitions triggered by handpad buttons

##### Virtual Gamepad (Virtual Gamepad UI)
- Provides a graphical virtual gamepad interface
- Supports keyboard button and slider input simulation
- Uses the same control mapping relationship as F710 (XBox mode), with full control logic compatibility

```bash
# in docker
python3 tools/virtual_gamepad/virtual_gamepad.py
```

![Virtual Gamepad Interface](docs/virtual_gamepad.png)

#### System Startup

After executing `./run.sh` or `./run_robot.sh`, the system enters the **idle** state by default. `idle` is the initial safe state after the robot is powered on — the controller does not activate active motion control.

#### State Transition Overview

State transition logic varies by robot model. Refer to the per-model sections below for details.

##### pm01_edu

**State Description:**

| State | Description |
|:-----:|:------------|
| idle | Initial safe state after power-on. No active motion control is activated. |
| passive | Damping state. The controller applies passive damping torque; the robot can be moved manually. |
| pd_stand | Stable standing control task. The robot maintains a fixed upright posture via PD control. |
| walk | Walking task. The robot executes gait locomotion. |
| dance | Dance task. The robot executes predefined choreographed motion sequences. |
| rl_lab | RL Lab task. Deployment counterpart for the EngineAI open-source RL training framework [engineai_amp](https://github.com/engineai-robotics/engineai_amp). Executes policies trained with engineai_amp on real hardware. |

**State Machine Configuration:** `assets/config/pm01_edu/task_motion/default.yaml`

##### t800

**State Description:**

| State | Description |
|:-----:|:------------|
| idle | Initial safe state after power-on. No active motion control is activated. |
| passive | Damping state. The controller applies passive damping torque; the robot can be moved manually. |
| pd_stand | PD posture preparation task. It smoothly drives measured joints toward the configured target posture for motion recovery preparation. |
| walk | Walking task. The robot executes gait locomotion. |
| walk_leo | ImpactV2 DR 171000 main walking task, LB + A. |
| walk_v2_dr_194500 | ImpactV2 DR 194500 walking task, LB + d-pad up. |
| walk_v2_dr_191500 | ImpactV2 DR 191500 walking task, LB + d-pad right. |
| walk_plus_origin_180000 | ImpactV2Plus 180000 walking task, LB + d-pad down. |
| walk_plus_dr_189000 | ImpactV2Plus DR 189000 walking task, LB + d-pad left. |
| leo_taunt | Walking taunt on the main gait model, LB + Y. It is available from `walk_leo` and returns there automatically. |
| supine_to_stance | Stand-up task using the local `rl_dance_example_runner` with a 50 Hz, 134-observation/25-action policy. Trigger: START + d-pad up; `walk_leo` can also be selected manually before the trajectory ends. |
| prone_to_stance | Prone get-up using `rl_supermimic_runner` at 50 Hz, with 1520 inputs and 25 outputs. Trigger: START + d-pad right. |
| getup | HoST get-up. Trigger: START + d-pad left; upright detection hands off directly to the main gait. |
| getup2 | Second HoST get-up policy. Trigger: LB + START; LB + BACK is unassigned. |
| supermimic_action_06 | 06 High left flick punch, 32000 iterations, RB + d-pad up. |
| supermimic_action_09 | 09 Left hook, 20000 iterations, RB + d-pad down. |
| supermimic_action_11 | 11 Left pull-down hammer, 18000 iterations, RB + d-pad left. |
| supermimic_action_13 | 13 Mid left flick punch, 32000 iterations, RB + d-pad right. |
| supermimic_action_14 | 14 Left front kick, 18000 iterations, RB + B. |
| supermimic_action_15 | 15 Lead-leg side stomp, 18000 iterations, RB + A. |
| supermimic_action_16 | 16 Low left snap kick, 26000 iterations, RT + d-pad left. |
| supermimic_action_17 | 17 High left snap kick, 18000 iterations, RT + d-pad right. |
| supermimic_action_19 | 19 Low left roundhouse, 18000 iterations, RB + Y. |
| supermimic_action_20 | 20 Mid left roundhouse, 18000 iterations, RB + X. |
| stance_to_supine | Lie-down task. The robot transitions from standing posture to a supine (lying on back) posture. |

**State Machine Configuration:** `assets/config/t800/task_motion/default.yaml`

`LB+A` enters `walk_leo`, whose `rl_walking_leolab_example` tag now selects the former `LB + d-pad right` model at `assets/config/t800/rl_walking_leolab_impact_v2_dr_171000/default.yaml`. This DR 171000 policy is now the main walking model. The previous AnkleCombinedV2 140000 and ImpactV2 154500 walking packages were removed.

Four additional Leo walking policies remain as independent states. `LB` + d-pad up selects `walk_v2_dr_194500`, d-pad right selects `walk_v2_dr_191500`, d-pad down selects `walk_plus_origin_180000`, and d-pad left selects `walk_plus_dr_189000`. They use the same recurrent Leo runner and 84-observation / 25-action contract as the main gait.

The 10 formal SuperMimic motions use `rl_supermimic_runner` with the exported 1520-input / 25-output contract. They can be entered from any of the three PD postures or the current walking states and automatically return to `walk_leo` after the reference clip. Their packages are shared with virtual tests 06, 09, 11, 13–17, and 19–20 under `assets/config/t800/numbered_actions/`.

`RT` + d-pad up/down are unassigned. The current input encoding uses simultaneous d-pad up/down as the RT marker, so RT+up and RT+down cannot select distinct actions. Physical controls use `supermimic_action_NN` states. The default checkout does not provide a numbered virtual-test catalog; `test_action_NN` states are generated only for a custom catalog.

`prone_to_stance` uses `rl_supermimic_prone_j12768_m22000`. The reference retains 143 rows, while runtime skips the last 12 rows for about 2.60 seconds of playback. `hold_seconds: 0.0` adds no terminal hold. Completion returns directly to `walk_leo`. `leo_taunt` now uses `LB+Y` and the new main gait model as its policy base. The old `walk_impact` and `walk_v2_dr_171000` states are removed. Punch references 06/09/11/13 trim their low-speed tails by 12/8/8/13 frames. Leg references remain unchanged.

#### Global Safety Mechanism (Emergency Fallback)

> **Any state** can be forcefully switched to `passive` via **`LB + RB`**.

This functions as a **Soft Emergency Stop**:

- Immediately terminates the current motion control logic
- Returns the system to a safe passive state
- Critical for debugging and real-world operation — reduces the risk of motion control runaway

### 1.5 Mujoco Simulation

#### 1.5.1 Build

```bash
# Enter the container
engineai_robotics_env_1116

# Build the simulation module
./scripts/build_mujoco.sh

# If GitHub is not accessible, use this command to fetch dependencies from Gitee and build
./scripts/build_mujoco.sh -m
```

#### 1.5.2 Run

> Before running, ensure that `active_mode` is set to `sim` in `assets/config/<robot>/mode.yaml`.

```bash
# Enter the container
engineai_robotics_env_1116

# Run simulation (default robot model)
./scripts/run_mujoco.sh

# Run with a specific robot model
./scripts/run_mujoco.sh pm01_edu
```

After launching, use the remote controller to switch states.

#### 1.5.3 Improving Simulation Performance

If you have a dedicated NVIDIA GPU with proper drivers installed, you can enable GPU passthrough in Docker to improve simulation rendering frame rates.

**Step 1: Install the NVIDIA Docker Toolkit**

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Step 2: Enable GPU Passthrough**

Set `NVIDIA_GPU_AVAILABLE` to `y` in `docker/generate.sh`:

```bash
NVIDIA_GPU_AVAILABLE=y
```

**Step 3: Regenerate the Container Environment**

```bash
./docker/generate.sh
```

After restarting the container, GPU-accelerated rendering will be available.

---

### 1.6 Real-Robot Deployment

> **T800 note:** The Native SDK real-robot executor runs on the Nezha Robot PC, not the Jetson AGX Orin AI PC. The current confirmed target is `user@192.168.0.163`; both T800 boards report `aarch64`, so identify the board by its address/platform rather than CPU architecture. Use the repository's `真机部署.txt` for the current T800 deployment flow. The generic values below are retained for other product examples.

#### 1.6.1 Configure Deployment Target

Edit the deployment target parameters in `install.sh`:

```bash
remote_user="user"
remote_host="192.168.0.163"
remote_dir="~/projects/engineai_robotics"
```

Execute the installation:

```bash
cd native_sdk

# ./install.sh <robot_model> <mode> [arch]
# arch is optional: x86_64 or aarch64 (default: x86_64)
./install.sh pm01_edu robot

# Deploy aarch64 artifacts (after ./build.sh -a aarch64)
./install.sh pm01_edu robot aarch64
```

#### 1.6.2 Running on the Real Robot

> [!CAUTION]
> **Safety Notice:**
> - Ensure all personnel maintain a safe distance from the robot
> - If the robot behaves abnormally, stop it immediately (press the emergency stop button or switch to `passive` mode)
> - It is recommended to suspend the robot on a gantry first. After entering `pd_stand` mode, lower it to the ground before switching to walking mode

**Pre-run Preparation:**

1. Enable the robot's motor system using the emergency stop remote controller
2. Connect to the robot's hotspot or use an Ethernet cable to connect to the robot

**Startup Steps:**

```bash
# SSH into the robot (Nezha)
ssh user@192.168.0.163

# Stop the auto-started motion control service
sudo systemctl stop robotics.service

# Launch native_sdk
cd ~/projects/engineai_robotics
sudo ./run_robot.sh pm01_edu
```

**Running in Background:**

```bash
nohup sudo ./run_robot.sh pm01_edu > nohup.out 2>&1 &
tail -f nohup.out
```

After launching, use the remote controller to switch actions as described in [State Switching](#14-state-switching).

## License

This project is licensed under the [BSD 3-Clause License](LICENSE.txt).

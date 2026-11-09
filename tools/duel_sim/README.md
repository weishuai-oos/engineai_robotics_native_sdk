# T800 单机器人与双机器人仿真

## AnkleCombinedV2 最终模型：导出与替换走路策略

本次仅导出最终的 **`ckpt_140000.pt`**。该任务从 100000 轮继续训练 40000 轮，最终轮数为 **140000**。`LB+A` 的 `walk_leo` 现已切换到该模型，本机和 109 共用下述配置。`mode.robot`、`mode.sim` 的走路 scope 都已更新；`active_mode` 仍为 `sim`。

### 部署目录只保留运行文件

| 文件 | 用途 |
| --- | --- |
| `assets/config/t800/rl_walking_leolab_ankle_combined_v2_140000/default.yaml` | 走路配置，四个踝关节 Kd 为 2.0 |
| 同目录 `policies/boxing_policy.mnn` | SDK 实际加载的 FP32 循环策略，兼容目标 MNN 2.9 |

### 1. 从训练框架导出这一个 checkpoint

在**本机训练环境**执行，使用已有 `leo` 环境。`scripts/cusrl/export_ankle_v2.py` 在 CPU 上严格加载 actor 与观测 RMS，保留归一化后的 ±10 裁剪和 LSTM 状态，不启动 Isaac Sim。原 `export.py` 的任务选项尚未支持 V2。

当前部署模型已准备好，无需重复导出。需要重新导出时，仅指定最终的一个 PT；ONNX、接口描述和导出记录写入仓库外的新临时目录，转换完成后只将 MNN 复制到部署目录：

```bash
LEO_DIR=/srv/shared/home/xjh/workspace/zq_leo_final
SDK_DIR=/srv/shared/home/xjh/workspace/zq/engineai_robotics_native_sdk
LEO_PY=/srv/shared/home/xjh/app/miniconda3/envs/leo/bin/python
MNN_PY=/srv/shared/home/xjh/app/miniconda3/envs/whole_body_tracking/bin/python
CKPT_PATH="$LEO_DIR/logs/cusrl/T800_base-AnkleCombinedV2/2026-11-07-23-48-06:T800_base-AnkleCombinedV2_finetune_2gpu_seed42/ckpt/ckpt_140000.pt"
OUT_DIR=$(mktemp -d /tmp/engineai-ankle-v2.XXXXXX)
DEPLOY_DIR="$SDK_DIR/assets/config/t800/rl_walking_leolab_ankle_combined_v2_140000/policies"

"$LEO_PY" "$LEO_DIR/scripts/cusrl/export_ankle_v2.py" \
  --checkpoint "$CKPT_PATH" \
  --expected-iteration 140000 \
  --output-dir "$OUT_DIR"

"$MNN_PY" -m MNN.tools.mnnconvert \
  -f ONNX --modelFile "$OUT_DIR/actor.onnx" \
  --MNNModel "$OUT_DIR/boxing_policy.mnn" \
  --bizCode MNN --targetVersion 2.9
```

确认以上两步成功后，退出正在使用该模型的 SDK，再复制最终 MNN：

```bash
cp "$OUT_DIR/boxing_policy.mnn" "$DEPLOY_DIR/boxing_policy.mnn"
```

本机 MNN 转换工具为 3.6.1，109 原 SDK 容器的推理库为 MNN 2.9.5，因此显式指定 `--targetVersion 2.9` 并保留 FP32 权重。临时目录不参与运行，完成转换和复制后可删除；不要将其中的 ONNX、`actor.yml` 或导出记录复制回部署仓库。

部署接口如下；重新导出时可在临时目录的 `actor.yml` 核对：

| 输入 | 形状 | 输出 | 形状 |
| --- | --- | --- | --- |
| `observation` | `[1, 1, 84]` | `action` | `[1, 1, 25]` |
| `memory_in.0` | `[2, 1, 256]` | `memory_out.0` | `[2, 1, 256]` |
| `memory_in.1` | `[2, 1, 256]` | `memory_out.1` | `[2, 1, 256]` |

现有 runner 发送原始 84 维观测，并在每步反馈 hidden/cell；RMS 已在模型内，无需在 C++ 再做归一化。

### 2. 当前启用的走路配置

`assets/config/t800/mode.yaml` 的 **`mode.sim` 和 `mode.robot`** 都已按下面的 tag 选择新模型，按键与状态机保持原样：

```yaml
    - tag: rl_walking_leolab_example
      scope: rl_walking_leolab_ankle_combined_v2_140000/default
```

完成 PD 准备后，`LB+A` 进入的 `walk_leo` 即使用 V2；双机中 A、B 都使用该模型，但各自维护独立的观测和 LSTM 状态。原本自动回到 `walk_leo` 的拳脚和起身动作，现在也回到 V2。`LB+X` 的 `walk_leo_terrain` 保留原模型。

新 YAML 以 `rl_walking_leolab_terrain_example/default.yaml` 为基准，参数差异只有以下两项：

| 参数 | 新 `walk_leo` / LB+A | `walk_leo_terrain` / LB+X |
| --- | --- | --- |
| 模型 | AnkleCombinedV2 最终 `ckpt_140000` | 原 LowCylinders-AnkleStable-1 |
| 四个踝关节 Kd（索引 4、5、10、11） | **2.0** | 0.3 |

两者 Kp、25 维关节顺序、默认姿态、动作缩放、84 维观测、50 Hz 控制周期和动作切入过渡一致；遥控的 37°方向偏置、启停阈值、反向保护、加速度 `5.0 m/s²`、减速度 `2.0 m/s²` 也一致。固定战术方向目标为：前进 `1.0 m/s`、后退/左右 `0.8 m/s`、左右转向 `1.5 rad/s`。这比旧 `walk_leo` 的平移 `0.5 m/s`、转向 `1.0 rad/s` 更快；这些数值是指令目标，实际速度和稳定性由新策略决定。

旧 `LB+A` 的配置及其实际引用的模型保留在 `assets/config/t800/old/rl_walking_leolab_example`，用于回退；历史模型和未使用的副本已清理。根目录不再保留旧目录的兼容软链接，新配置通过独立目录加载。`leo_taunt` 原本共用旧目录的 MNN，现改为引用 terrain 目录下**字节完全相同**的模型，其余参数保持原样，因此它仍使用原策略及自己的 PD。

### 3. 本机与 109 启动、回退

两边仓库已同步新模型、配置、归档和相关说明。109 继续使用已有 `engineai_robotics_env` 容器，无需重建容器、修改网络或重新编译 C++。**已运行的仿真需正常退出并重新启动后才会加载新选择**，本次没有停止现有双机进程。保持 `active_mode: sim`，在 SDK 容器中任选一种：

```bash
./scripts/run_single.sh --virtual-gamepad
./scripts/run_duel.sh --virtual-gamepad
```

也可使用下文实体手柄启动命令。双机的配置在启动时复制到临时目录，修改源 YAML 不会影响已经运行的进程。实机模式也已映射到同一 V2 配置及四踝 Kd=2.0，但本次没有启动实机；模型转换数值一致不等于实机稳定性验证。

需要回退旧 `LB+A` 时，先退出 SDK，按 `assets/config/t800/old/README.md` 的“旧 Leo 走路”步骤移回旧目录，将两个模式下该 tag 的 scope 恢复为 `rl_walking_leolab_example/default` 后重启。其他模型和按键配置不用恢复。

### 4. 当前走路目录

| 目录 | 状态 / 按键 |
| --- | --- |
| `rl_walking_leolab_ankle_combined_v2_140000` | 当前 `walk_leo` / LB+A |
| `rl_walking_leolab_terrain_example` | `walk_leo_terrain` / LB+X；嘲讽步态也共用其中的当前模型 |
| `rl_walking_leolab_taunt` | `leo_taunt` / LT+B，仅从 `walk_leo` 进入 |
| `rl_walking_example` | `walk` / LB+B |
| `rl_walking_custom_example` | `walk_custom` / LB+Y |
| `old/rl_walking_leolab_example` | 原 LB+A 的运行备选，保留配置与其引用的 MNN |

terrain 根目录下的 `2026-09-06-19-30-18:t800_lowcylinders_ankle_stable_1_seed42/boxing_policy.mnn` 仍由 LB+X 和嘲讽步态使用。原 LB+A 同名模型在 `old` 中保留作回退。

## 仿真框架说明

一个 MuJoCo 场景包含两台 T800，两套独立 executor 各自运行原有状态机和策略。两台使用同一份模型、轨迹和动作配置，但观测、循环记忆、动作进度和手柄输入分别维护。双机入口不会修改 `assets/config/t800/mode.yaml`；单机器人仿真和实机部署入口分别读取 `mode.sim` 和 `mode.robot` 的选择。

## 启动前准备（单机、双机共用）

在 109 电脑上做本机仿真时，先在**宿主机终端（进入容器前）**准备组播路由，再进入已有的 SDK 开发容器：

```bash
sudo ip route replace 239.255.76.67/32 dev lo
engineai_robotics_env
```

路由命令用于避免本机 LCM 通信被 VPN 的 `ppp0` 接口接管，可以重复执行。已有容器使用宿主机网络，单独重启容器通常不会清除这条路由；宿主机重启后需重新添加，网络服务删除或重置路由后也需检查。该命令只用于同一台电脑里的仿真；使用同一组播地址做跨机或实机通信前，按文末说明撤销路由。

以下编译、测试和启动命令均在该容器内的仓库目录执行。宿主机的 Conda `(base)` 环境不包含容器里的 SDK 依赖；若找不到 `lcmConfig.cmake` 或 Pinocchio，先确认已经进入容器。

```bash
cd /home/xjh/workspace/zq/engineai_robotics_native_sdk
```

首次使用或修改 C++ 代码后，重新编译控制器和仿真；已有最新构建产物时可跳过：

```bash
./build.sh
./scripts/build_mujoco.sh
```

## 单机器人仿真

先确认 `assets/config/t800/mode.yaml` 中为：

```yaml
active_mode: sim
```

推荐在容器内用一个终端同时启动 T800 仿真和控制器：

```bash
./scripts/run_single.sh --gamepad /dev/input/js0
```

默认手柄为 `/dev/input/js0`，也可以直接运行 `./scripts/run_single.sh`。入口自动加载 ROS 环境、指定实体手柄输入；关闭仿真窗口、任一子进程退出或按 `Ctrl+C`，会一起停止本次启动的仿真和控制器。控制台及 ROS 日志保存在临时目录下的 `engineai-single-logs-*`，退出后保留，启动时会显示路径；可用 `--log-dir PATH` 指定目录。

也可以让虚拟手柄控制单机：

本次新增了独立的 `virtual` 输入源，首次使用前需在原 SDK 容器内执行一次 `./build.sh` 重编控制器；无需重编 MuJoCo。未传 `--virtual-gamepad` 时仍使用原实体手柄流程。

```bash
./scripts/run_single.sh --virtual-gamepad
```

该命令会自动打开虚拟手柄界面，并连接单机 LCM 地址 `udpm://239.255.76.67:7667?ttl=0`。界面连接后仍需按 F5 或勾选 `Enable` 才会开始发送；方向键控制平移，N/M 控制转向，动作组合键与实体手柄相同。虚拟模式不需要 `/dev/input/js0`。

如果已有单机、双机或实机 SDK 进程在运行，新入口会提示并拒绝重复启动。先在原启动终端正常退出，再切换模式。

只检查启动配置，或只运行单机场景做无界面检查：

```bash
./scripts/run_single.sh --dry-run
./scripts/run_single.sh --no-controllers --headless --steps 1000
```

单机直接读取 `assets/config/t800`，动作键位仍改 `task_motion/default.yaml`，修改 YAML 后重启入口生效。`LT+A` 使用原 `pd_stand/default.yaml` 中的姿态目标，并非下文双机临时配置的零位准备姿态，应按原有姿态准备和起身流程进入步态。单机场景默认启用吊绳辅助；需要观察自由站立和平衡时，在仿真窗口按数字键 `9` 关闭吊绳，控制台会显示 `Elastic band disabled`。

### 原双终端启动方式（可选）

原 `run_mujoco.sh` 和 `run.sh` 仍可分别启动仿真和控制器，方便单独调试。先退出其他 SDK 实例；原 `run.sh` 也会检查已有 `src_executor`，发现仍在运行就拒绝重复启动。

**终端 1：**完成上述容器和目录准备后，启动单机器人 MuJoCo 窗口：

```bash
source /opt/ros/humble/setup.bash
./scripts/run_mujoco.sh t800
```

**终端 2：**新开一个宿主机终端，进入同一个已有容器，启动单机器人控制器：

```bash
engineai_robotics_env
cd /home/xjh/workspace/zq/engineai_robotics_native_sdk
source /opt/ros/humble/setup.bash
ENGINEAI_INPUT_SOURCE=gamepad ENGINEAI_GAMEPAD_DEVICE=/dev/input/js0 ./run.sh t800
```

两边都需保持运行。`/dev/input/js0` 可换成实际手柄的 joystick 设备路径；这里显式选择实体手柄输入，避免其他输入源覆盖。原 `run.sh` 需要在该终端手动加载 ROS 环境，以便找到运行库；统一入口会自动加载。

使用双终端方式时，结束需关闭仿真窗口，并在终端 2 按 `Ctrl+C` 退出控制器；两个独立入口不会自动互相退出。

## 双机器人仿真

将两只 Xbox 或兼容 Linux joystick 的手柄接入同一台仿真计算机，运行：

```bash
./scripts/run_duel.sh --gamepad-a /dev/input/js0 --gamepad-b /dev/input/js1
```

无需再单独运行 `run.sh`。A 出生在左侧、朝向正 X，B 出生在右侧、朝向负 X；场景是平地，初始脚底约离地 5 mm，默认无吊绳。双机临时配置将 `LT+A` 的 PD 准备目标设为出生时的零关节角，保留原增益和插值时长；原单机和实机的 PD 配置不变。

入口会在自己的进程中加载已有 ROS 环境，让控制器找到容器里的动态库，不修改终端或容器配置。

双机窗口默认暂停。先让双方分别用各自手柄按 `LT+A` 进入 PD 准备，释放按键后，按窗口中的空格开始物理仿真；等双方实际站稳后再分别用 `LB+A` 切换到 Leo 步态。暂停期间控制器仍会计时，因此应在物理运行时切换策略或触发动作。摇杆控制自身前后、横移和转向，拳脚动作沿用原配置。完整键位以 `assets/config/t800/task_motion/default.yaml` 为准。

双机也支持先用一个虚拟手柄控制 A：

```bash
./scripts/run_duel.sh --virtual-gamepad
```

虚拟界面自动连接 A 的 `udpm://239.255.76.67:17667?ttl=0`；如果使用 `--port-a PORT`，界面会跟随连接对应端口。B 仍按实体手柄模式启动，可用 `--gamepad-b /dev/input/js0` 指定设备；虚拟模式下不要求 A 的实体手柄。若 B 没有可用手柄，它会保持初始状态，不会自动完成 PD 准备或站立，物理仿真开始后可能倒地。

掉地后可分别使用下述俯卧起身或原有仰卧起身。GUI 的 Reset 只复位共享物理世界，不会同时清空两个 executor 的策略记忆和状态机；需要完整重开一局时退出并重新运行此入口。关闭窗口、任一子进程退出或按 `Ctrl+C` 会停止本次入口启动的全部子进程。

若设备支持，优先用稳定的设备链接避免拔插后 js0/js1 交换：

```bash
ls -l /dev/input/by-id/*-joystick
./scripts/run_duel.sh \
  --gamepad-a /dev/input/by-id/<手柄A>-joystick \
  --gamepad-b /dev/input/by-id/<手柄B>-joystick
```

实体手柄路径必须指向 Linux joystick 设备，不能使用 `event-joystick`。不同型号、USB/蓝牙驱动可能报告不同的轴和按键布局；首次使用需确认摇杆、LT/RT 和动作组合的实际映射。默认实体模式下两实例固定使用 gamepad 输入；`--virtual-gamepad` 模式下 A 使用虚拟输入，B 仍使用 gamepad 输入，不会由 RC02 覆盖。

虚拟手柄依赖 PyQt6 和 Python LCM 包。原 SDK 容器已有依赖时可直接启动；缺少时使用 `python3 -m pip install -r tools/virtual_gamepad/requirements.txt` 安装。界面使用容器原有的图形环境。关闭虚拟手柄窗口、仿真窗口、任一子进程退出或按 `Ctrl+C`，都会结束本次入口启动的全部进程。

## SuperMimic 动作与备用目录

单机、双机共用以下 T800 动作映射：

| 按键 | 动作 | 状态名 |
| --- | --- | --- |
| RB+B | 嘲讽 | `supermimic_come_here` |
| RB+Y | 左勾拳 | `supermimic_left_hook` |
| RB+X | 拉锤 | `supermimic_pull_hammer` |
| RB+A | 右高踢腿 | `supermimic_right_high_kick` |

从 PD 准备或原有步态中进入，动作参考播放完成后自动回到 `walk_leo`。四个动作各自的 `default.yaml` 当前设置 `hold_seconds: 0.0`，不额外保持末尾目标；走路侧的进入过渡仍保留。修改该值不需要重新编译，重启单机或双机入口后生效；双机在启动时复制配置。其他原有动作和急停组合保留，完整允许切换关系仍以 `assets/config/t800/task_motion/default.yaml` 为准。

当前 `RB+B` 的嘲讽使用第一个重训包 `rl_supermimic_zq_come_here_01_j13904_0_m36000`，`mode.sim` 与 `mode.robot` 使用同一包。参考共 501 帧、50 Hz、10 秒，额外保持仍为 `hold_seconds: 0.0`。第二个包 `rl_supermimic_zq_come_here_02_j13904_1_m36000` 和第三个包 `rl_supermimic_zq_come_here_03_j13904_2_m38000` 保留在 `assets/config/t800/old/嘲讽/` 留作备用，每包仅含运行配置、MNN 和参考轨迹。正常退出并重启单机或双机入口后生效，虚拟手柄按钮仍显示状态名 `supermimic_come_here`，不需要重新编译。

四个动作使用专用 `rl_supermimic_runner`。每包仅保留 `default.yaml`、`policies/policy.mnn` 和 `trajectories/supermimic_reference.npz`；导出中间文件和评测数据不参与部署。旧 dance、victory、ridicule 及更早的俯卧起身保留在 `assets/config/t800/old`，恢复步骤见该目录的 `README.md`。首次接入 runner 才需要在已有容器内执行 `./build.sh`。

## 俯卧起身

单机、双机的 `prone_to_stance` 使用旧俯卧 `rl_supermimic_prone_j12768_m22000`，`mode.sim` 与 `mode.robot` 的 `rl_prone_to_stance` tag 均选择该包，按键为 **START + 十字键右**。可从 `passive` 或三种 PD 准备状态（`pd_stand`、`pd_stand_x`、`pd_stand_y`）进入；执行中可手动切换到 `passive`、`pd_stand`、`walk_leo`，完成后自动回到 `walk_leo`。

该包使用现有 `rl_supermimic_runner`，输入 1520 维、输出 25 维，以 50 Hz 完整播放 143 帧参考轨迹，共 2.84 秒。模型输出和上一动作历史均为 raw action，目标为 `default_joint_pos + action_scale * raw_action`。部署 `hold_seconds: 0.0`，不额外保持末尾；原模型、参考、PD 和关节映射保留。入口为策略实时运行的命令衔接，不执行上键 WBT 的纯 PD 等待；起身回 Leo 目前直接接管，没有四个格斗动作回步态时的入口混合。

原 **START + 十字键上** 仍使用 WBT 仰卧，HoST 起身也不变。虚拟手柄的 `prone_to_stance` 按钮执行该旧俯卧。更早的 `rl_prone_to_stance_hybrid_9825_m29500` 仍在 `assets/config/t800/old`。双机复制配置发生在启动时，正常退出并重启单机或双机入口后生效，无需重新编译。

## 双机配置与日志隔离

- 每次运行在系统临时目录下的 `engineai-duel-*` 生成双机 MJCF 及两份私有 assets 配置，退出后自动删除；`--keep-runtime` 可保留用于检查。
- YAML 配置使用真实目录和独立副本，以兼容 SDK 的配置扫描；策略文件、轨迹、URDF 等资源继续共享，只修改临时副本中的仿真模式、PD 准备目标、通信、ROS 话题和日志设置。
- 两端默认使用 `239.255.76.67:17667` 和 `239.255.76.67:17668`，保持原 `sim_state` / `sim_command` 消息契约。可用 `--port-a`、`--port-b` 指定不同端口。
- 两控制器分别使用 ROS domain 70/71，并将话题加上 `/robot_a`、`/robot_b` 前缀；`--ros-domain-a N` 改为 N/N+1。
- 控制台及 SDK 日志分别写入独立目录。默认保存在系统临时目录下的 `engineai-duel-logs-*`，退出后保留；用 `--log-dir PATH` 指定保存位置。

## 双机检查与无界面运行

只生成并打印启动参数，不启动仿真或控制器：

```bash
./scripts/run_duel.sh --dry-run
./scripts/run_duel.sh --dry-run --keep-runtime
```

只查看双机场景，不连接手柄或启动策略：

```bash
./scripts/run_duel.sh --no-controllers
```

无界面运行有限步数，适合场景和通信检查；没有控制器时机器人会自然倒下：

```bash
./scripts/run_duel.sh --no-controllers --headless --steps 1000
```

可用 `--simulator PATH`、`--executor PATH` 指定构建产物；`--separation 2.0` 设置两台初始底座距离，单位米。入口仅支持 T800 双机，原其他机型入口不变。

不依赖硬件或 SDK 运行库的入口测试：

```bash
python3 -m unittest discover -s tools/duel_sim -p 'test_*.py' -v
```

在已安装 SDK 编译依赖的环境中，可额外运行机器人状态与扭矩映射测试：

```bash
cmake -S simulation/mujoco -B simulation/mujoco/build -DBUILD_TESTS=ON
cmake --build simulation/mujoco/build --target test_robot_binding
ctest --test-dir simulation/mujoco/build -R '^test_robot_binding$' --output-on-failure
```

该测试也接受生成的真实场景，额外检查 50 路电机隔离、零位 PD 准备和跨机器人碰撞：

```bash
python3 -c 'from pathlib import Path; from tools.duel_sim.launcher import generate_scene, REPO_ROOT; generate_scene(REPO_ROOT / "assets", Path("/tmp/t800_duel_check.xml"))'
simulation/mujoco/build/test_robot_binding /tmp/t800_duel_check.xml
```

完整 SDK 启用 `BUILD_TESTS` 后，还可运行 `test_input_binding`，验证双路手柄输入、设备断开清零和重连。

两台互相接触和打击后的平衡能力取决于现有策略。该功能提供双人控制和共享物理接触，未添加胜负判定、自动瞄准或对手观测。

## VPN/PPP 环境的本机通信

若启动时报 `LCM self test failed` 或 `LCM receiver initialization failed`，先在宿主机查看组播地址的路由：

```bash
ip route get 239.255.76.67
```

若结果走 `ppp0` 等 VPN 接口，可在仅运行本机仿真时临时将这个地址送回本机：

```bash
sudo ip route replace 239.255.76.67/32 dev lo
```

`replace` 在路由已存在时更新它，因此重复执行不会报 `File exists`。这是临时设置，不会写入开机网络配置；容器重启通常不影响它，宿主机重启后则需重新执行。可用上面的 `ip route get` 确认结果为 `dev lo`。

结束仿真后，在宿主机撤销本次添加的路由：

```bash
sudo ip route del 239.255.76.67/32 dev lo
```

这条路由覆盖该组播地址的所有端口，包括原单机端口 7667 和双机端口 17667/17668；使用同一地址的跨电脑或实机通信前需撤销。它不修改默认路由或 VPN 配置，双机入口也不会自动修改系统路由。已有容器使用宿主机网络，因此在宿主机操作即可，无需更改容器网络。

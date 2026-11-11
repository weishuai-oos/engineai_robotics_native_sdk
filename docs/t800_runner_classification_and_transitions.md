# T800 动作分类、Runner 与切换衔接

更新日期：2026-11-11。本文描述当前部署配置，适用于单机与双机中的每个 T800。
正式动作以 [task_motion/default.yaml](../assets/config/t800/task_motion/default.yaml) 为准，策略目录由 [mode.yaml](../assets/config/t800/mode.yaml) 选择。当前有 **24 个正式状态**；使用虚拟手柄启动时，单机或双机 A 额外注册 **5 个编号测试动作**，编号容量仍为 50。编号测试通过会话临时配置注册，不写入正式实机动作图。

## 正式动作与 Runner

Runner 是执行动作的程序，同一个 runner 可以运行不同模型；名字不能代替对配置用途的判断。

| 类别 | 状态名称 | 遥控按键 | Runner |
|---|---|---|---|
| 初始状态 | `idle` | BACK+START | `idle_runner` |
| 速度阻尼 | `passive` | LB+RB | `passive_runner` |
| 三种 PD 姿态 | `pd_stand` / `pd_stand_x` / `pd_stand_y` | LT+A / LT+X / LT+Y | `pd_stand_runner` |
| 普通走路 | `walk` | LB+B | `rl_walking_example_runner` |
| CombinedV2 走路 | `walk_leo` | LB+A | `rl_walking_leolab_example_runner` |
| Terrain 走路 | `walk_leo_terrain` | LB+X | `rl_walking_leolab_example_runner` |
| ImpactV2 走路 | `walk_impact` | LB+Y | `rl_walking_leolab_example_runner` |
| 走路策略上的嘲讽 | `leo_taunt` | LT+B | `rl_walking_leolab_example_runner` |
| SuperMimic 嘲讽 | `supermimic_come_here` | RB+B | `rl_supermimic_runner` |
| SuperMimic 左勾拳 | `supermimic_left_hook` | RB+Y | `rl_supermimic_runner` |
| SuperMimic 拉锤 | `supermimic_pull_hammer` | RB+X | `rl_supermimic_runner` |
| SuperMimic 右高踢腿 | `supermimic_right_high_kick` | RB+A | `rl_supermimic_runner` |
| SDK 左前踢 | `rl_left_front_kick_002_complete_heightfall` | RB+十字上 | `rl_dance_example_runner` |
| SDK 回旋踢 | `rl_roundhouse_kick` | RB+十字下 | `rl_dance_example_runner` |
| SDK 左直拳 | `rl_straight_punch_L_improved` | RB+十字左 | `rl_dance_example_runner` |
| SDK 左勾拳 | `rl_left_hook_001_improved` | **RT+十字右** | `rl_dance_example_runner` |
| SDK 示例直拳 | `rl_punch_example` | RB+十字右 | `rl_dance_example_runner` |
| 俯卧起身 | `prone_to_stance` | START+十字右 | `rl_supermimic_runner` |
| 仰卧起身 | `supine_to_stance` | START+十字上 | `rl_dance_example_runner` |
| HoST 起身 | `getup` / `getup2` | LB+START / LB+BACK | `rl_getup_example_runner` |
| 主动躺下 | `stance_to_supine` | START+十字下 | `rl_mimic_trajectory_runner` |

普通 `walk` 的策略周期为 0.01 秒；三个 Leo 走路、参考动作跟踪和起身策略的周期为 0.02 秒。三种 PD 目标分别由 `pd_stand` 配置决定，仿真与实机的 `pd_stand` scope 可以不同。

当前几个重要的模型选择如下，目录均相对于 `assets/config/t800/`：

| 按键 | 当前策略目录 | 说明 |
|---|---|---|
| LB+A | `rl_walking_leolab_ankle_combined_v2_140000` | CombinedV2 140000 轮，四踝 Kd=2.0；状态名仍为 `walk_leo` |
| LB+Y | `rl_walking_leolab_impact_v2_154500` | ImpactV2 154500 轮；已替换原 `walk_custom` |
| RB+B | `rl_supermimic_zq_come_here_01_j13904_0_m36000` | 第一个新版嘲讽，36000 轮 |
| RB+Y | `rl_supermimic_zq_left_hook_02_j13564_m82000` | 左勾拳，82000 轮 |
| RB+X | `rl_supermimic_zq_pull_hammer_20261102_j13551_m182000` | 拉锤，182000 轮 |
| RB+A | `rl_supermimic_lab3_zq_right_high_kick_20261102_j14030_m90000` | 原测试 21 号，90000 轮；旧 184000 轮版本在 `old/` |
| START+十字右 | `rl_supermimic_prone_j12768_m22000` | 当前俯卧起身，22000 轮 |
| START+十字上 | `rl_supine_to_stance_v1_hybrid_9843_m13000` | 当前仰卧起身，13000 轮 |

LT+B 的 `leo_taunt` 与 RB+B 的 SuperMimic 嘲讽是两种动作。前者只允许从 `walk_leo` 进入，8 秒后自动回 `walk_leo`。

## 朝向对齐如何工作

参考动作有自己的初始朝向，机器人启动动作时可能面向另一个方向。程序拟合一个绕世界 Z 轴的固定旋转，把参考朝向对齐到实际机器人，再计算策略需要的相对姿态。

当前 SuperMimic 与 dance 已共用 [heading_alignment.h](../src/runner/motion_transition/include/motion_transition/heading_alignment.h)：用完整初始姿态拟合水平旋转，避免分别提取 Euler yaw 时，趴卧状态下身体前轴近乎竖直造成的数值跳变。极端相对倾斜使拟合退化时，使用共同水平投影最强的机身轴；非有限姿态会被拒绝。

计算时机有所区别：

- 普通 SuperMimic 动作进入时先对齐；入口桥结束后清空历史和上一动作，以当时姿态重新对齐，从参考第 0 帧正式播放。
- 俯卧和仰卧起身先完成 PD 首姿态准备；这期间不推理策略、不填策略历史、不计算对齐，也不消耗参考帧。到位后的首次策略周期才对齐并开始动作。
- 普通 dance 动作在第一次组装策略观测时对齐。dance 的观测和初始对齐均使用 IMU 安装偏置修正后的机身坐标。

正式播放期间，对齐偏移保持固定，每周期仍使用实时机身姿态和对齐后的当前参考姿态计算误差，因此机器人真正的转身不会被逐帧重新对齐消除。该修改改善计算稳定性；IMU 估计或安装偏置不准确仍会影响策略观测，也不能仅凭对齐修复推断起身一定成功。

实现见 [SuperMimic runner](../src/runner/rl_supermimic/src/rl_supermimic_runner.cc) 和 [dance runner](../src/runner/rl_dance_example/src/rl_dance_example_runner.cc)。

## 进入与退出衔接

表中的时间是配置标称值。公共指令桥根据初始关节差异和速度、加速度参数估算时间，再限制到最小、最大时长之间；姿差太大时，最大时长截断不能保证所有动态上限仍满足。实现见 [entry_command_transition.cc](../src/runner/motion_transition/src/entry_command_transition.cc)。

| 动作类别 | 进入方式 | 完成或退出方式 |
|---|---|---|
| 四种走路 | 通常采用 0.16 秒指令桥，范围 0.10～0.28 秒；目标策略从首周期运行 | 持续运行，等待手动切换 |
| 四个正式 SuperMimic 动作及编号测试 | 通常采用 0.20 秒实时策略指令桥，范围 0.10～0.28 秒；期间参考停第 0 帧，策略闭环推理；无独立 PD 到位等待 | 参考播放完自动回 `walk_leo`，通常经过走路的 0.16 秒入口桥 |
| 五个普通 dance 拳脚动作 | 0.16 秒入口桥，范围 0.10～0.28 秒；参考停第 0 帧，策略闭环推理 | 参考播放完自动回 `walk_leo`，通常经过走路入口桥 |
| 俯卧、仰卧起身 | 先 PD 过渡到参考首姿态，再连续检查关节到位；满足后才启动策略 | 参考播放完请求回 `walk_leo`，走路直接接管 |
| `getup/getup2` | 姿态、角速度检查通过后运行起身策略，无上述参考首姿态 PD 等待 | 满足直立稳定判据后回 `walk_leo`，走路直接接管；超时则故障回阻尼 |
| 三种 PD 姿态 | 自身执行 3 秒五次关节插值 | 完成后继续 PD 控制，等待手动切换 |
| 主动躺下 | 自身 0.1 秒位置混合，同时推进策略和参考 | 完成后进入 `passive` |
| `idle/passive` | 直接切换控制状态 | `idle` 自动到 `passive`；`passive` 持续速度阻尼 |

普通 SuperMimic 实时指令桥的参考姿态引导权重为 0；普通 dance 入口配置为 0.35；走路入口配置为 0.25。这些用于入口指令构造，不是动作结束后的静态保持时间。

四个走路配置都包含直接接管来源：`getup`、`getup2`、`supine_to_stance`、`prone_to_stance`。从这些来源切入走路时跳过走路入口桥；能切到哪种走路还受动作图允许边约束。目前俯卧、仰卧只允许手动切 `passive`、`pd_stand`、`walk_leo`，自动目标也是 `walk_leo`。

### 俯卧与仰卧：当前相同的 PD 准备条件

俯卧仍由 SuperMimic runner 执行，仰卧仍由 dance runner 执行，但已使用同一套 [ReferencePoseGate](../src/runner/motion_transition/include/motion_transition/reference_pose_gate.h) 到位判定。当前两份 YAML 的覆盖值相同：

| 参数或步骤 | 当前值与含义 |
|---|---|
| `entry_transition_to_reference_pose` | `true`，先准备参考动作第 0 帧 |
| PD 入口桥时间 | 标称 0.16 秒，范围 0.10～0.28 秒，从实测当前关节姿态开始 |
| 入口动态参数 | 关节速度 8 rad/s、加速度 120 rad/s²；参考权重 0.35 |
| 到位误差 | 策略控制关节的最大实测位置误差 ≤0.80 rad，约 46° |
| 连续到位次数 | **桥完成后**连续 5 个 0.02 秒周期满足误差条件，约 0.1 秒；桥期间不计数，不满足则清零重计 |
| 策略启动 | 到位计数通过后的下一周期，从参考第 0 帧开始 |
| 手臂 PD 增益 | 准备阶段 Kp 乘 2.5，Kd 乘 sqrt(2.5)，策略阶段恢复策略增益 |

这里的 0.80 rad / 5 周期是当前 YAML 配置，不能与类默认值 0.12 rad / 8 周期混淆。当前门槛**没有身体方向、角速度、关节速度或足底接触检查，也没有等待超时**；未到位就继续 PD 等待，可以手动切出。

俯卧与仰卧结束后目前按参考进度请求走路，**不额外检查是否已经站稳**。入口 PD 准备与结束后是否采用走路混合，是两个独立配置问题。

配置见 [俯卧](../assets/config/t800/rl_supermimic_prone_j12768_m22000/default.yaml) 与 [仰卧](../assets/config/t800/rl_supine_to_stance_v1_hybrid_9843_m13000/default.yaml)。

### getup/getup2：按直立稳定判定结束

两套 HoST 起身使用相同 runner，目前配置条件相同：

- 进入：状态有效，机身坐标系中的重力投影 z > -0.8，角速度模长 ≤6 rad/s。直立时投影 z 约为 -1，因此过于直立会拒绝进入。
- 成功：重力投影 z ≤-0.9 且角速度模长 ≤2 rad/s，连续满足 0.3 秒，才请求自动回 `walk_leo`。这 0.3 秒内策略继续闭环控制。
- 超时：8 秒内未证明成功，进入故障回退到阻尼；超时不算成功，不自动冒充完成切走路。

实现见 [rl_getup_example_runner.cc](../src/runner/rl_getup_example/src/rl_getup_example_runner.cc)，配置见 [getup](../assets/config/t800/rl_getup_example/default.yaml) 和 [getup2](../assets/config/t800/rl_getup2_example/default.yaml)。

### 动作图允许的主要切换

- `passive` 可进入三种 PD 姿态和四种起身，不能直接进入走路或拳脚。
- 三种 PD 姿态可进入四种走路、四个正式 SuperMimic 动作、五个普通 dance 拳脚和四种起身。
- 四种走路可相互切换，均可进入正式拳脚、`getup/getup2` 和主动躺下。俯卧、仰卧入口需要先经 `passive` 或 PD；走路没有直接进入这两种参考起身的边。
- 正式拳脚完成后自动回 `walk_leo`；也可手动切到 `passive`、`pd_stand` 或四种走路。正式拳脚之间没有直接互切边。
- 编号测试仅从三种 PD 姿态或四种走路进入；完成后自动回 `walk_leo`，也可手动回任一上述准备状态或 `passive`。编号之间没有直接互切边。
- `leo_taunt` 仅与 `walk_leo` 往返；主动躺下完成后到 `passive`。故障通过任务的故障路径回到阻尼。

完整允许边以 `task_motion/default.yaml` 为准；编号扩展见 [numbered_actions.py](../tools/duel_sim/numbered_actions.py)。按键被识别、动作图允许切换、runner 入口检查通过，是不同步骤。

## 当前编号测试动作

清单：[catalog.json](../assets/config/t800/numbered_actions/catalog.json)。以下 5 个策略及清单随 Git 共享；每个目录只保留运行需要的 `default.yaml`、`policies/policy.mnn`、`trajectories/supermimic_reference.npz`。

| 当前编号 | 原编号 | 动作 | 批次 | 训练轮数 | 策略目录（位于 `assets/config/t800/numbered_actions/`） |
|---|---|---|---|---|---|
| 01 | 12 | 左勾拳 | 2 | 26000 | `rl_supermimic_s2bf_zq_left_hook_02_j13961_3_m26000` |
| 02 | 19 | 拉锤 | 3 | 56000 | `rl_supermimic_s2air25_zq_pull_hammer_20261102_j14016_4_m56000` |
| 03 | 28 | 左直拳 | 6 | 12000 | `rl_supermimic_s2air20_zq_left_straight_05_j14393_0_m12000` |
| 04 | 29 | 左抖拳中 | 6 | 8000 | `rl_supermimic_s2air20_zq_left_flick_mid_01_j14282_1_m8000` |
| 05 | 30 | 拉锤 | 6 | 22000 | `rl_supermimic_s2air20_zq_pull_hammer_20261102_j14282_2_m22000` |

原 21 号右高踢腿已成为正式 RB+A，其余未保留的测试策略已删除。五个编号动作均使用 SuperMimic 普通动作入口和自动回 LB+A 的走路桥；双虚拟手柄模式下，测试页只控制 A，B 使用普通虚拟手柄。新增动作和启动方式见[虚拟手柄 README](../tools/virtual_gamepad/README.md#shared-test-policies-and-git)及[启动说明](../tools/duel_sim/README.md)。

## 末尾静态保持与参考尾段

- 四个正式 SuperMimic 动作、俯卧起身及当前五个编号动作的 `hold_seconds` 均为 **0.0**，不额外停留 1 秒。
- 五个编号参考动作各有导出时附加的 100 帧静态尾段，均配置 `skip_terminal_hold_frames: 100`，部署播放时跳过，NPZ 文件本身不改写。
- 正式 RB+A 的 Lab3 踢腿没有这段附加尾部，配置 `skip_terminal_hold_frames: 0`。
- 当前普通 dance 拳脚和仰卧起身在参考结束时退出，不配置末尾继续静态保持。

额外保持、参考动作自带静态段、入口 PD 到位等待、起身成功稳定确认、走路指令混合分别作用于不同阶段。取消额外保持不会自动消除参考中原本存在的所有停顿，也不能保证策略本身平衡稳定。

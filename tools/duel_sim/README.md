# T800 双机器人仿真

一个 MuJoCo 场景包含两台 T800，两套独立 executor 各自运行原有状态机和策略。两台使用同一份模型、轨迹和动作配置，但观测、循环记忆、动作进度和手柄输入分别维护。双机入口不会修改 `assets/config/t800/mode.yaml`，原单机器人仿真和实机部署入口继续使用原配置。

## 启动

先在现有 SDK 开发环境中重新编译控制器和仿真：

```bash
./build.sh
./scripts/build_mujoco.sh
```

将两只 Xbox 或兼容 Linux joystick 的手柄接入同一台仿真计算机，运行：

```bash
./scripts/run_duel.sh --gamepad-a /dev/input/js0 --gamepad-b /dev/input/js1
```

无需再单独运行 `run.sh`。A 出生在左侧、朝向正 X，B 出生在右侧、朝向负 X；场景是平地，初始脚底约离地 5 mm，默认无吊绳。双机临时配置将 `LT+A` 的 PD 准备目标设为出生时的零关节角，保留原增益和插值时长；原单机和实机的 PD 配置不变。

双机窗口默认暂停。先让双方分别用各自手柄按 `LT+A` 进入 PD 准备，释放按键后，按窗口中的空格开始物理仿真；等双方实际站稳后再分别用 `LB+A` 切换到 Leo 步态。暂停期间控制器仍会计时，因此应在物理运行时切换策略或触发动作。摇杆控制自身前后、横移和转向，拳脚动作沿用原配置。完整键位以 `assets/config/t800/task_motion/default.yaml` 为准。

掉地后可分别使用原起身动作。GUI 的 Reset 只复位共享物理世界，不会同时清空两个 executor 的策略记忆和状态机；需要完整重开一局时退出并重新运行此入口。关闭窗口、任一子进程退出或按 `Ctrl+C` 会停止本次入口启动的全部子进程。

若设备支持，优先用稳定的设备链接避免拔插后 js0/js1 交换：

```bash
ls -l /dev/input/by-id/*-joystick
./scripts/run_duel.sh \
  --gamepad-a /dev/input/by-id/<手柄A>-joystick \
  --gamepad-b /dev/input/by-id/<手柄B>-joystick
```

路径必须指向 Linux joystick 设备，不能使用 `event-joystick`。不同型号、USB/蓝牙驱动可能报告不同的轴和按键布局；首次使用需确认摇杆、LT/RT 和动作组合的实际映射。两实例固定使用 gamepad 输入，不会由 RC02 或虚拟手柄覆盖。

## 配置与日志隔离

- 每次运行在系统临时目录下的 `engineai-duel-*` 生成双机 MJCF 及两份私有 assets 配置，退出后自动删除；`--keep-runtime` 可保留用于检查。
- 策略文件、轨迹、URDF 等共享只读源资源，仿真模式、PD 准备目标、通信、ROS 话题和日志配置分别写入临时副本。
- 两端默认使用 `239.255.76.67:17667` 和 `239.255.76.67:17668`，保持原 `sim_state` / `sim_command` 消息契约。可用 `--port-a`、`--port-b` 指定不同端口。
- 两控制器分别使用 ROS domain 70/71，并将话题加上 `/robot_a`、`/robot_b` 前缀；`--ros-domain-a N` 改为 N/N+1。
- 控制台及 SDK 日志分别写入独立目录。默认保存在系统临时目录下的 `engineai-duel-logs-*`，退出后保留；用 `--log-dir PATH` 指定保存位置。

## 检查与无界面运行

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

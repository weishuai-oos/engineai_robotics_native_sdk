# T800 单机与双机仿真启动

以下运行命令均在 SDK 容器内、仓库根目录执行。首次在新电脑创建容器，见[根目录 README 的容器环境说明](../../README_CN.md#11-容器环境)；已有容器直接按下面进入。

## 1. 进入环境

宿主机终端：

```bash
engineai_robotics_env
```

进入容器后（其他电脑改成实际仓库路径）：

```bash
cd /home/xjh/workspace/zq/engineai_robotics_native_sdk
```

确认 `assets/config/t800/mode.yaml` 为 `active_mode: sim`。首次使用或修改 C++ 后，在容器内编译；已有最新构建时跳过：

```bash
./build.sh
./scripts/build_mujoco.sh
```

虚拟手柄缺少依赖时，在容器内执行一次 `python3 -m pip install -r tools/virtual_gamepad/requirements.txt`。若编译提示缺少 `lcmConfig.cmake`，先确认当前已进入 SDK 容器。

## 2. 单机器人：任选一种

```bash
# 实体手柄
./scripts/run_single.sh --gamepad /dev/input/js0

# 虚拟手柄
./scripts/run_single.sh --virtual-gamepad
```

单机默认有吊绳辅助，观察自由站立或抗击打时，在 MuJoCo 窗口按数字键 `9` 关闭，控制台会显示 `Elastic band disabled`。

## 3. 双机器人：任选一种

```bash
# 两个实体手柄
./scripts/run_duel.sh --gamepad-a /dev/input/js0 --gamepad-b /dev/input/js1

# A 虚拟手柄，B 实体手柄
./scripts/run_duel.sh --virtual-gamepad --gamepad-b /dev/input/js0

# A、B 各一个虚拟手柄
./scripts/run_duel.sh --virtual-gamepads
```

双机默认无吊绳。两个虚拟窗口分别控制 A、B；编号策略测试仅在 A 上启用。

## 4. 开始控制与退出

1. 虚拟手柄先勾选 **Enable** 或按 **F5**；双窗口分别启用。
2. 用 `LT+A` 进入 PD 准备。双机窗口默认暂停：双方进入 PD 后释放按键，在 MuJoCo 窗口按空格开始物理仿真。
3. 等机器人站稳后，用 `LB+A` 进入 CombinedV2 走路，再使用摇杆或虚拟手柄控制移动、转向和动作。策略切换应在物理仿真运行时进行。
4. 测试页从 PD 或走路状态点击编号动作，完成后自动回到 `LB+A` 走路；下一项需手动点击。

关闭仿真或虚拟手柄窗口，或在启动终端按 `Ctrl+C`，会结束本次启动的全部进程。切换单机、双机前先正常退出上一会话。

动作映射和策略选择在 `assets/config/t800/`，修改 YAML 后重启会话。当前 5 个测试策略与 `catalog.json` 位于 `assets/config/t800/numbered_actions/`，随 Git 一起共享；编号和模型见[动作分类与衔接说明](../../docs/t800_runner_classification_and_transitions.md#当前编号测试动作)。[添加编号动作的方法](../virtual_gamepad/README.md#shared-test-policies-and-git)。

## 5. 如果出现 LCM self test failed

在**宿主机**检查路由；109 的本机仿真若被 VPN/PPP 路由影响，执行：

```bash
ip route get 239.255.76.67
sudo ip route replace 239.255.76.67/32 dev lo
```

这是临时本机路由，容器重启通常保留，宿主机重启后需重新设置。使用同一组播地址进行跨电脑或实机通信前，在宿主机撤销：

```bash
sudo ip route del 239.255.76.67/32 dev lo
```

## 模型导出与替换（AnkleCombinedV2）

当前 `LB+A` 已使用最终 **`ckpt_140000.pt`** 导出的模型，日常启动无需导出。以下仅用于重新导出这一个 checkpoint，在**本机训练环境**执行，先退出正在使用该模型的 SDK。路径按实际环境修改：

```bash
(
set -e
LEO_DIR=/srv/shared/home/xjh/workspace/zq_leo_final
SDK_DIR=/srv/shared/home/xjh/workspace/zq/engineai_robotics_native_sdk
LEO_PY=/srv/shared/home/xjh/app/miniconda3/envs/leo/bin/python
MNN_PY=/srv/shared/home/xjh/app/miniconda3/envs/whole_body_tracking/bin/python
CKPT_PATH="$LEO_DIR/logs/cusrl/T800_base-AnkleCombinedV2/2026-11-07-23-48-06:T800_base-AnkleCombinedV2_finetune_2gpu_seed42/ckpt/ckpt_140000.pt"
DEPLOY_DIR="$SDK_DIR/assets/config/t800/rl_walking_leolab_ankle_combined_v2_140000/policies"
OUT_DIR=$(mktemp -d /tmp/engineai-ankle-v2.XXXXXX)
trap 'rm -rf -- "$OUT_DIR"' EXIT

"$LEO_PY" "$LEO_DIR/scripts/cusrl/export_ankle_v2.py" \
  --checkpoint "$CKPT_PATH" --expected-iteration 140000 --output-dir "$OUT_DIR"

"$MNN_PY" -m MNN.tools.mnnconvert \
  -f ONNX --modelFile "$OUT_DIR/actor.onnx" \
  --MNNModel "$OUT_DIR/boxing_policy.mnn" --bizCode MNN --targetVersion 2.9

cp "$OUT_DIR/boxing_policy.mnn" "$DEPLOY_DIR/boxing_policy.mnn"
)
```

只把最终 MNN 放入部署目录，ONNX 和导出记录留在临时目录并在结束时自动清理。转换保持 FP32，`--targetVersion 2.9` 对应 SDK 容器的 MNN 2.9 推理库。

`assets/config/t800/mode.yaml` 的 `mode.sim`、`mode.robot` 均应包含以下选择，当前已配置：

```yaml
    - tag: rl_walking_leolab_example
      scope: rl_walking_leolab_ankle_combined_v2_140000/default
```

运行只需该目录的 `default.yaml` 与 `policies/boxing_policy.mnn`；保持四踝 Kd=2.0。接口为 84 维观测、25 维动作及两组 `[2,1,256]` 循环状态。以上命令只更新 `SDK_DIR` 指定的仓库；在 109 测试时，需同步对应 MNN 后重启单机或双机会话。

该导出脚本仅适用于 CombinedV2。`LB+Y` 的 ImpactV2 需要其带头部掩码的专用导出流程。

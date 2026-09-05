# T800 RC02 初始化握手修复

核查日期：2026-09-05。对象：当前 Native SDK 与机载默认程序的本地副本
`../apps/engineai_robotics`。未运行机载程序或连接机器人。

## 核查结论

SDK 已有 RC02 串口驱动、输入解析和逻辑按键映射，但原来的
`Rc02InputAdapter::Rc02Init()` 仅调用 `Init()` 和
`GetRc02HardwareVersion()`；没有调用已经实现的 `SendInitData()`。
因此串口打开、版本查询成功，并不代表完成了遥控器初始化。

默认程序把这条通道命名为 LoRa：

- `libsrc_runner_input_command_arbiter.so` 中存在 `LoraInputAdapter`。
- `libsrc_hardware_lora.so` 中存在 `LORADriver::BuildInitData`、
  `SendInitData`、`BuildLoraInitFrame`、`ParseLoraInitAck`。
- 反汇编确认 `0x0D` 初始化帧及 500 ms 的 ACK 等待。
- 资源 `assets/resource/rc02/robot_model_map.yaml` 给出 T800 Product = 4。

此前根据不存在 `libsrc_hardware_rc02.so` 推测缺少官方驱动并不充分；
默认程序通过 `libsrc_hardware_lora.so` 实现该功能。

外部排查报告的日志摘录与这条代码证据吻合，但所拷贝目录未包含这些
`/tmp/logs` 原始日志，因此没有把报告中的真机现象当作本次测试结果。

报告的版本来源需要区分：官方 Protocol 取自
`motion_task.rc_protocol_version`（缺失默认 1.0.1）；Motion 取自
`ENGINEAI_ROBOTICS_VERSION`，Hardware 当前复制 Motion。
官方启动脚本设置 `1.0.3+hotfix.v2`，报告记录 Protocol 同为 1.0.3。
菜单 JSON 的 `schema_version=1.0.3`、`fw_version=1.0.2` 是不同字段，
不能把后者直接填进初始化帧。

## 本次修改

`RC02Driver::Connect()` 执行完整顺序：

1. 打开串口（默认 `/dev/ttyJS`，2000000 baud）。
2. 发送 `AA 01 04 FA FF`，解析硬件版本。
3. 发送 55 字节初始化帧 `AA 33 0D ...`。
4. 在 500 ms 内收到并校验 `AA 01 0D F1 FF` 后，才报告初始化成功。

首次连接和重连均走这一入口。失败关闭串口，下一次重试重新查询和握手。
版本查询和 ACK 等待在后台任务执行，输入线程仅检查任务是否完成，不等待串口握手，
使虚拟手柄的停止／松键及输入过期处理继续运行。退出时先等待后台任务结束再释放驱动。
适配器仍保留最多 0.3 秒输入；过期清空按键和摇杆，连续 1 秒无有效输入帧时
关闭串口以触发重新握手，避免把“串口仍打开”一直当作无线连接有效。

输入仲裁器初始化并轮询所有硬件输入源；优先使用有有效输入的 RC02，其次是 F710，
虚拟手柄继续作为覆盖输入。RC02 首次握手失败后仍可在后台恢复，不会因 F710 被选中
而永久停止重试。每个适配器分别发布 `hardware/rc02_info`、`hardware/f710_info`；
原有 `hardware/gamepad_info` 由仲裁器只发布选中的硬件快照，避免备用源的断开状态
覆盖正在使用的输入。

T800 专用配置为 `assets/config/t800/rc02/default.yaml`，通过现有参数读取器
固定读取当前所选产品下的 `rc02/default` scope。它不需要 mode.yaml 的 tag 映射，
robot/sim 选择同一产品时均可读到。`install.sh` 会随整个 T800 配置目录复制它。
其他产品如需使用 RC02，需提供各自的 `rc02/default.yaml`，不能复用 Product=4。

配置中的 Protocol/Motion/Hardware 兼容字段使用已记录的官方握手值 1.0.3；
没有修改全局 `ENGINEAI_ROBOTICS_VERSION`，不会把 Native SDK 标记成官方固件版本。
无线硬件版本始终取串口查询结果；例如返回 R009 时，发送帧应为：

```text
AA 33 0D 04 03 00 01 03 00 01 03 00 01 09 [39 个 00] A6 FF
```

协议中命名为 CRC 的两字节实际为：长度字节、命令和数据求和后按 16 位取反，
不包含 AA 帧头。

## 按键与屏幕菜单的边界

实体按键仍通过 `rc02_input_adapter.cc` 转成 `GamepadInfo`，再由
`assets/config/t800/task_motion/default.yaml` 的 `key` 和状态转换规则处理。
握手恢复后，真机也能使用这里的自定义组合；仿真键盘的 J/K/Q/E 等物理键位
属于虚拟手柄界面配置，不会直接重定义 RC02 实体按钮。

此次没有复制官方动作菜单或接入 MotionCommand/SkillCommand：SDK 自定义动作
与官方动作 ID 并不相同，复制 JSON 本身不会实现对应状态切换。
周期状态帧仍沿用原有 `SendMotionAndACK(0, 0)`，屏幕当前动作／错误详情
尚未接入 SDK 状态机。恢复握手和实体按键不等于完整移植官方屏幕菜单。

## 验证与真机验收

新增 `src/hardware/rc02/test_rc02_connection.cc`：使用本机伪串口和真实串口驱动，
覆盖完整握手、帧字节、分片 ACK、损坏／丢失 ACK、再次连接及版本配置。
现有 RC02 CMake 测试目标会自动包含该文件，SDK 开发环境中可执行：

```bash
./build.sh -T -a x86_64 -r humble
./build/x86_64/src/hardware/rc02/test_src_hardware_rc02
```

真机部署仍使用项目原有流程，重新编译 aarch64 并部署库与配置：

```bash
./build.sh -a aarch64 -r humble -t release
./install.sh t800 robot aarch64
```

启动 SDK 后应依次看到 `RC02 version rx`、`RC02 init frame (55B)`、
`RC02 init ACK received`、`initialized successfully (init ACK confirmed)`。
随后确认屏幕连接状态恢复，以及实体按键按当前 YAML 切换状态。
若版本查询成功但出现 `init ACK timeout`，需保留完整收发十六进制日志继续比对，
不能仅凭伪串口测试宣称真机已恢复。

本次环境没有 Docker、ROS 和完整的主机构建依赖，根项目 CMake 配置停在缺少 glog。
伪串口测试采用独立编译，连接 SDK 自带的 x86_64 `libsrc_hardware_common.so`，
临时 GoogleTest/glog 验证依赖放在 `/tmp`；未安装到系统或新增项目依赖。
输入适配器和仲裁器以所拷贝 SDK 依赖的真实头文件做 C++20 语法检查。
本次执行结果：7 项伪串口测试全部通过；修改过的驱动、输入适配器、仲裁器通过
C++20 语法检查；T800 YAML 可正确解析，sim/robot 均指向同一动作按键配置；
`git diff --check` 通过。测试还验证了原有数字 LT 输入语义没有被改成模拟扳机值。
完整 aarch64 链接、后台重连的真机时序和屏幕连接状态仍需开发容器／机器人验收。

# 嘲讽备用动作

当前 `RB+B` 使用 `t800/rl_supermimic_zq_come_here_01_j13904_0_m36000`，
单机、双机以及 `mode.robot` 的选择一致，额外保持为 `hold_seconds: 0.0`。

以下两个包完整移动到本目录留作备用，文件内容保持原样：

| 版本 | 目录 |
| --- | --- |
| 第二个 | `rl_supermimic_zq_come_here_02_j13904_1_m36000` |
| 第三个 | `rl_supermimic_zq_come_here_03_j13904_2_m38000` |

每包包含模型、对应参考轨迹、配置、导出来源和校验数据。
父目录 `old/manifest.json` 记录全部归档文件的 SHA256；当前 mode 不引用这两个备用包。

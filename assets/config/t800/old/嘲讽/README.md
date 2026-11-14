# 嘲讽备用动作

原 `RB+B` 的 `rl_supermimic_zq_come_here_01_j13904_0_m36000` 已归档到 `old/11.14/`；当前 `RB+B` 使用新导入的 18 号右高弹踢。这里继续保留另外两版嘲讽，额外保持均为 `hold_seconds: 0.0`。

| 版本 | 备用目录 |
| --- | --- |
| 第二个 | `rl_supermimic_zq_come_here_02_j13904_1_m36000` |
| 第三个 | `rl_supermimic_zq_come_here_03_j13904_2_m38000` |

每包仅保留 `default.yaml`、`policies/policy.mnn` 和 `trajectories/supermimic_reference.npz`；当前 mode 不引用它们。

恢复前退出 SDK，将所选目录移到 `assets/config/t800/` 根目录，根据 [11.14 归档](../11.14/README.md) 中原嘲讽的记录恢复 `rl_supermimic_come_here` tag、`supermimic_come_here` 任务及进入边；将两个模式的 scope 指向 `所选目录名/default`，绑定空余按键并同步操作说明和普通虚拟手柄宏。继续使用 `rl_supermimic_runner`，重启会话即可，无需重新编译。

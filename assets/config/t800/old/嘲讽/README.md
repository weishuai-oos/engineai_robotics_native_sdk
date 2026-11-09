# 嘲讽备用动作

当前 `RB+B` 使用 T800 根目录的 `rl_supermimic_zq_come_here_01_j13904_0_m36000`，单机、双机及 `mode.robot` 的选择一致，额外保持为 `hold_seconds: 0.0`。

| 版本 | 备用目录 |
| --- | --- |
| 第二个 | `rl_supermimic_zq_come_here_02_j13904_1_m36000` |
| 第三个 | `rl_supermimic_zq_come_here_03_j13904_2_m38000` |

每包仅保留 `default.yaml`、`policies/policy.mnn` 和 `trajectories/supermimic_reference.npz`；当前 mode 不引用它们。

切换前退出 SDK，将所选目录移到 `assets/config/t800/` 根目录，再将 `mode.yaml` 中 `mode.sim`、`mode.robot` 的 `rl_supermimic_come_here` tag 的 scope 改为 `所选目录名/default`，重启单机或双机入口。继续使用 `rl_supermimic_runner` 和 RB+B 按键，无需重新编译。

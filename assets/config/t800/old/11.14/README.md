# 11.14 实体遥控器动作归档

本目录保存更换按键映射前的 9 个完整动作包。配置、MNN 和参考轨迹原样保留，未被当前 `mode.robot`、`mode.sim` 或任务图使用；原虚拟测试 01～05 清单位于 `numbered_actions/catalog.json`。

| 原实体按键 | 原状态 | 相对本目录的动作包 |
| --- | --- | --- |
| RB+B | `supermimic_come_here` | `rl_supermimic_zq_come_here_01_j13904_0_m36000` |
| RB+Y | `supermimic_left_hook` | `rl_supermimic_zq_left_hook_02_j13564_m82000` |
| RB+X | `supermimic_pull_hammer` | `rl_supermimic_zq_pull_hammer_20261102_j13551_m182000` |
| RB+A | `supermimic_right_high_kick` | `rl_supermimic_lab3_zq_right_high_kick_20261102_j14030_m90000` |
| RB+上 | `supermimic_left_hook_26000` | `numbered_actions/rl_supermimic_s2bf_zq_left_hook_02_j13961_3_m26000` |
| RB+下 | `supermimic_left_straight_12000` | `numbered_actions/rl_supermimic_s2air20_zq_left_straight_05_j14393_0_m12000` |
| RB+左 | `supermimic_left_flick_mid_8000` | `numbered_actions/rl_supermimic_s2air20_zq_left_flick_mid_01_j14282_1_m8000` |
| RT+右 | `supermimic_pull_hammer_56000` | `numbered_actions/rl_supermimic_s2air25_zq_pull_hammer_20261102_j14016_4_m56000` |
| RB+右 | `supermimic_pull_hammer_22000` | `numbered_actions/rl_supermimic_s2air20_zq_pull_hammer_20261102_j14282_2_m22000` |

`manifest.json` 记录迁移前提交、原任务定义、两种模式的原 tag/scope、全部归档资源的 SHA-256，以及本次从 `ds@192.168.20.249:/home/ds/robot/engineai_robotics_native_sdk` 导入的 13 个动作、原编号、新按键和远端资源校验值。新旧资源均已逐文件校验。

恢复某个动作前先退出 SDK，将其完整目录按上表的相对路径移回 `assets/config/t800/`。根据 manifest 的 `archived_registrations` 恢复两种模式的 tag/scope，根据 `archived_tasks` 恢复该任务与三种 PD、四种走路的入口边；先释放新动作占用的按键，并同步普通虚拟手柄快捷宏与 `操作.txt`。旧动作与当前动作均使用 SuperMimic runner。

当前编号测试清单仅保留新动作 06、08、09、11～20。若恢复旧编号测试，将所需的旧包移回 `numbered_actions/` 后，把对应条目合入当前 catalog；保留现有新动作条目和原编号。

# T800 备用动作

本目录仅保存可运行备选的配置、模型和必要参考轨迹，当前 `mode.sim`、`mode.robot` 均未选择它们。

| 备选 | 目录 |
| --- | --- |
| 旧 dance | `rl_dance_example` |
| 旧 victory | `rl_victory_example` |
| 旧 ridicule | `rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500` |
| 更早的 WBT 俯卧起身 | `rl_prone_to_stance_hybrid_9825_m29500` |
| 原 LB+A 步态 | `rl_walking_leolab_example` |
| 第二、第三个重训嘲讽 | `嘲讽/`，见该目录的 `README.md` |

当前俯卧起身使用根目录的 `rl_supermimic_prone_j12768_m22000`；RB+B 使用第一个重训嘲讽。空庆祝模板、历史模型副本、整套配置备份和评测数据已清理。

## 旧 Leo 走路

先退出 SDK，将 `old/rl_walking_leolab_example` 移回 `assets/config/t800/` 根目录，再把 `mode.yaml` 的 `mode.robot`、`mode.sim` 中 `rl_walking_leolab_example` tag 的 scope 改为 `rl_walking_leolab_example/default`，然后重启。

该备选只保留 `default.yaml` 及它引用的 `2026-09-06-19-30-18:t800_lowcylinders_ankle_stable_1_seed42/boxing_policy.mnn`。不要直接用 `old/rl_walking_leolab_example/default` scope：配置内部仍按 T800 根目录的相对路径查找模型。LT+B 的 `leo_taunt` 直接引用 terrain 模型，不依赖这里的备选。

## 恢复旧 WBT 动作

先退出 SDK，将所选动作目录移回 T800 根目录，使 YAML 内的资源相对路径有效。再在 `mode.yaml` 的相应模式中选择它的 `目录名/default` scope，并在 `task_motion/default.yaml` 为目标状态选择对应参数 tag 和 `rl_dance_example_runner`，核对进入、退出与完成切换关系后重启。当前四个 RB 动作和俯卧起身使用 SuperMimic runner，恢复 WBT 备选时不能只换 scope。

不再提供整套 mode/task 配置覆盖备份，恢复单个备选时保留其他状态和按键配置。第二、第三个嘲讽仍使用 SuperMimic runner，切换方法见 [嘲讽备用动作](嘲讽/README.md)。

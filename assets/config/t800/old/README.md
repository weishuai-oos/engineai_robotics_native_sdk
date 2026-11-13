# T800 备用动作

本目录仅保存可运行备选的配置、模型和必要参考轨迹，当前 `mode.sim`、`mode.robot` 均未选择它们。

| 备选 | 目录 |
| --- | --- |
| 旧 dance | `rl_dance_example` |
| 旧 victory | `rl_victory_example` |
| 旧 ridicule | `rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500` |
| 更早的 WBT 俯卧起身 | `rl_prone_to_stance_hybrid_9825_m29500` |
| 原 LB+A 步态 | `rl_walking_leolab_example` |
| 原 RB+A 右高踢腿（184000 轮） | `rl_supermimic_zq_right_high_kick_20261102_j13551_m184000` |
| 第二、第三个重训嘲讽 | `嘲讽/`，见该目录的 `README.md` |
| 上次比赛的五个 WBT 拳脚动作 | `上次比赛/`：左前踢、回旋踢、左直拳、左勾拳和示例拳 |

当前俯卧起身使用根目录的 `rl_supermimic_prone_j12768_m22000`；RB+B 使用第一个重训嘲讽。空庆祝模板、历史模型副本、整套配置备份和评测数据已清理。

RB+A 当前使用根目录的 `rl_supermimic_lab3_zq_right_high_kick_20261102_j14030_m90000`（原虚拟测试 21 号、90000 轮）。本目录中的旧右高踢保留原配置、MNN 和参考；恢复时先移回 T800 根目录，再把 `mode.yaml` 两个模式的 `rl_supermimic_right_high_kick` scope 改回旧目录的 `/default`，重启会话即可，runner 与按键不变。

## 旧 Leo 走路

先退出 SDK，将 `old/rl_walking_leolab_example` 移回 `assets/config/t800/` 根目录，再把 `mode.yaml` 的 `mode.robot`、`mode.sim` 中 `rl_walking_leolab_example` tag 的 scope 改为 `rl_walking_leolab_example/default`，然后重启。

该备选只保留 `default.yaml` 及它引用的 `2026-09-06-19-30-18:t800_lowcylinders_ankle_stable_1_seed42/boxing_policy.mnn`。不要直接用 `old/rl_walking_leolab_example/default` scope：配置内部仍按 T800 根目录的相对路径查找模型。LT+B 的 `leo_taunt` 直接引用 terrain 模型，不依赖这里的备选。

## 恢复旧 WBT 动作

先退出 SDK，将所选动作目录移回 T800 根目录，使 YAML 内的资源相对路径有效。再在 `mode.yaml` 的相应模式中选择它的 `目录名/default` scope，并在 `task_motion/default.yaml` 为目标状态选择对应参数 tag 和 `rl_dance_example_runner`，核对进入、退出与完成切换关系后重启。当前四个 RB 动作和俯卧起身使用 SuperMimic runner，恢复 WBT 备选时不能只换 scope。

不再提供整套 mode/task 配置覆盖备份，恢复单个备选时保留其他状态和按键配置。第二、第三个嘲讽仍使用 SuperMimic runner，切换方法见 [嘲讽备用动作](嘲讽/README.md)。

`上次比赛/` 的五个目录保留各自原始配置和运行资源。恢复其中一个时，将该目录移回 T800 配置根目录；在 `mode.robot` 和 `mode.sim` 恢复对应 tag/scope，并将动作图中的对应状态和所有入口边改回原状态名、`rl_dance_example_runner` 和参数 tag。旧 YAML 的模型、轨迹路径相对 T800 配置根目录，不能只把 scope 指向 `old/上次比赛/`。

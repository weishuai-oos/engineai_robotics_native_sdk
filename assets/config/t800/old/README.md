# T800 备用动作

以下五个目录由新 SuperMimic 动作替换后，完整移到这里，文件内容保持原样：

本次另将旧的 `rl_walking_leolab_example`（LB+A 原步态及其历史模型）完整归档到
`old/rl_walking_leolab_example`。早期保留的根目录同名兼容软链接现已移除；
新的 `mode.robot` 和 `mode.sim` 已改为
`rl_walking_leolab_ankle_combined_v2_140000/default`，正常新启动会使用
AnkleCombinedV2 最终 `ckpt_140000` 模型。

| 原状态 / 按键 | 目录 |
| --- | --- |
| dance / RB+B | `rl_dance_example` |
| victory / RB+Y | `rl_victory_example` |
| ridicule / RB+X | `rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500` |
| celebration / RB+A | `rl_celebration`，模型和轨迹尚未提供的预留模板 |
| prone_to_stance / START+十字键右 | `rl_prone_to_stance_hybrid_9825_m29500` |

前三个归档前仍在使用，第四个没有进入路径；俯卧起身当前使用 `rl_supermimic_prone_j12768_m22000`，状态名和按键保留。测试和输入配置也继续保留。

`RB+B` 当前使用第一个重训嘲讽 `rl_supermimic_zq_come_here_01_j13904_0_m36000`。
第二、第三个包完整归档到 `嘲讽/` 子目录，模型、轨迹和配置均保持原样，
同样由下述 `manifest.json` 记录文件 SHA256；详情见 `嘲讽/README.md`。

`mode.before_supermimic.yaml.backup` 和 `task_motion.before_supermimic.yaml.backup` 是四个 RB 动作替换前的模式及按键图，也仍引用旧俯卧起身包，并非仅回退本次俯卧起身的备份。`manifest.json` 按原 T800 根目录的相对路径记录归档文件的 SHA256。

旧动作的 YAML 内部资源路径仍相对 `t800` 根目录。原四个 RB 动作的兼容软链接现已按 109 当前状态移除；动作数据只在 `old` 中保存一份。

恢复整套旧配置时，先退出 SDK，将四个 RB 动作目录和旧俯卧起身目录共五个目录移回 `t800` 根目录，再把两份 `.backup` 分别恢复为 `t800/mode.yaml` 和 `t800/task_motion/default.yaml`。只移回四个 RB 动作会缺少备份中引用的旧俯卧起身包。不要直接用 `old/.../default` scope 启动原样归档的配置。

SDK 仍会读取 `old` 内合法的 YAML，但当前 mode 和动作图均未引用它们，因此不会执行这些备用动作。

旧 walk 目录中的历史子目录和模型只是备用文件，不能当作当前 LB+A 配置。需要回退旧步态时，先退出 SDK，
将 `old/rl_walking_leolab_example` 移回根目录，
再把 `mode.robot` 和 `mode.sim` 中 `rl_walking_leolab_example` 的 scope 改回
`rl_walking_leolab_example/default`，然后重启程序。不要直接把 scope 指向
`old/rl_walking_leolab_example/default`；归档 YAML 内部仍按原根目录相对路径查找模型。

`rl_walking_leolab_taunt/default.yaml` 已改为直接引用 terrain 当前模型，因为两份旧模型
SHA256 相同；这样归档旧 walk 后，LT+B 嘲讽仍不依赖归档目录。新 CombinedV2 与
`walk_leo_terrain` 使用相同遥控参数，但四个踝关节 Kd 为 2.0，terrain 为 0.3。

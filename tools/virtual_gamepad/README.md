# Virtual Gamepad

PyQt6-based visualization and debugging tools for the virtual gamepad workflow. The current toolchain provides:

- LCM connection management
- a virtual gamepad publisher
- a lightweight message diagnostic script

## Dependencies

- Python 3.7+
- PyQt6
- lcm
- A Chinese font in the environment running the GUI (included in newly built SDK containers)

In the SDK container, install the required dependencies with:
```bash
pip3 install -r tools/virtual_gamepad/requirements.txt
```

## Usage

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py
```

The window connects manually by default. To select an address and connect when
the window starts, use:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=0' --connect
```

Automatic connection does not enable publishing. Press F5 or enable the
`Enable` checkbox before sending virtual commands.

For two virtual controllers in the shared duel scene, run:

```bash
./scripts/run_duel.sh --virtual-gamepads
```

The launcher opens separate **Robot A** and **Robot B** windows on their
respective LCM ports. Enable each window separately; keyboard shortcuts act
on the focused window. Robot A keeps numbered policy tests; Robot B uses the
ordinary gamepad controls and has no executable numbered tests. Both windows
share the same program, with separate processes and logs. Closing either
window ends the session. The existing `--virtual-gamepad` (singular) option
still starts one virtual controller for A and a physical controller for B.

## Numbered policy tests

Start the SDK with `./scripts/run_single.sh --virtual-gamepad` or
`./scripts/run_duel.sh --virtual-gamepad`. The launcher registers the policy
catalog in a private simulation configuration and opens the **策略测试** tab.
In a duel, this controls robot A; robot B keeps its existing controller.

Enable sending, enter PD standing or walking, then click a numbered action.
Buttons show the action, batch, and checkpoint iteration in three columns.
The shared catalog currently contains 15 actions: 01–12 and 14–16; number 13
is not included. It supports numbers 01–50; unconfigured numbers are hidden.
Robot feedback confirms the active action.
After confirmation the next item is selected, but execution always requires
another click. Walking, PD standing, and passive recovery buttons remain
available during testing. F5/Enable is shared with the original gamepad tab.

The imported policies use zero extra hold time. Actions 01, 03, 04, and 06–12
skip their appended 100-frame static tail; 02, 05, and 14–16 have no appended
tail and skip zero frames. They share the regular SuperMimic entry bridge and automatically
return through the existing gait bridge to `walk_leo` (LB+A). Restart the
simulation session after changing the catalog or mappings. The import preserves
the source models, YAML settings, and reference/goal arrays.

Chinese text needs a CJK font in the GUI environment. If it renders as square
glyphs, follow the Chinese font setup below.

Execution requires connected, enabled input and fresh robot state. Commands
are cancelled on disconnect, disable, stale feedback, or confirmation timeout;
reconnection never replays a pending action. `--test-catalog` is supplied by
the launcher, so a standalone window without that option cannot execute
numbered actions. See the shared catalog setup below; simulation startup
commands are in [the startup guide](../../操作.txt).

### Shared test policies and Git

`assets/config/t800/numbered_actions/` contains the shared `catalog.json` and
15 policy packages imported from `kyx@10.12.120.125`, each with its YAML, MNN,
and reference motion. Source IDs 01–12 and 14–16 are preserved; 13 is excluded.
These policies also drive the regular physical gamepad:

| Physical combination | Numbered action |
| --- | --- |
| LB+d-pad up | 01 Left hook, stage 2 (12000 iterations) |
| LB+d-pad down | 02 Left hook, stage 1, ankle PD 40/2 (10000 iterations) |
| LB+d-pad left | 03 Left hook, unstable plateau (6000 iterations) |
| LB+d-pad right | 04 Left hook, stage 2, ankle PD 40/2 (8000 iterations) |
| RB+d-pad up | 05 Low left flick, stage 1, ankle PD 40/2 (10000 iterations) |
| RB+d-pad down | 06 Low left flick, stage 2 (6000 iterations) |
| RB+d-pad left | 07 Low left flick, stage 2, ankle PD 40/2 (8000 iterations) |
| RB+d-pad right | 08 High left flick, stage 2 (6000 iterations) |
| RB+A | 09 Left pull-down hammer, stage 2 (12000 iterations) |
| RB+B | 10 Low left roundhouse, stage 2 (16000 iterations) |
| RB+X | 11 Mid left roundhouse, stage 2 (16000 iterations) |
| RB+Y | 12 Left front push kick, unstable latest (26000 iterations) |
| RT+d-pad up | 14 Taunt 01 with head shake (36000 iterations) |
| RT+d-pad left | 15 Taunt 02 with head shake (36000 iterations) |
| RT+d-pad right | 16 Taunt 03 with head shake (36000 iterations) |

RT+d-pad up is supported by the updated input encoding. Rebuild the controller
and restart the session after this input change. The formal
`supermimic_action_NN` states and virtual `test_action_NN` states use the same
package files but keep separate command names. Details are listed in the
[runner and transition reference](../../docs/t800_runner_classification_and_transitions.md#当前编号测试动作).

To share these tests, include the catalog and all corresponding runtime files
in Git. For a checkout without a catalog, initialize an empty one from the SDK
root:

```bash
mkdir -p assets/config/t800/numbered_actions
cp -n tools/virtual_gamepad/catalog.example.json assets/config/t800/numbered_actions/catalog.json
```

An empty catalog is supported, and `cp -n` preserves an existing catalog.
Each package needs only these runtime files:

```text
assets/config/t800/numbered_actions/
├── catalog.json
└── rl_supermimic_my_punch/
    ├── default.yaml
    ├── policies/policy.mnn
    └── trajectories/supermimic_reference.npz
```

Add entries to `actions`, or populate an empty catalog, for example:

```json
{
  "schema_version": 1,
  "capacity": 50,
  "actions": [
    {"id": 1, "label": "左直拳", "batch": 1, "iteration": 12000,
     "package": "rl_supermimic_my_punch"}
  ]
}
```

Use unique integer IDs from 1 to 50; gaps are allowed. `batch` is a positive
integer, and `iteration` is a nonnegative integer. The package name must match
its directory, start with `rl_supermimic_`, and contain only ASCII letters,
digits, and underscores. Configuration, model, and reference must match the
current SuperMimic runner; an arbitrary ONNX or another runner's YAML is not
directly supported. Missing runtime files and malformed catalogs are rejected.

Keep `hold_seconds: 0.0` to remove extra end holding. If the reference includes
an appended static tail, set `skip_terminal_hold_frames` to its actual frame
count; otherwise use 0. Do not assume every model has a 100-frame tail.
Restart the session to load changes. Adding a numbered test alone does not
change formal button mappings; those are defined separately in `task_motion`
and registered in both `mode.sim` and `mode.robot`.
To share additions, commit the catalog together with the corresponding three
runtime files. This directory is no longer automatically ignored, so review
the staged files before committing. Keep training checkpoints, ONNX exports,
evaluation reports, and videos outside the runtime packages.

### Chinese font setup

If Chinese labels appear as boxes while English and numbers look normal,
check fonts **inside the environment running the virtual gamepad**. Installing
fonts only on the host does not make them available inside a container.
Source files and `catalog.json` use UTF-8; changing the locale does not add
missing font glyphs.

New SDK container builds install `fonts-noto-cjk` and `fontconfig`. An existing
container is not updated by `git pull`; repair it once inside that container:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends fonts-noto-cjk fontconfig
fc-cache -f
fc-list :lang=zh family
```

Close and reopen the virtual gamepad afterwards. A simple container restart
keeps the installed fonts; a recreated container needs the updated image or
the installation above again. The GUI selects an available Chinese font at
startup and prints an English repair message when none is installed.

Available widgets/components:

1. `LcmManagerWidget`: connect and disconnect from the target LCM URL.
2. `VirtualGamepadWidget`: publish virtual gamepad messages for debugging.
3. `diagnose_gamepad.py`: basic LCM/message diagnostics for the virtual gamepad channel.

## License

This directory is licensed under **GNU General Public License v3.0 or later** (`GPL-3.0-or-later`).
See `LICENSE` for the full license text.

This subdirectory-specific license applies to the virtual gamepad tool itself. Other parts of the repository remain under the repository root license (`LICENSE.txt`) unless explicitly stated otherwise.

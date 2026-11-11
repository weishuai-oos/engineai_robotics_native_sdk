# Virtual Gamepad

PyQt6-based visualization and debugging tools for the virtual gamepad workflow. The current toolchain provides:

- LCM connection management
- a virtual gamepad publisher
- a lightweight message diagnostic script

## Dependencies

- Python 3.7+
- PyQt6
- lcm

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
The shared catalog currently contains actions 01–05 and supports numbers
01–50; unconfigured numbers are hidden. Robot feedback confirms the active action.
After confirmation the next item is selected, but execution always requires
another click. Walking, PD standing, and passive recovery buttons remain
available during testing. F5/Enable is shared with the original gamepad tab.

Numbered policies skip their appended static reference tail and use zero extra
hold time. They share the regular SuperMimic entry bridge and automatically
return through the existing gait bridge to `walk_leo` (LB+A). This requires
rebuilding the updated C++ controller and restarting the simulation session;
models and exported reference/goal arrays are unchanged.

Chinese text needs a CJK font in the GUI environment. If it renders as square
glyphs, follow the Chinese font setup below.

Execution requires connected, enabled input and fresh robot state. Commands
are cancelled on disconnect, disable, stale feedback, or confirmation timeout;
reconnection never replays a pending action. `--test-catalog` is supplied by
the launcher, so a standalone window without that option cannot execute
numbered actions. See the shared catalog setup below; simulation startup
commands are in [the startup guide](../duel_sim/README.md).

### Shared test policies and Git

`assets/config/t800/numbered_actions/` is shared through Git. It contains
`catalog.json` and five selected policy packages, each with its YAML, MNN,
and reference motion. The catalog maps 01–05 to left hook, pull hammer,
left straight, left flick mid, and pull hammer; checkpoints and previous
numbers are listed in the [runner and transition reference](../../docs/t800_runner_classification_and_transitions.md#当前编号测试动作).

A fresh clone includes these five tests. For a checkout without a catalog,
initialize an empty one from the SDK root:

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
Restart the session to load changes; formal button mappings need no changes.
To share additions, commit the catalog together with the corresponding three
runtime files. This directory is no longer automatically ignored, so review
the staged files before committing. Keep training checkpoints, ONNX exports,
evaluation reports, and videos outside the runtime packages.

### Chinese font setup

If Chinese labels appear as boxes and `fc-list :lang=zh` finds no fonts, the
existing 109 setup uses the host's Droid font. When that font file is available,
run these commands on the host, then reopen the virtual gamepad:

```bash
docker cp /usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf engineai_robotics_env:/usr/local/share/fonts/
docker exec engineai_robotics_env fc-cache -f /usr/local/share/fonts
```

Available widgets/components:

1. `LcmManagerWidget`: connect and disconnect from the target LCM URL.
2. `VirtualGamepadWidget`: publish virtual gamepad messages for debugging.
3. `diagnose_gamepad.py`: basic LCM/message diagnostics for the virtual gamepad channel.

## License

This directory is licensed under **GNU General Public License v3.0 or later** (`GPL-3.0-or-later`).
See `LICENSE` for the full license text.

This subdirectory-specific license applies to the virtual gamepad tool itself. Other parts of the repository remain under the repository root license (`LICENSE.txt`) unless explicitly stated otherwise.

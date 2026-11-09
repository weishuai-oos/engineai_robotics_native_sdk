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

Available widgets/components:

1. `LcmManagerWidget`: connect and disconnect from the target LCM URL.
2. `VirtualGamepadWidget`: publish virtual gamepad messages for debugging.
3. `diagnose_gamepad.py`: basic LCM/message diagnostics for the virtual gamepad channel.

## License

This directory is licensed under **GNU General Public License v3.0 or later** (`GPL-3.0-or-later`).
See `LICENSE` for the full license text.

This subdirectory-specific license applies to the virtual gamepad tool itself. Other parts of the repository remain under the repository root license (`LICENSE.txt`) unless explicitly stated otherwise.

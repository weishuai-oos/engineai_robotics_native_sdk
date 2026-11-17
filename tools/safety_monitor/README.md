# Sim to real data collection

`record_safety.py` is a read-only ROS2/LCM recorder. It writes one
`samples.jsonl` row on every joint-state callback, plus `events.jsonl` and
`metadata.json`. MuJoCo and the robot expose the same ROS2 bridge topics.
Starting this recorder is optional; it only subscribes to telemetry and never
publishes commands or changes the controller. All records are stored in the
dedicated repository folder `records/safety_monitor/`, which is ignored by
Git so runtime data does not appear as source changes.

For a real T800 run, use the host-side wrapper after the robot controller is
already running:

```bash
./tools/safety_monitor/start_real_recording.sh --duration 300
```

The wrapper starts or reuses `engineai_robotics_env`, sets the real-robot ROS 2
domain and Fast DDS implementation, disables the container's localhost-only
setting, checks `/hardware/joint_state` and `/hardware/rc02_diagnostics`, and
starts the recorder with `assets/config/t800/lcm/robot.yaml`. The default run
name is `real_t800_<timestamp>`. A custom name can be supplied as the first
argument:

```bash
./tools/safety_monitor/start_real_recording.sh real_t800_walk_01
```

Press Ctrl-C in the recording terminal to flush and close the files. Use
`--skip-preflight` only when the ROS 2 graph check is unavailable. The wrapper
accepts `--ros-domain` and `--rmw` when a robot uses values different from the
T800 defaults.

After the simulator or robot controller is already running, open a second
terminal in the repository and start the recorder with one command:

```bash
./tools/safety_monitor/start_recording.sh sim_run_01 --config assets/config/t800/mode.yaml
```

The wrapper sources `/opt/ros/$ROS_DISTRO` and the SDK overlay at
`build/x86_64/ros2_env/install/setup.bash`. Set `ENGINEAI_ROS2_OVERLAY` if the
overlay is elsewhere. Extra options such as `--duration 300` are passed to the
recorder.

The relative `--output sim_run_01` above resolves to
`<repo>/records/safety_monitor/sim_run_01`. Absolute output paths are also
accepted only when they remain below that same `records/safety_monitor/`
directory.

Use the existing controller launch procedure for the run. MuJoCo launchers
select the virtual controller or Xbox/F710 gamepad, while the normal robot
startup keeps its existing RC02 input behavior. The recorder does not change
the input source.

Run the same command during a MuJoCo session and a real-robot session, then
send the two completed record directories for gap analysis. Each record keeps
its local receive monotonic time and ROS stamp separately; timestamps from two
hosts are never subtracted.

The `/hardware/gamepad_keys` stream is included in each sample. In MuJoCo this
is the virtual controller or Xbox source. RC02 is only used on the real robot;
`/hardware/rc02_diagnostics` preserves the complete raw frame bytes (including
header, length, and CRC), heartbeat, trigger bytes, sticks, buttons,
motion/skill codes, and normalized values when that adapter is active.
When LCM is available, the recorder also listens to `sim_state` and stores
MuJoCo simulation time, base/IMU truth, joint truth, and contact force in the
sample. It also stores `sim_control_diagnostics` with nominal and effective
control, the callback-applied control delta, and command age/validity. The
stable controller path keeps the delta at zero.
When the rebuilt ROS2 overlay contains `/motion/policy_diagnostics`, each
sample also includes the policy observation, raw/clipped/controlled action,
transformed state, and pre/post-transition targets.

Policy diagnostics are sampled at 100 Hz by default so telemetry cannot consume
the walking loop's full 500 Hz budget. If a run needs one snapshot per control
cycle, set `ENGINEAI_POLICY_DIAGNOSTICS_HZ=500` **before starting the normal
controller**; this is a controller environment setting, not a recorder option.
MuJoCo's `sim_control_diagnostics` channel follows the same 100 Hz default; set
`ENGINEAI_SIM_CONTROL_DIAGNOSTICS_HZ=500` before starting MuJoCo when every
control-cycle validity sample is required.

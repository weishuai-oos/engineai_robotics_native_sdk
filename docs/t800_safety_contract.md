# T800 SDK safety contract

The sole T800 safety source for the native SDK is
`assets/config/t800/safety/t800_safety_contract.json`. It is SDK-local and must
not depend on another repository or generated snapshot.

## Schema v2

The contract contains exactly 25 SDK joints. Each joint records:

- `hard_limits`: URDF position interval, velocity and positive effort magnitude.
- `operational_limits`: SDK command envelope.
- `sdk_name` and `sdk_index`: stable SDK-side identity.

Operational defaults are 90% of the URDF values:

- Position endpoints are scaled about zero: `0.9 * lower`, `0.9 * upper`.
- Velocity is `0.9 * velocity`.
- Effort is `0.9 * effort`; runtime torque checks use the signed interval
  `[-effort_nm, +effort_nm]`.
- One-sided intervals require a stricter explicit override. Both knee joints
  reserve a 0.05 rad lower position bound. Every override must include a
  human-readable `reason` and `evidence`, and may only tighten the envelope.

The URDF values are model limits, not hardware certification. The
`qualification: urdf_derived_provisional` marker is intentional. The contract
does not claim acceleration, jerk, current, temperature, zero-offset, collision,
or impact limits because those values are not established by the URDF.

## Runtime meaning

The operational envelope is enforced by the SDK-resident
`t800_safety_runner` every 2 ms. On hardware it runs after motion aggregation
and before `joint_motor_transform_runner`; a second
`t800_motor_pre_driver_safety_runner` checks the transformed `motor_info` frame
before `motor_runner`. In simulation the joint-space gate runs before
`sim_publish_runner`. The joint gate validates dimensions and finite values,
rejects full-torque mode and negative gains, bounds desired position, velocity,
target slew and feed-forward torque, and rejects commands whose estimated PD
effort exceeds the provisional operational effort. The pre-driver gate again
rejects non-finite values, negative gains, full-torque commands, and transformed
desired position, desired velocity, feed-forward torque, and estimated PD effort
outside that envelope for direct-drive joints. The four parallel-ankle actuator
channels receive numeric/gain/full-torque checks but not fabricated motor-space
position, velocity, or effort limits: a qualified transform/Jacobian and real
actuator limits are required for that boundary.

Every safe writeback first clears the complete command frame, including the
legacy full-torque channel, and then writes only the validated PD command. This
also prevents a stale torque command from bypassing the simulation path, which
does not use the hardware pre-driver gate.

The first accepted target is slew-limited from the measured position; later
targets are slew-limited from the last accepted target. A non-head feedback
fault or a rejected command produces a finite zero-actuation command instead of
reusing the previous unsafe request. The post-gate joint override converter is
disabled because it could otherwise replace an already checked command.

J23/J24 head feedback is isolated from policy inputs. NaN/Inf, a hard position
or velocity violation, a MotorDebug offline/disabled/error flag, an implausible
single-frame position jump, ten consecutive position/velocity mismatch frames,
or 100 ms of commanded tracking error with near-zero measured velocity marks
that head joint failed. Policies receive the last good head position with zero
velocity, their head action/history is cleared where applicable, and both SDK
gates send zero head gains and torque. Recovery requires 50 consecutive healthy
safety frames (100 ms at the resident 2 ms period). A safety snapshot older
than 50 ms is rejected, and policy consumers fail closed before the first
resident safety snapshot exists.

On real hardware, `MotorDebug` is mandatory. Its publisher must replace the
shared sample at least once every 100 ms and its `offline`, `enable`, and
`error_code` arrays must contain exactly 25 entries. Missing, malformed, or
stale diagnostics produce a whole-frame zero-actuation fault. MuJoCo is allowed
to run without MotorDebug because there is no hardware diagnostic publisher in
that mode.

Per-state protections remain in place as defense in depth; they do not replace
the global SDK boundary. This contract alone does not certify continued
operation after a motor, sensor, bus, IMU, or mechanical failure. The head
consistency checks catch finite jumps and sustained command/feedback
disagreement, but finite biased feedback that remains mutually plausible still
cannot be identified reliably because `JointInfo` has no source timestamp or
sequence number. A non-head fault currently removes actuation rather than
executing a hardware-qualified controlled kneel or fall, so the surrounding
area and fall protection remain part of the real-robot safety case.

## Source integrity

The source URDF path and SHA-256 are recorded in `provenance`. If the URDF
changes, regenerate the contract values and update the hash in the same review.
Consumers must use the explicit SDK name and index; array order must not be
silently reinterpreted.

The contract remains provisional until hardware evidence and independent runtime
tests establish actuator, thermal, calibration, collision, and interlock limits.
The added post-transform check is deliberately named a **pre-driver** gate: the
precompiled `motor_runner` still applies sign, zero offset, minimum gain and
driver encoding before `set_motor_cmd()`. Without its source and real actuator
limits, this SDK cannot claim an independently qualified final hardware gate or
parallel-ankle motor-position envelope. There is also no current/temperature
derating gate, self-collision checker, per-joint feedback timestamp, or hardware
watchdog implemented by this contract.

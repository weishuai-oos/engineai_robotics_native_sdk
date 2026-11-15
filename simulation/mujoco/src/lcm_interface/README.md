# MuJoCo LCM diagnostics wire types

The `sim_control_diagnostics` channel uses the pre-generated LCM type
`data::DynamicVectorDouble` from
`core/include/data/lcm_data/DynamicVectorDouble.hpp`. Its wire layout is:

```text
timestamp: double
num_ranges: int32
data: double[num_ranges]
```

The ROS2 message with the same short name,
`interface_protocol/msg/DynamicVectorDouble.msg`, is a separate ROS interface
with `std_msgs/Header` and `float64[] data`. It is not used by this LCM
channel. The C++ member is named `lcm_control_diagnostics_` to keep the two
protocols distinct in the MuJoCo code.

For `sim_control_diagnostics`, `data` is laid out as
`nominal_ctrl[N]`, `effective_ctrl[N]`, `injected_noise[N]`, `command_age_ms`,
and `command_valid` encoded as `1.0` or `0.0`. The stable controller path
keeps nominal and effective controls identical, so `injected_noise` is zero.
The publisher samples this channel at 100 Hz by default. Set
`ENGINEAI_SIM_CONTROL_DIAGNOSTICS_HZ=500` before starting MuJoCo for one
message per control cycle.

# litearm_driver

A standalone ROS 2 lifecycle node that owns the LiteArm arm's USB link and exposes the
SDK's administrative command set over services, for maintaining, licensing or tuning an
arm without a controller_manager in the loop.

English · [简体中文](README.zh-CN.md)

The node talks to the firmware through the [litearm-cpp](https://github.com/nexform-tech/litearm-cpp)
SDK. It creates its services on activation and destroys them on deactivation, and it
serialises every SDK call through one callback group.

**This node is mutually exclusive with the ros2_control stack.** Both open the same serial
port and the SDK takes an exclusive `flock` on it, so the second process fails to open the
port. Stop one before starting the other.

## Run

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

The launch file puts the node in the `litearm` namespace, so its relative service names
become `/litearm/enable`, `/litearm/clear_faults` and so on. Every service name below is
written relative to that namespace.

## Command set

The node exposes 24 services in six groups, three topics and 13 parameters. Every service
returns `success` plus a `message`; on failure the message carries the SDK's own words, and
when a gate refused the call before the frame was built, the reason for the refusal.

Three rules shape the whole surface:

- **Gates run before the frame.** A refused argument puts nothing on the wire. The node
  refuses what the firmware would silently turn into a different request: a payload mass
  outside `[0, 20]` kg, a speed percent outside `0..100`, a zero-gravity keep-alive period
  past the 100 ms command watchdog, a feedforward value that is not finite.
- **One command at a time.** Every SDK call runs in one mutually exclusive callback group,
  so two commands can never interleave on the wire. A command waits for the firmware ACK,
  so a stalled link can hold a service call for up to the SDK's own 1.2 s timeout.
- **Reads are cached.** Only `get_status`, `get_license`, `get_feedforward_scalar` and
  `get_joint_params` talk to the firmware on demand. `/litearm/status` never does, so
  subscribing cannot disturb a command in flight.

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | `""` | USB CDC device path. Empty means auto-discovery by VID:PID `1d50:606f`. |
| `joint_names` | unset | Axis names in firmware axis order. Unset derives `joint1`..`jointN` from the axes the firmware reports; a list of the wrong length fails configure. |
| `auto_enable` | `true` | Enable the motors on activation. A refused ENABLE fails the activation instead of leaving an "active" node with dead motors. |
| `enable_attempts` | `12` | Retries for the `enable` service. Only the code the firmware marks retryable consumes them. |
| `status_rate_hz` | `10.0` | `/litearm/status` publication rate. |
| `publish_joint_states` | `false` | Publish `joint_states_topic`. Keep it false whenever a `joint_state_broadcaster` runs. |
| `joint_states_topic` | `/joint_states` | Topic for the optional joint states. |
| `publish_diagnostics` | `true` | Publish `/diagnostics`. |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` publication rate. |
| `zero_g_keepalive_period_s` | `0.04` | Zero-gravity keep-alive period, `[0.005, 0.10)`. Read on every `zero_g` call. |
| `allow_dfu` | `false` | Gate for `enter_dfu`. Read on every call. |
| `allow_license_activation` | `false` | Gate for `activate_license`. Read on every call. |
| `frame_id` | `""` | Header frame id of the status and joint state messages. |

### Topics

| Topic | Type | Contents |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | Cached and local SDK state at `status_rate_hz`. Never sends a frame. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | One status: link, enable, fault, drop-hold inference, licence, feedback age. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Only when `publish_joint_states:=true`. Off by default so it cannot compete with `joint_state_broadcaster`. |

`LitearmStatus` carries, in `joint_names` order: `temperature_mos`, `temperature_coil`,
`joint_error_code` and the `joint_fault` bitmap. An axis the current frame does not cover
reads `NaN` for temperature and `0` for the error code; `feedback_age` is `-1` until the
first frame arrives. `speed_scaling` and the `zero_g_*` fields are host-side views, not
read-back values.

### Service index

| Service | Type | Purpose |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | Enable every axis. |
| `disable` | `std_srvs/srv/Trigger` | Disable every axis; the arm stops being held. |
| `reset` | `std_srvs/srv/Trigger` | Clear the fault and re-anchor the pose. |
| `clear_faults` | `std_srvs/srv/Trigger` | Clear the RAM fault bits only. |
| `emergency_stop` | `std_srvs/srv/Trigger` | Request a stop. |
| `park` | `std_srvs/srv/Trigger` | Declare a static hold at full stiffness. |
| `zero_g` | `std_srvs/srv/SetBool` | Enter or leave zero gravity (hand-guided motion). |
| `get_status` | `litearm_msgs/srv/GetStatus` | Send `GET_STATUS` and return one snapshot. |
| `get_license` | `litearm_msgs/srv/GetLicense` | Read the device licence record. |
| `activate_license` | `litearm_msgs/srv/ActivateLicense` | Submit a vendor-signed credential. |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | Write the global speed governor, in percent. |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | Write the firmware motion mode. |
| `set_payload` | `litearm_msgs/srv/SetPayload` | Declare the end-effector mass and centre of mass. |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | Write the feedforward enable mask. |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | Select a feedforward preset. |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | Write one feedforward scalar. |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | Write one feedforward vector. |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | Read one scalar back. |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | Read the joint parameter table. |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | Write `kp`/`kd`/`tau_max` of one axis. |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | Write the soft limits of one axis. |
| `reset_factory_params` | `std_srvs/srv/Trigger` | Restore the factory joint parameters. |
| `save_params` | `std_srvs/srv/Trigger` | Write the runtime parameters to flash. |
| `enter_dfu` | `std_srvs/srv/Trigger` | Mark the device for the ROM bootloader. |

### State and safety

#### `enable`

`Arm::enable(enable_attempts)`. Only the error code the firmware marks retryable is
retried; a latched fault or a missing licence fails immediately.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. The licence must be activated, otherwise the firmware refuses ENABLE with `ERR{0x10,0x08}`. |

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger "{}"
```

#### `disable`

`Arm::disable()`. Bypasses the zero-gravity guard, because losing energy is safe.

**Do not** call `disable` on an arm that is not supported: the position loop stops holding
and a load will fall.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/disable std_srvs/srv/Trigger "{}"
```

#### `reset`

`Arm::reset()`: clear the fault and re-anchor the pose. Not an MCU reset.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/reset std_srvs/srv/Trigger "{}"
```

#### `clear_faults`

`Arm::clear_faults()`: clear the RAM fault bits only. Use `reset` when the pose reference
also has to be re-anchored.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
```

#### `emergency_stop`

`Arm::emergency_stop()`. A software request: the hardware emergency stop remains the
authority, and the firmware's own watchdog holds the arm if the link dies.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/emergency_stop std_srvs/srv/Trigger "{}"
```

#### `park`

`Arm::park()`: declare a static hold at full stiffness, which does not depend on the 100 ms
command watchdog.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/park std_srvs/srv/Trigger "{}"
```

#### `zero_g`

`data=true` calls `Arm::zero_g_start(zero_g_keepalive_period_s)`; `data=false` calls
`Arm::zero_g_stop(true)`, so a keep-alive thread that died is reported instead of swallowed.
The SDK owns the keep-alive thread.

**Do not** send any other motion command while zero gravity is active: the firmware rejects
it, and the arm is under human guidance at that moment. Leave zero gravity first.

| Field | Type | Meaning |
| --- | --- | --- |
| `data` (request) | `bool` | `true` enters zero gravity, `false` leaves it. |
| `success` (response) | `bool` | `false` when the gate or the SDK refused. |
| `message` (response) | `string` | On leave, carries the keep-alive failure if one happened. |

```bash
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: false}"
```

### Status and licence

#### `get_status`

Sends `GET_STATUS` (`Arm::get_status_now(timeout)`) and returns one `LitearmStatus`. Use it
when the passive stream is silent and you need proof of life with a deadline.

| Field | Type | Meaning |
| --- | --- | --- |
| `timeout` (request) | `float64` | Seconds to wait for the reply; `<= 0` returns the cached frame immediately. |
| `status` (response) | `litearm_msgs/LitearmStatus` | The snapshot, same fields as the topic. |
| `success` (response) | `bool` | `false` when the arm is not connected or the firmware did not answer. |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

#### `get_license`

`Arm::license()`: the device licence record. An unactivated arm answers normally, because
"not activated" is a state and not an error. The record also refreshes the node's cached
copy, which is what `/litearm/status` reports.

| Field | Type | Meaning |
| --- | --- | --- |
| `state` (response) | `uint8` | 0 unactivated, 1 activated, 2 activated in factory mode. |
| `state_name` (response) | `string` | The name the SDK gives that state. |
| `cust_id` (response) | `uint32` | Customer number; 0 while unactivated. |
| `issued` (response) | `uint32` | Issue date as `YYYYMMDD`; 0 while unactivated. |
| `flags` (response) | `uint32` | bit 0 is the factory code. |
| `uid_hex` (response) | `string` | 24 lowercase hex characters: the UID the signing tool must use. |

```bash
ros2 service call /litearm/get_license litearm_msgs/srv/GetLicense "{}"
```

#### `activate_license`

`Arm::activate(cust_id, issued, flags, mac, 16)` followed by a read-back, because the
firmware aggregates "already activated" and "MAC mismatch" into one error code; the state
is the only reliable answer.

**Do not** activate while the motors are enabled: the service refuses, and so do the SDK
and the firmware. Read `get_license` first and sign the UID from that record.

| Field | Type | Meaning |
| --- | --- | --- |
| `cust_id` (request) | `uint32` | Customer number. |
| `issued` (request) | `uint32` | Issue date as `YYYYMMDD`. |
| `flags` (request) | `uint32` | bit 0 is the factory code. |
| `mac` (request) | `uint8[16]` | Two SipHash-2-4 tags from the vendor's signing tool. |
| `state`, `state_name` (response) | `uint8`, `string` | The licence state read back after the attempt. |
| Preconditions | | `allow_license_activation:=true` and disabled motors. |

```bash
ros2 param set /litearm/driver allow_license_activation true
ros2 service call /litearm/activate_license litearm_msgs/srv/ActivateLicense \
  "{cust_id: 7, issued: 20261006, flags: 0, mac: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

### Motion configuration

#### `set_speed_scaling`

`Arm::set_speed(percent)`: the firmware's global speed governor. It is global and persists
until changed, unlike the per-move `speed` argument of the SDK's `movej`.

| Field | Type | Meaning |
| --- | --- | --- |
| `percent` (request) | `uint8` | 0..100. It is an integer percentage, so 1 is one percent of full speed, not full speed. |
| Gate | | Out-of-range values are refused before any frame is built. |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
```

#### `set_motion_mode`

`Arm::set_motion_mode(mode)`. This firmware recognises mode 0 only, which is the same thing
as `park()`; the SDK rejects any other value rather than ACKing a mode change that does not
happen. The response message says so.

| Field | Type | Meaning |
| --- | --- | --- |
| `mode` (request) | `int32` | Mode number. 0 is the only accepted value today. |

```bash
ros2 service call /litearm/set_motion_mode litearm_msgs/srv/SetMotionMode "{mode: 0}"
```

#### `set_payload`

`Arm::set_payload(mass, com)`: declare the load the firmware compensates for. The firmware
clamps mass into `[0, 20]` kg silently, so the node refuses a value outside that range
rather than reporting success for a different number.

| Field | Type | Meaning |
| --- | --- | --- |
| `mass` (request) | `float64` | Kilograms, `[0, 20]`. |
| `com` (request) | `float64[3]` | Centre of mass in metres, flange frame, all components finite. |
| Gate | | Range and finiteness are checked here; the SDK and the firmware enforce the wire rule too. |

Read it back with `get_feedforward_scalar` (`item: 4, sub: 0`), which returns the clamped
truth.

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### Dynamics and feedforward

The firmware clamps feedforward values silently, so a successful write means the clamped
value was stored, not the requested one. **Read back what you wrote.**

#### `set_feedforward_mask`

`Arm::set_ff_mask(mask)`: the feedforward enable mask. Bits outside `FF_ALL` are rejected
by the SDK rather than folded into "all off", which would silently remove gravity
compensation.

| Field | Type | Meaning |
| --- | --- | --- |
| `mask` (request) | `uint32` | Only bits inside `FF_ALL` are accepted. |

```bash
ros2 service call /litearm/set_feedforward_mask litearm_msgs/srv/SetFeedforwardMask "{mask: 511}"
```

#### `set_feedforward_preset`

`Arm::ff_preset(preset)`.

| Field | Type | Meaning |
| --- | --- | --- |
| `preset` (request) | `int32` | 0 all off, 1 factory, 2 all on. |
| Gate | | Any other value is refused before any frame is built. |

```bash
ros2 service call /litearm/set_feedforward_preset litearm_msgs/srv/SetFeedforwardPreset "{preset: 1}"
```

#### `set_feedforward_scalar`

`Arm::set_ff_scalar(item, sub, value)`. Item numbering is
`litearm::Arm::ff_scalar_items()`; `sub` is meaningful for items 5 and 6 only.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Scalar item number. |
| `sub` (request) | `int32` | Sub-index; meaningful for items 5 and 6 (0..2). |
| `value` (request) | `float64` | The value to store; the firmware clamps it. |

```bash
ros2 service call /litearm/set_feedforward_scalar litearm_msgs/srv/SetFeedforwardScalar \
  "{item: 4, sub: 0, value: 0.8}"
```

#### `set_feedforward_vector`

`Arm::set_ff_vec(item, values)`. Item numbering is `litearm::Arm::ff_vec_items()`: 7 values
for items 1..5, 7 and 8; 3 for the gravity vector (item 6). The SDK rejects a wrong length;
the node refuses an empty list or a value that is not finite.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Vector item number. |
| `values` (request) | `float64[]` | Non-empty, all finite; the length has to match the item. |

```bash
ros2 service call /litearm/set_feedforward_vector litearm_msgs/srv/SetFeedforwardVector \
  "{item: 7, values: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

#### `get_feedforward_scalar`

`Arm::get_ff_scalar(item, sub)`: the read-back path. Item 9 is the read-only extension that
returns `ff_mask`.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Scalar item number; 9 reads `ff_mask`. |
| `sub` (request) | `int32` | Sub-index; meaningful for items 5 and 6. |
| `value` (response) | `float64` | The value the firmware actually holds. |

```bash
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 9, sub: 0}"
```

### Joint parameters

#### `get_joint_params`

`JointParams::get_joint_param(joint)` or `all_joint_params()`. One axis or the whole table.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index, or `-1` for every axis. |
| `params` (response) | `litearm_msgs/JointParam[]` | One entry per requested axis. |
| Gate | | An index outside the reported axes is refused. |

Each `JointParam` carries `index`, `kp`, `kd`, `tau_max`, `q_min`, `q_max`.

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
```

#### `set_joint_gains`

`JointParams::set_joint_param(joint, kp, kd, tau_max)`. These are the gains of the
firmware's own position loop; the ros2_control component sends none of its own.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index. |
| `kp`, `kd`, `tau_max` (request) | `float64` | Position gain, damping gain and torque ceiling for that axis. |

```bash
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
```

#### `set_joint_limits`

`JointParams::set_joint_limits(joint, q_min, q_max)`. The host caches the limits at connect
time, so the SDK's local pre-check picks the new values up on the next connect, not
immediately.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index. |
| `q_min`, `q_max` (request) | `float64` | Soft limits in radians. |

```bash
ros2 service call /litearm/set_joint_limits litearm_msgs/srv/SetJointLimits \
  "{joint: 0, q_min: -1.0, q_max: 1.0}"
```

#### `reset_factory_params`

`JointParams::reset_factory()`. Requires disabled motors.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up and motors disabled. |

```bash
ros2 service call /litearm/reset_factory_params std_srvs/srv/Trigger "{}"
```

#### `save_params`

`Arm::save_params()`: write the runtime parameters to flash. Requires disabled motors.
A flash write is not reversible.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up and motors disabled. |

```bash
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### Maintenance

#### `enter_dfu`

`Arm::enter_dfu()`: mark the device for the ROM bootloader. After the ACK the CDC device
disappears, this node is unusable until it is reconfigured with a new link, and the port
re-enumerates as `0483:DF11` until a firmware image is flashed.

**Do not** call `enter_dfu` to try it out.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | `allow_dfu:=true`, motors disabled, valid ROM vector table. |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

### Refusals

Each gate refuses before any frame is built, so a refused call puts nothing on the wire.
The test `RefusedArgumentsSendNoFrame` covers that.

| Refusal message | Trigger | Why the node refuses instead of forwarding |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` outside the range | The firmware takes an integer percent: `1` is one percent, not full speed. |
| `preset must be 0 ... 2` | `set_feedforward_preset` outside the range | Only three presets exist. |
| `period must be in [0.005, 0.10)` | `zero_g` with a bad keep-alive period | The firmware command watchdog trips at 0.10 s, so a slower keep-alive would silently drop out of zero gravity. |
| `mass must be in [0, 20] kg` | `set_payload` outside the range | The firmware clamps silently; the stored value would differ from the requested one. |
| `requires the motors to be disabled` | `save_params`, `reset_factory_params`, `activate_license` while enabled | Flash writes and activation are only safe with no torque authority. |
| `set the parameter ... to true` | `enter_dfu`, `activate_license` while the opt-in parameter is false | Both are irreversible or trust-relevant; the default is off. |
| `not connected` | any service with no live link | Names the usual causes: USB cable, 24 V supply, or the ros2_control stack holding the port. |
| `axis index must be 0..N-1` | a joint service with an index outside the reported axes | A typo would otherwise address the wrong axis. |

The pre-checks are best effort: `save_params` reads the cached state frame to decide whether
the motors are enabled. The SDK and the firmware enforce the same rule on the wire, so a
stale cache can only turn a clean refusal into the firmware's own error message, never into
an unsafe write.

### Not exposed

Trajectory execution, cartesian servo and teleoperation, firmware cartesian planning, model
import, log capture and `kin_bench` have no service here. Motion control belongs to the
ros2_control stack (`joint_trajectory_controller`, MoveIt 2), and servo needs a controller
in the command path rather than a service that hands over one pose at a time. The reasons
are in [`docs/command-set.md`](../docs/command-set.md), which also carries the ownership
rules, the error semantics and the compatibility table.

## Tests

The suite is offline. It injects the SDK's `FakeTransport` and drives the whole lifecycle
and every service handler without an arm attached:

```bash
colcon test --packages-select litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## License

Apache-2.0. See [LICENSE](../LICENSE).

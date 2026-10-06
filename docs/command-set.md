# litearm command set

This is the reference for every ROS 2 interface `litearm_driver` exposes and what each one
does to the arm; read it when you operate an arm without the ros2_control stack, or when
you write a client against the driver. The Chinese reference is
[`docs/command-set.zh-CN.md`](command-set.zh-CN.md); the English version here is canonical.

Verification status: the service names, types, request handling, refusals and the status
message are covered by the offline test suite (`colcon test --packages-select litearm_driver`).
The `ros2` command lines below are the interface contract as tested; running them against
a real arm was not part of this repository's test run.

## 1. One process per arm

The driver opens the arm's USB CDC port and the SDK takes an exclusive `flock` on it. Two
processes therefore cannot drive one arm:

- **Do not run `litearm_driver` while the ros2_control stack is running.** The second
  process fails to open the port, and the driver reports it at configure time.
- The same rule applies in reverse: stop the driver before you start
  `ros2_control_node`.

The driver is the maintenance path: licensing, fault handling, parameter tuning and
bring-up checks, with no `controller_manager` in the loop. Motion control belongs to
`joint_trajectory_controller` (via MoveIt 2) or to a custom controller such as the servo
controller in `litearm_servo_control`.

## 2. Start and stop

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

The launch file starts the node as `/litearm/driver` in the `litearm` namespace, so every
service below is `/litearm/<name>`. It configures and activates the node unless you pass
`autostart:=false`.

```bash
ros2 launch litearm_driver litearm_driver.launch.py autostart:=false
ros2 lifecycle set /litearm/driver configure
ros2 lifecycle set /litearm/driver activate
ros2 lifecycle get /litearm/driver
```

Deactivating (`ros2 lifecycle set /litearm/driver deactivate`) removes every service,
stops zero gravity if it was active, and leaves the motors as they are: `park()` and the
firmware hold keep the arm where it is. Disabling the motors on deactivate would let the
arm fall.

Run the executable directly only when you set the namespace yourself; without a namespace
the relative service names resolve to `/enable`, `/status` and so on.

### 2.1 Parameters

Every parameter has a default; the node reads them at configure time, except
`allow_dfu`, `allow_license_activation` and `zero_g_keepalive_period_s`, which are read
again on each call so `ros2 param set` takes effect immediately.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | `""` | USB CDC device path. Empty means auto-discovery by VID:PID `1d50:606f`. |
| `joint_names` | unset | Axis names in firmware axis order. Unset derives `joint1`..`jointN` from the axes the firmware reports; a list of the wrong length fails configure. |
| `auto_enable` | `true` | Enable the motors on activation. A refused ENABLE fails the activation instead of leaving an "active" node with dead motors. |
| `enable_attempts` | `12` | `enable()` retries; only the firmware's retryable code consumes them. |
| `status_rate_hz` | `10.0` | `/litearm/status` publication rate. |
| `publish_joint_states` | `false` | Publish `joint_states_topic`. Keep it false whenever a `joint_state_broadcaster` runs. |
| `joint_states_topic` | `/joint_states` | Topic for the optional joint states. |
| `publish_diagnostics` | `true` | Publish `/diagnostics`. |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` publication rate. |
| `zero_g_keepalive_period_s` | `0.04` | Zero-gravity keep-alive period, `[0.005, 0.10)`. |
| `allow_dfu` | `false` | Gate for `enter_dfu`. |
| `allow_license_activation` | `false` | Gate for `activate_license`. |
| `frame_id` | `""` | Header frame id of the status and joint state messages. |

## 3. Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | Cached and local SDK state only; the publisher never sends a frame, so subscribing cannot disturb a command. `speed_scaling` and the `zero_g_*` fields are host-side views, not read-back values. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | One status: link, enable, fault, drop-hold inference, licence, feedback age. Published by the node itself rather than through `diagnostic_updater`, to keep the dependency set to what the ros2_control stack already needs. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Only when `publish_joint_states:=true`. |

`LitearmStatus` carries the per-axis temperatures, error codes and `joint_fault` bitmap
indexed in `joint_names` order. An axis the current frame does not cover reads `NaN` for
temperature and `0` for the error code; `feedback_age` is `-1` before the first frame.

## 4. Services

All service names are relative to the node's namespace, so with the launch file they are
`/litearm/<name>`. `success=false` always comes with a `message` that names the reason,
including, where a gate refused the call, why it refused.

### 4.1 State and safety

| Service | Type | What it does |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | `Arm::enable(enable_attempts)`. |
| `disable` | `std_srvs/srv/Trigger` | `Arm::disable()`. The arm stops being held; support it first. |
| `reset` | `std_srvs/srv/Trigger` | `Arm::reset()`: clear the fault and re-anchor the pose. Not an MCU reset. |
| `clear_faults` | `std_srvs/srv/Trigger` | `Arm::clear_faults()`: clear the RAM fault bits only. |
| `emergency_stop` | `std_srvs/srv/Trigger` | `Arm::emergency_stop()`. A software request; the hardware emergency stop remains the authority. |
| `park` | `std_srvs/srv/Trigger` | `Arm::park()`: declare a static hold at full stiffness. |
| `zero_g` | `std_srvs/srv/SetBool` | `data=true` enters zero gravity and starts the keep-alive; `data=false` leaves it. The arm is then free to move by hand, so run it with the workspace clear. |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
```

### 4.2 Status and licence

| Service | Type | What it does |
| --- | --- | --- |
| `get_status` | `litearm_msgs/srv/GetStatus` | Sends `GET_STATUS` and returns one `LitearmStatus`. `timeout <= 0` returns the cached frame immediately. Use it when the passive stream is silent and you need proof of life with a deadline. |
| `get_license` | `litearm_msgs/srv/GetLicense` | Reads the device licence record. An unactivated arm answers normally: "not activated" is a state, not an error. |
| `activate_license` | `litearm_msgs/srv/ActivateLicense` | Submits a vendor-signed credential. Requires `allow_license_activation:=true` and disabled motors. Read `get_license` first: the signing tool must use the device UID from that record. |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
ros2 param set /litearm/driver allow_license_activation true
ros2 service call /litearm/activate_license litearm_msgs/srv/ActivateLicense \
  "{cust_id: 7, issued: 20261006, flags: 0, mac: [171, 171, 171, 171, 171, 171, 171, 171, \
  171, 171, 171, 171, 171, 171, 171, 171]}"
```

The `mac` value is produced by the vendor's signing tool. Nothing in this repository
computes or stores key material.

### 4.3 Motion configuration

| Service | Type | What it does |
| --- | --- | --- |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | Writes the firmware's global speed governor, an integer percent 0..100. It is global and persists until changed, unlike the per-move `speed` argument of the SDK's `movej`. |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | Writes `SET_MOTION_MODE`. The current firmware recognises mode 0 only, which is `park()`; the SDK rejects any other value rather than ACKing a mode change that does not happen. |
| `set_payload` | `litearm_msgs/srv/SetPayload` | Declares the end-effector mass (0..20 kg) and centre of mass (metres, flange frame). |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### 4.4 Dynamics and feedforward

The firmware clamps feedforward values silently, so a write reports success for the
clamped value, not the requested one. Read back what you wrote.

| Service | Type | What it does |
| --- | --- | --- |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | Writes the feedforward enable mask. Bits outside `FF_ALL` are refused instead of being folded into "all off", which would remove gravity compensation. |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | 0 all off, 1 factory, 2 all on. |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | Writes one scalar (`item`, `sub`, `value`). `sub` is meaningful for items 5 and 6 only. Item numbering is `litearm::Arm::ff_scalar_items()`. |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | Writes one vector (`item`, `values`): 7 values for items 1..5, 7 and 8; 3 for the gravity vector (item 6). The SDK rejects a wrong length; non-finite values are refused here. |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | Reads one scalar back. Item 9 is the read-only extension that returns `ff_mask`. |

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload "{mass: 0.8, com: [0, 0, 0.05]}"
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 4, sub: 0}"
```

The feedforward mask and item numbering come from the firmware's own parameter table; the
SDK's `ff_scalar_items()` and `ff_vec_items()` are the authoritative lists.

### 4.5 Joint parameters

| Service | Type | What it does |
| --- | --- | --- |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | Reads one axis (`joint >= 0`) or every axis (`joint: -1`). |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | Writes `kp`, `kd`, `tau_max` for one axis. These are the gains of the firmware's own position loop. |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | Writes the soft limits of one axis. The host caches the limits at connect time, so the SDK's local pre-check picks them up on the next connect. |
| `reset_factory_params` | `std_srvs/srv/Trigger` | `JointParams::reset_factory()`. Requires disabled motors. |
| `save_params` | `std_srvs/srv/Trigger` | Writes the runtime parameters to flash. Requires disabled motors. Flash writes are not reversible. |

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### 4.6 Maintenance

| Service | Type | What it does |
| --- | --- | --- |
| `enter_dfu` | `std_srvs/srv/Trigger` | Marks the device for the ROM bootloader. Requires `allow_dfu:=true`, disabled motors and a valid ROM vector table. After an ACK the CDC device disappears and this node is unusable until it is reconfigured with a new link. |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

**Do not** call `enter_dfu` to "try it out": the device re-enumerates as `0483:DF11` and
needs a firmware image to come back.

## 5. Refusals and what they mean

Every refusal names the parameter, the value or the operation it refused. The gates run
before any frame is built, so a refused call puts nothing on the wire (covered by the
test `RefusedArgumentsSendNoFrame`).

| Refusal | Trigger | Why the driver refuses instead of forwarding |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` outside the range | The firmware takes an integer percent: `1` is one percent, not full speed. |
| `preset must be 0 ... 2` | `set_feedforward_preset` outside the range | Only three presets exist. |
| `period must be in [0.005, 0.10)` | `zero_g` with a bad `zero_g_keepalive_period_s` | The firmware command watchdog trips at 0.10 s, so a slower keep-alive would silently drop out of zero gravity. |
| `mass must be in [0, 20] kg` | `set_payload` outside the range | The firmware clamps silently; the stored value would differ from the requested one. |
| `requires the motors to be disabled` | `save_params`, `reset_factory_params`, `activate_license` while enabled | Flash writes and activation are only safe with no torque authority. |
| `set the parameter ... to true` | `enter_dfu`, `activate_license` while the opt-in parameter is false | Both are irreversible or trust-relevant; the default is off. |
| `not connected` | any service with no live link | Names the usual causes: USB cable, 24 V supply, or the ros2_control stack holding the port. |
| `axis index must be 0..N-1` | a joint service with an index outside the reported axes | A typo would otherwise address the wrong axis. |

The pre-checks are best effort: `save_params` reads the cached state frame to decide
whether the motors are enabled. The SDK and the firmware enforce the same rule on the
wire, so a stale cache can only turn a clean refusal into the firmware's own error
message, never into an unsafe write.

## 6. Deliberately not exposed

| Capability | Where it lives instead | Why |
| --- | --- | --- |
| Trajectory execution (`movej`, `movej_sync`, `move_p`, `home`) | `litearm_ros2_control` + `joint_trajectory_controller` | Motion control belongs to the ros2_control stack, which owns the real-time loop and the joint interfaces. |
| Cartesian servo and teleoperation | MoveIt Servo plus a custom controller (`litearm_servo_control` in the working workspace) | Servo needs a controller in the command path, not a service that hands over one pose at a time. |
| Firmware cartesian planning (`move_l`, `move_c`, `move_path`) | Not wired to ROS | It is a one-shot planning service, and it is serial-only; a future action interface is the right shape, and it must not be mixed with host-side servo on the same arm. |
| Model import, log capture, `kin_bench` | The SDK CLI and the diagnostics scripts | Long-running or bulk transfers that do not belong in a service call. |
| Joint state publication while the control stack runs | `joint_state_broadcaster` | Two publishers on one `joint_states` topic make the arm's state ambiguous; the driver keeps its own publisher off by default. |

## 7. Errors, timeouts and the wire

- One mutually exclusive callback group serialises every SDK call. A command waits for the
  firmware ACK, so a stalled USB link can hold a service call for up to the SDK's own
  timeout (1.2 s); the status publication pauses for that long, which is the price of
  never interleaving two commands on the wire.
- A service that fails returns `success=false` and the SDK's own message, for example a
  clamped register, the firmware's "reset first" refusal for a latched fault, or a refusal
  of `ENABLE` when the licence is not activated.
- The arm is not a real-time system from the host's point of view: this node is for
  maintenance and diagnostics, not for control loops.

## 8. Compatibility

| Item | Requirement |
| --- | --- |
| ROS 2 | Humble |
| Firmware | `Litearm1.5.0` or newer for the status layout and `joint_fault`; `Litearm1.8.0` or newer for the licence record and activation |
| SDK | `litearm-cpp`, found as an installed package or built from the sibling source tree |
| Hardware | USB CDC `VID:PID 1d50:606f`, 24 V supply for motion |

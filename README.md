# litearm-ros2-control

A ros2_control `SystemInterface` plugin that exposes the LiteArm arm as a ros2_control system, for
ROS 2 users who want `joint_trajectory_controller` or MoveIt 2 to drive the real arm over a single
USB cable.

The plugin owns the arm's USB CDC link itself, through the
[litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK. There is no helper process and no
shared memory: `read()` consumes the SDK's cached 100 Hz status frame, and `write()` streams the
command reference back with the firmware's `MOVE_JS` servo. Trajectory planning, kinematics and
dynamics all stay in the firmware — the host sends joint references and reads state back.

## Packages

| Package | What it is |
| --- | --- |
| `litearm_ros2_control` | The ros2_control `SystemInterface` plugin: the control path, driven by `joint_trajectory_controller` or MoveIt 2. |
| `litearm_driver` | A standalone lifecycle node that owns the same USB link and exposes the SDK's administrative command set as services. Run it instead of the control stack, never next to it. |
| `litearm_msgs` | The message and service definitions used by `litearm_driver`. |

The control stack and the driver are mutually exclusive: both open the same serial port and
the SDK takes an exclusive lock on it. The driver's full command set is in [docs/command-set.md](docs/command-set.md), with a
Chinese reference in [docs/command-set.zh-CN.md](docs/command-set.zh-CN.md).

## Highlights

- **One cable, no daemon.** The controller manager talks to the firmware directly.
- **The firmware runs the position loop**, `tau = kp*(q_ref - q) + kd*(dq_ref - dq) + feedforward(q, dq)`,
  with `kp`/`kd` from its own parameter table. The plugin sends no gains and computes no dynamics.
- **Standard interfaces**, so `joint_state_broadcaster` and `joint_trajectory_controller` work
  unchanged.
- **Offline tests.** The whole lifecycle is exercised against the SDK's `FakeTransport`, so the test
  suite needs no arm.

## Interface mapping

Each joint exports three state interfaces and two command interfaces:

| Direction | Interface | Firmware meaning |
| --- | --- | --- |
| command | `position` | `q_ref` (`MOVE_JS`) |
| command | `velocity` | `dq_ref` (`MOVE_JS`) |
| state | `position` | measured `q` |
| state | `velocity` | measured `dq` |
| state | `effort` | measured `tau` — a current estimate, noisy, not for control |

Opt-in diagnostic state interfaces, exported on every joint when
`export_diagnostic_interfaces` is `true` (the default):

| Interface | Meaning |
| --- | --- |
| `temperature_mos` | MOSFET temperature |
| `temperature_coil` | coil temperature |
| `error_code` | firmware fault word |
| `feedback_age` | age of the last status frame, in seconds |

## Requirements

| Item | Requirement |
| --- | --- |
| ROS 2 | Humble |
| Build | `ament_cmake`, C++17 |
| SDK | `litearm-cpp`, either installed or as a sibling source tree |
| Firmware | `Litearm1.5.0` or newer — reported as `Litearm<major.minor.patch>-{7J\|1J}` |
| Connection | USB CDC serial, `VID:PID 1d50:606f` |

Firmware older than 1.5.0 is rejected at connect time: the plugin depends on the 6+21N status frame
layout, the `joint_fault` field, and the enabled bit that 1.5.0 introduced.

On Linux, grant yourself serial port access:

```bash
sudo usermod -aG dialout $USER      # takes effect after you log in again
```

## Build

The package needs the `litearm-cpp` source tree as a sibling, unless an installed `litearm` CMake
package is already on `CMAKE_PREFIX_PATH`:

```text
~/litearm_ws/src/
├── litearm-cpp/              # the C++ SDK
└── litearm-ros2-control/     # this repository
```

```bash
source /opt/ros/humble/setup.bash
cd ~/litearm_ws
colcon build
source install/setup.bash
```

To build against a `litearm-cpp` checkout somewhere else, point CMake at it:

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

Do not build `litearm-cpp` with its default static archive and expect this plugin to link it. A
ros2_control plugin is a shared object, so the SDK it embeds must be position independent. The
package handles this by building the sibling tree as a static **PIC** library and linking it in, so
the installed plugin is self-contained. An installed `litearm` that is a non-PIC static library
makes the build fail with an explanatory error rather than a link error — reinstall the SDK with
`-DBUILD_SHARED_LIBS=ON`, or delete it and let this package use the sibling tree.

## Usage

Declare the component in the URDF's `<ros2_control>` block. The joint names must be `joint1..jointN`,
one per axis, and N must equal the axis count the firmware reports — 7 for the arm, 1 for the bench.
The name fixes the axis mapping, so the order of the `<joint>` elements does not matter.

```xml
<ros2_control name="LitearmSystem" type="system">
  <hardware>
    <plugin>litearm_ros2_control/LitearmSystem</plugin>
    <!-- Empty or omitted: auto-discover by VID:PID 1d50:606f. -->
    <param name="port"></param>
    <param name="export_diagnostic_interfaces">true</param>
    <param name="auto_enable">true</param>
    <param name="disable_on_shutdown">false</param>
    <param name="enable_attempts">12</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- joint2 .. joint7: identical blocks -->
</ros2_control>
```

Every parameter is optional:

| Parameter | Default | Effect |
| --- | --- | --- |
| `port` | empty | USB CDC device path. Empty triggers auto-discovery by `VID:PID 1d50:606f`. |
| `export_diagnostic_interfaces` | `true` | Export the four diagnostic state interfaces. |
| `auto_enable` | `true` | Enable the motors in `on_activate`. Set `false` to control enablement yourself. |
| `disable_on_shutdown` | `false` | Leave the motors enabled on shutdown, so the firmware holds the pose. `true` loses force and the arm drops. |
| `enable_attempts` | `12` | Retries for `enable()`. The SDK sleeps 300 ms between them, and only retryable codes consume an attempt. |

A minimal controller set, using the standard controllers this package depends on:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz; anything at or above 100 satisfies the firmware watchdog

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Streaming contract

The firmware trips a command watchdog after 100 ms without a command, so `write()` has to keep
re-sending. A controller manager update rate of 100 Hz or higher satisfies that automatically.

**Do not let the update loop stall for more than about 100 ms while active.** The firmware drops to
its fail-soft hold and the arm sags toward whatever the load pulls it.

Be aware of the tradeoff in driving the arm in-process: `write()` calls into the SDK, which writes a
frame and waits for the firmware ACK, bounded by the SDK's own 1.2 s timeout. On a healthy link that
is sub-millisecond, but a stalled USB link can stall the controller loop for up to that timeout. If
you need a hard real-time loop isolated from USB, run the arm through a shared-memory daemon variant
instead.

## Maintenance driver

`litearm_driver` is the second way to reach the arm. It opens the USB link itself and
exposes the SDK's administrative calls as services: enable, park, clear faults, enter zero
gravity, tune the speed governor and the feedforward terms, read and write the joint
parameter table, read and activate the licence, and enter DFU.

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
```

**Do not run the driver and the ros2_control stack at the same time.** The driver is for
maintenance and bring-up, not for motion control: trajectories, servo and teleoperation
stay in the control path. See [docs/command-set.md](docs/command-set.md) for every
service, the refusals and the reason for each one.

## Tests

The suite is fully offline — it injects the SDK's `FakeTransport` and drives the whole lifecycle
without an arm attached:

```bash
cd ~/litearm_ws
colcon test --packages-select litearm_ros2_control --event-handlers console_direct+
colcon test-result --verbose
```

## License

Apache-2.0. See [LICENSE](LICENSE).

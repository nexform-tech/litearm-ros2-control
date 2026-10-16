# litearm-ros2-control

A ros2_control `SystemInterface` plugin that exposes the LiteArm arm as a ros2_control system, for
ROS 2 users who want `joint_trajectory_controller` or MoveIt 2 to drive the real arm over a single
USB cable.

The plugin owns the arm's USB CDC link itself, through the
[litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK. There is no helper process and no
shared memory: `read()` consumes the SDK's cached 100 Hz status frame, and `write()` streams the
command reference back with the firmware's `MOVE_JS` servo. Trajectory planning, kinematics and
dynamics all stay in the firmware — the host sends joint references and reads state back.

This repository holds one package: the plugin. The URDF description it launches with lives in its
own repository, [litearm-description](https://github.com/nexform-tech/litearm-description) (package
`litearm_description`). The standalone maintenance driver and its message package live in
[litearm-ros2](https://github.com/nexform-tech/litearm-ros2).

## Packages

| Package | What it is |
| --- | --- |
| `litearm_ros2_control` | The ros2_control `SystemInterface` plugin: the control path, driven by `joint_trajectory_controller` or MoveIt 2. |

The control stack and the maintenance driver are mutually exclusive: both open the same serial port
and the SDK takes an exclusive lock on it. Never run them together. Start with
[docs/quickstart.md](docs/quickstart.md) for the commands that bring the stack up.

The arm's URDF description lives in its own repository,
[`nexform-tech/litearm-description`](https://github.com/nexform-tech/litearm-description)
(package `litearm_description`), and `litearm_ros2_control`'s URDF entry includes
`$(find litearm_description)/urdf/litearm_description.urdf`. That package therefore has to be in
the workspace (or installed) at launch time; this repository no longer vendors a copy. The model is
maintained there, not here.

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
| Display, optional | `rviz2` and `joint_state_publisher_gui`, needed only for `ros2 launch litearm_description display.launch.py` |

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
├── litearm-description/      # the URDF description (package litearm_description)
└── litearm-ros2-control/     # this repository
    └── litearm_ros2_control/ # the plugin
```

Keep a single arm description in the workspace: the litearm-stm32 firmware repository ships its own
`litearm` URDF package, and two arm descriptions in one workspace make it ambiguous which one a
launch resolves.

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

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_ros2_control litearm_control.launch.py
```

That starts `robot_state_publisher`, then `controller_manager` with the `LitearmSystem` component,
then spawns and activates `joint_state_broadcaster` and `joint_trajectory_controller`. Bring-up opens
the arm's USB link and enables the motors, so **it moves real hardware**: support the arm and keep
the emergency stop within reach.

The launch pins the ROS domain to 42 and enables localhost-only discovery, so a shell that has not
exported the same settings sees nothing:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
```

Pass `port:=/dev/ttyACM1` when several boards are attached; empty, the default, auto-discovers by
`VID:PID 1d50:606f`. The remaining launch arguments, the serial-port preflight and a copy-pasteable
trajectory are in [docs/quickstart.md](docs/quickstart.md).

To drive the arm, send a trajectory to
`/joint_trajectory_controller/follow_joint_trajectory`, or plan in MoveIt 2 and let `move_group`
execute it.

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
| `servo_command` | `move_js` | Which command the frames carry: `move_js` (target position plus a slew ceiling) or `joint_follow`. See below. |
| `joint_follow_kp` | `200 200 200 200 80 80 80` | Stiffness per joint, only read when `servo_command` is `joint_follow`. |
| `joint_follow_kd` | `3 5 3 3 1.5 1.5 1.5` | Damping per joint, same condition. Joint 2's 5.0 is the firmware's `MIT_KD_MAX`. |

`servo_command:=joint_follow` sends `q`, `dq`, a stiffness and a damping per joint, and lets
the firmware compute the feedforward (gravity and the joint limit wall) itself. **The
firmware waives its overspeed and position envelope rules for the whole session**, so the
host becomes the only bound on the arm — that is what the mode is for, and it is why it is
not the default. It warns at startup, with the gains, every time it is selected.

Do not reach for it to make the arm faster. Reach for it when something upstream needs to
move the arm at a rate the envelope refuses, and only with the host's own limits in place.

The gain lists are per joint and per joint count: a list whose length does not match the arm
turns the mode off and falls back to `move_js`, rather than sending a command the SDK would
refuse on every tick. The defaults are `DEFAULT_K_P` / `DEFAULT_K_D` from litearm-lerobot's
`servo.py` — the one gain table in this workspace that has run this path on real hardware,
taken there from liteteleop's `SETUP_K` / `SETUP_B`.

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

The name `arm_controller` is arbitrary. The configuration this package ships,
`config/litearm_controllers.yaml`, calls that controller `joint_trajectory_controller`, and that is
the name `litearm_control.launch.py` loads — use it when you address the controller by name.

## Streaming contract

The firmware trips a command watchdog after 100 ms without a command, so `write()` has to keep
re-sending. A controller manager update rate of 100 Hz or higher satisfies that automatically.

**Do not let the update loop stall for more than about 100 ms while active.** The firmware drops to
its fail-soft hold and the arm sags toward whatever the load pulls it.

Be aware of the tradeoff in driving the arm in-process: `write()` calls into the SDK, which writes a
frame and waits for the firmware ACK, bounded by the SDK's own 1.2 s timeout. On a healthy link that
is sub-millisecond, but a stalled USB link can stall the controller loop for up to that timeout. If
you need a hard real-time loop isolated from USB, this in-process design is the wrong tradeoff —
the plugin does not provide one.

## Tests

The suite is fully offline — it injects the SDK's `FakeTransport` and drives the whole lifecycle
without an arm attached:

```bash
cd ~/litearm_ws
colcon test --packages-select litearm_ros2_control --event-handlers console_direct+
colcon test-result --verbose
```

## Related repositories

| Repository | What it holds |
| --- | --- |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | The standalone maintenance driver and the message package it speaks. |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | The C++ SDK this plugin builds against. |

## License

Apache-2.0. See [LICENSE](LICENSE).

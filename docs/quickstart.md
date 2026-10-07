# litearm quickstart

The two ways to reach the arm, in the order you would use them: the ros2_control stack when
you want controllers, MoveIt and RViz; `litearm_driver` when you want the SDK's administrative
calls with no `controller_manager` in the loop. They are mutually exclusive — both open the
same USB CDC port and the SDK takes an exclusive `flock` on it — so start one at a time.

**Do not run both.** The symptom of trying is a launch that cannot find the device, or a
driver whose configure fails: neither process can share the port.

## 0. Before anything

```bash
source /opt/ros/humble/setup.bash
source ~/github/litearm/install/setup.bash

lsusb | grep 1d50          # expect 1d50:606f
ls /dev/ttyACM*            # expect /dev/ttyACM0
```

## 1. The ros2_control stack

**Only the control stack** — `robot_state_publisher`, `controller_manager` and the two
controllers. The manager has to come from a stack that owns the description and the hardware
component; this launch starts the manager itself and therefore needs the URDF entry this
package ships (`urdf/litearm.urdf.xacro`).

```bash
ros2 launch litearm_ros2_control litearm_control.launch.py
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `port` | `''` | USB CDC path. Empty auto-discovers by VID:PID `1d50:606f`; pass `port:=/dev/ttyACM1` when several boards are attached. |
| `clear_faults` | `true` | Clear a latched arm fault while configuring, so a bring-up after a fault is still one command. `false` lets a latched fault stop the launch. |
| `controllers_file` | `''` | Controller parameters. Empty uses the packaged `config/litearm_controllers.yaml`. |
| `start_joint_state_broadcaster` | `true` | Spawn and activate `joint_state_broadcaster`. |
| `start_joint_trajectory_controller` | `true` | Spawn and activate `joint_trajectory_controller`. |
| `ros_domain_id` | `42` | ROS domain of this stack. Deliberately not 0. |
| `ros_localhost_only` | `true` | Localhost-only discovery: a second robot on the network otherwise shows up in RViz and as a second `/move_action` server. |

**Control stack plus MoveIt and RViz** — one command; it includes the launch above.

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
ros2 launch litearm_moveit_config litearm_moveit.launch.py use_rviz:=false
ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false
```

The second is for a machine without a display; the third is for when the control stack is
already running somewhere else and only `move_group` and RViz are missing.

### Seeing the stack from another terminal

Both launches pin the domain to 42 and enable localhost-only discovery, so a plain shell sees
nothing until it exports the same settings:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
ros2 node list
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

`ros2 control list_controllers` needs `ros-humble-ros2controlcli`, which is not a dependency
of this repository; the service call above needs nothing extra.

## 2. The driver

`litearm_driver` is the standalone maintenance node: it owns the USB link itself and exposes
the whole SDK command set as services under `/litearm`. No `controller_manager`, no
controllers, no MoveIt.

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `namespace` | `litearm` | Namespace of the services; they live directly in it. |
| `autostart` | `true` | Configure and activate the node. `false` starts it inactive for manual lifecycle driving. |
| `config` | packaged `config/litearm_driver.yaml` | Parameter file. |
| `log_level` | `info` | Log level of the node. |

### Read-only bring-up

`auto_enable` lives in the parameter file and defaults to `true`. To look at an arm without
energising it:

```bash
cp install/litearm_driver/share/litearm_driver/config/litearm_driver.yaml /tmp/driver_no_enable.yaml
sed -i 's/auto_enable: true/auto_enable: false/' /tmp/driver_no_enable.yaml
ros2 launch litearm_driver litearm_driver.launch.py config:=/tmp/driver_no_enable.yaml
```

The driver does not pin a ROS domain, so a normal shell reaches it directly.

### Calls that do not move the arm

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{}"
ros2 service call /litearm/get_diagnostics litearm_msgs/srv/GetDiagnostics "{}"
ros2 service call /litearm/get_tcp litearm_msgs/srv/GetTcp "{}"
ros2 service call /litearm/get_feedforward_mask litearm_msgs/srv/GetFeedforwardMask "{}"
ros2 service call /litearm/get_feedforward_catalog litearm_msgs/srv/GetFeedforwardCatalog "{}"
ros2 service call /litearm/inverse_kinematics litearm_msgs/srv/InverseKinematics \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], seed: [], timeout: 0.5}"
ros2 service call /litearm/log_start litearm_msgs/srv/LogStart "{ticks: 600}"
ros2 service call /litearm/log_dump litearm_msgs/srv/LogDump \
  "{filename: tick_log.bin, wait: true, timeout: 3.0}"
```

`get_feedforward_catalog` answers from the SDK's static tables and needs no session at all;
`log_dump` writes inside the node's `log_dir` (`$HOME/.ros/litearm` by default) and refuses a
path as `filename`.

### Moving the arm

Every service that can energise the motors is refused until `allow_motion` is set. It is read
on every call, like `allow_dfu`, so no restart is needed:

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

`speed` is a fraction of full speed, `0 < speed <= 1`; `30` means 30×, not 30 percent. The
percentage governor is `/litearm/set_speed_scaling`, and the firmware's own `speed_scaling`
field in the status message is a host-side record of the last write.

⚠ Support the arm before enabling anything that moves it, and keep the emergency stop within
reach. `move_js`, `send_mit` and `send_mit_all` are single frames: the firmware's 100 ms
watchdog takes the arm back to its hold unless a caller keeps re-sending.

## 3. Switching between them

```bash
# In the terminal running the stack: Ctrl-C, and wait for the processes to exit.
fuser -v /dev/ttyACM0 2>/dev/null      # who still holds the port, if anything
```

The two cannot overlap because of the port, not because of a ROS resource: `litearm_driver`
does not take the shared-memory lock of the other deployment in this workspace.

## 4. When it does not come up

| Symptom | Cause and what to do |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | The board is unplugged, off, or in DFU. Check `lsusb`; a board in DFU enumerates as `0483:DF11` and needs a firmware image. |
| Configure fails while another stack ran a moment ago | The port is still held. Stop the other process and retry; nothing else can share it. |
| `ENABLE was refused: ... ERR{0x10,0x06}` | The firmware has a latched fault (EMERGENCY or `joint_fault`). The control launch clears it while configuring unless `clear_faults:=false`; the driver reports the licence fields in its status message if that is the cause. |
| `First command frame failed: ... ERR{0x03,0x02}` | The arm was not energized yet when the first frame went out. Fixed in the component (it waits for the status frame's enabled bit); if it reappears, the firmware refused a zero-`dq` frame whose target is more than 5 mrad from the measurement. |
| `Kinematics solver could not be instantiated` | The solver named in `litearm_moveit_config/config/kinematics.yaml` is not installed. KDL ships with MoveIt; SNS-IK needs its two packages in the workspace. |

## 5. Where to go next

- [`docs/command-set.md`](command-set.md) — every driver service, its refusals and the wire
  semantics behind them (Chinese: [`docs/command-set.zh-CN.md`](command-set.zh-CN.md)).
- [`litearm_driver/README.md`](../litearm_driver/README.md) — the driver's parameters, topics
  and field-level service reference.
- [`litearm_ros2_control/README.md`](../litearm_ros2_control/README.md) — the hardware
  component, the deployment files it ships and the controller configuration.
- [`litearm_moveit_config`](https://github.com/thetooler/litearm-moveit2) — the MoveIt
  configuration this quickstart's second entry point belongs to.

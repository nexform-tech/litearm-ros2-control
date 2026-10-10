# litearm ros2_control quickstart

The commands that bring the ros2_control stack up on a real arm, for ROS 2 users who want
`joint_trajectory_controller` or MoveIt 2 to drive it. Each section is a step you run in order;
section 4 is what to read when one of them fails.

The maintenance driver is not in this repository. If you want the SDK's administrative calls with
no `controller_manager` in the loop, use
[litearm-ros2](https://github.com/nexform-tech/litearm-ros2). The two entry points are mutually
exclusive — both open the same USB CDC port and the SDK takes an exclusive `flock` on it — so run
one at a time.

**Do not run the driver while this stack is running.** The symptom is a launch that cannot find the
device, or a driver whose configure fails: neither process can share the port.

## 0. Before anything

```bash
source /opt/ros/humble/setup.bash
source ~/litearm_ws/install/setup.bash

lsusb | grep 1d50          # expect 1d50:606f
ls /dev/ttyACM*            # expect /dev/ttyACM0
ros2 pkg prefix litearm    # expect <workspace>/install/litearm
```

That last one is the URDF description. It ships inside litearm-ros2-control, so nothing extra to
install — but if it is missing, the launch below fails while building the `robot_description`.

## 1. The control stack

**Only the control stack** — `robot_state_publisher`, `controller_manager` and the two controllers.
The manager has to come from a stack that owns the description and the hardware component; this
launch starts the manager itself and therefore needs the URDF entry this package ships
(`urdf/litearm.urdf.xacro`), which pulls in the `litearm` description from the same repository.

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

The second is for a machine without a display; the third is for when the control stack is already
running somewhere else and only `move_group` and RViz are missing.

### Seeing the stack from another terminal

Both launches pin the domain to 42 and enable localhost-only discovery, so a plain shell sees nothing
until it exports the same settings:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
ros2 node list
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

`ros2 control list_controllers` needs `ros-humble-ros2controlcli`, which is not a dependency of this
repository; the service call above needs nothing extra.

## 2. Handing the arm back

```bash
# In the terminal running the stack: Ctrl-C, and wait for the processes to exit.
fuser -v /dev/ttyACM0 2>/dev/null      # who still holds the port, if anything
```

The two deployments cannot overlap because of the port, not because of a ROS resource.

## 3. Driving it

Once the stack is up, the arm is an ordinary ros2_control system: send `joint_trajectory_controller`
trajectories, or plan in MoveIt 2 and let `move_group` execute them. The two controllers the launch
starts are `joint_state_broadcaster` and `arm_controller` (a
`joint_trajectory_controller`); the joints are `joint1`..`joint7`.

## 4. When it does not come up

| Symptom | Cause and what to do |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | The board is unplugged, off, or in DFU. Check `lsusb`; a board in DFU enumerates as `0483:DF11` and needs a firmware image. |
| Configure fails while the driver ran a moment ago | The port is still held. Stop the other process and retry; nothing else can share it. |
| `ENABLE was refused: ... ERR{0x10,0x06}` | The firmware has a latched fault (EMERGENCY or `joint_fault`). The control launch clears it while configuring unless `clear_faults:=false`. |
| `First command frame failed: ... ERR{0x03,0x02}` | The arm was not energized yet when the first frame went out. Fixed in the component (it waits for the status frame's enabled bit); if it reappears, the firmware refused a zero-`dq` frame whose target is more than 5 mrad from the measurement. |
| `Kinematics solver could not be instantiated` | The solver named in `litearm_moveit_config/config/kinematics.yaml` is not installed. KDL ships with MoveIt; SNS-IK needs its two packages in the workspace. |
| No data on `/joint_states` | `joint_state_broadcaster` is not active, or the arm is in a fault. Check the controller state and the launch log. |

## 5. Where to go next

- [`README.md`](../README.md) — the hardware component, its parameters, the deployment files it
  ships and the controller configuration (Chinese: [`README.zh-CN.md`](../README.zh-CN.md)).
- [`litearm_moveit_config`](https://github.com/thetooler/litearm-moveit2) — the MoveIt configuration
  this quickstart's second entry point belongs to.
- [`litearm-ros2`](https://github.com/nexform-tech/litearm-ros2) — the maintenance driver, for
  bring-up, licensing and parameter tuning outside the control loop.

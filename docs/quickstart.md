# litearm ros2_control quickstart

Everything needed to take a LiteArm from an empty machine to a moving joint under ros2_control, for
ROS 2 users who want `joint_trajectory_controller` or MoveIt 2 to drive the real arm. Work through it
in order; [When it does not come up](#when-it-does-not-come-up) is what to read when a step fails.

**This stack drives real hardware and has no dry-run mode.** The hardware component opens the arm's
USB link and enables the motors while it configures, so the launch below moves a real arm. Support
it, clear the workspace around it, and keep the emergency stop within reach.

The maintenance driver is not in this repository. If you want the SDK's administrative calls with no
`controller_manager` in the loop, use
[litearm-ros2](https://github.com/nexform-tech/litearm-ros2). The two entry points are mutually
exclusive — both open the same USB CDC port and the SDK takes an exclusive `flock` on it — so run one
at a time.

**Do not run the driver while this stack is running.** The symptom is a launch that cannot find the
device, or a driver whose configure fails: neither process can share the port.

## Quickstart

Create a colcon workspace, put the packages in `src/`, build from the workspace root, launch. Pick
the track that matches your machine — both end at the same commands.

| Track | Use it when |
| --- | --- |
| [Quickstart with Docker](#quickstart-with-docker) | ROS 2 is not installed on the host, or you would rather not install it |
| [Quickstart without Docker](#quickstart-without-docker) | ROS 2 Humble is already installed on the host |

### What you need

| Item | Requirement |
| --- | --- |
| OS | Ubuntu 22.04 (jammy), x86_64 |
| ROS 2 | Humble |
| Docker | any recent version, for the container track only |
| Disk | about 8 GB for the container track; about 3 GB for the host track, most of it the ROS 2 install itself |
| Arm | required: this stack has no dry run |
| Firmware | `Litearm1.5.0` or newer, reported as `Litearm<major.minor.patch>-{7J\|1J}` |
| Connection | USB CDC serial, `VID:PID 1d50:606f` |
| Serial access | your user in the `dialout` group; in the container track, the container in it |
| Display | optional, for RViz only |

Firmware older than 1.5.0 is rejected at connect time: the plugin depends on the 6+21N status frame
layout, the `joint_fault` field, and the enabled bit that 1.5.0 introduced.

### The workspace layout

Both tracks produce this layout, because both clone into `src/` and build from the workspace root:

```text
~/litearm_ws/          <- workspace root: run every colcon command here
├── src/               <- the packages go here, and only here
│   ├── litearm-cpp/
│   ├── litearm-description/
│   └── litearm-ros2-control/
├── build/             <- colcon output; never edit, never commit
├── install/
└── log/
```

**Do not run `colcon build` from inside `src/`.** It has to run in the workspace root, so that it
finds `src/` and writes `build/`, `install/` and `log/` beside it. Run it from `src/` and you get a
workspace nested inside your source tree, after which every `source install/setup.bash` resolves to
the wrong prefix.

### What goes in `src/`

Three repositories. All three are needed:

| Cloned into `src/` | Package it provides | Why it is needed |
| --- | --- | --- |
| `litearm-ros2-control` | `litearm_ros2_control` | this repository: the plugin, its URDF entry point and the control launch |
| `litearm-description` | `litearm_description` | the URDF and the meshes; `litearm_ros2_control`'s URDF entry includes `$(find litearm_description)/urdf/litearm_description.urdf` |
| `litearm-cpp` | none — plain CMake, no `package.xml` | the SDK the plugin links against |

`litearm-cpp` has to be a **sibling of this repository** under `src/`: the package's
`CMakeLists.txt` defaults `LITEARM_CPP_DIR` to `<package>/../../litearm-cpp`. To build against a
checkout somewhere else, point CMake at it instead of moving the tree:

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

**Do not add `litearm-moveit2` unless you want MoveIt.** It is needed by the third launch command in
[Running the MoveIt stack](#running-the-moveit-stack) and by nothing else here.

## Quickstart with Docker

### 1. Build the run image

No image for this stack is published, so build one. It is the public `osrf/ros:humble-desktop-full`
image plus the packages this workspace needs: `desktop-full` ships RViz, xacro,
`robot_state_publisher` and colcon, but no ros2_control.

```bash
docker build -t litearm-control:humble - <<'EOF'
FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-ros2controlcli \
 && rm -rf /var/lib/apt/lists/*
EOF
```

`ros-humble-ros2controlcli` is what provides the `ros2 control` subcommands used below. It is not a
dependency of this repository, and without it you fall back on the `controller_manager_msgs`
services, which need nothing extra.

This takes a few minutes and produces an image of about 5.5 GB.

### 2. Create the workspace and clone the packages (on the host)

Clone on the host, not inside the container: the workspace is a bind mount, so it outlives `--rm`,
and you keep your clone and your editor outside Docker.

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

The three packages are described [above](#what-goes-in-src).

**Do not point a `-v` mount at a directory that does not exist yet.** Docker creates it for you,
empty and owned by root, and the container then fails with
`install/setup.bash: No such file or directory` and
`Package 'litearm_ros2_control' not found`. The `mkdir -p` above is what prevents that.

### 3. Build the workspace in the container

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$HOME/litearm_ws:/ws" \
  litearm-control:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    echo "build ok"'
```

`--user "$(id -u):$(id -g)"` and `-e HOME=/tmp` keep the container from writing root-owned files into
your workspace. Without them `build/` and `install/` come back owned by root, and the next host-side
edit or `git clean` fails on permission.

**Do not add `--symlink-install`.** It fails in this workspace:

```text
CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:272 (message):
  ament_cmake_symlink_install_targets() can't find
  '/ws/build/litearm_ros2_control/litearm-cpp/litearm-cpp'
```

`litearm_ros2_control` pulls the `litearm-cpp` source tree in with
`add_subdirectory(... EXCLUDE_FROM_ALL)`, but `litearm-cpp` registers an unconditional
`install(TARGETS litearm-cpp ...)` for its command-line tool. Plain CMake skips installing a target
that was never built; ament's symlink-install macros assert the file exists instead. Plain
`colcon build` is what this workspace supports.

### 4. Run the control stack in the container

```bash
docker run --rm -it --name litearm-control \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  --device /dev/ttyACM0 \
  --group-add "$(getent group dialout | cut -d: -f3)" \
  -v "$HOME/litearm_ws:/ws" \
  litearm-control:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch litearm_ros2_control litearm_control.launch.py'
```

- `--device /dev/ttyACM0` hands the arm's USB CDC device to the container, and `--group-add` puts the
  container in the group that owns it. Without either, configure fails while opening the port.
- `--name litearm-control` is what lets the commands in the rest of this document reach this
  container with `docker exec`.
- The container is on Docker's default bridge network, and the launch pins discovery to loopback, so
  the stack is visible **only from inside this container**. That is why every later command is a
  `docker exec` into it. Add `--net=host` if the host also has ROS 2 installed and you want a host
  terminal to see the stack.

A successful bring-up logs, in this order:

```text
[LitearmSystem]: Initialized: 7 joints, port='auto-discover', diagnostics=on, auto_enable=on, reset_faults_on_configure=on.
[LitearmSystem]: Configured: firmware=Litearm1.10.0-7J port=/dev/ttyACM0 axes=7.
[LitearmSystem]: Activated: command reference seeded from the measured position, motors enabled.
[controller_manager]: update rate is 100 Hz
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
```

The version string will differ. `Configured:` is the line that proves the board answered; if it is
missing, the arm is not reachable and [When it does not come up](#when-it-does-not-come-up) applies.

## Quickstart without Docker

### 1. Install ROS 2 Humble

Follow the official instructions for Ubuntu 22.04 (jammy) and choose the `ros-humble-desktop`
variant:

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Then add what this workspace needs on top of it:

```bash
sudo apt install ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2controlcli \
  ros-dev-tools
```

`ros-humble-ros2controlcli` provides the `ros2 control` subcommands; it is not a dependency of this
repository. Without it, use the `controller_manager_msgs` services instead.

**`colcon` is not in Ubuntu's own repositories.** On jammy, `sudo apt install colcon` fails with
`E: Unable to locate package colcon` and `sudo apt install python3-colcon-common-extensions` fails
the same way — colcon ships from the ROS 2 apt repository, and `ros-dev-tools` is the package that
pulls it in. There is no colcon to install until ROS 2 itself is installed.

Grant yourself access to the serial port:

```bash
sudo usermod -aG dialout "$USER"    # takes effect after you log in again
```

### 2. Create the workspace and clone the packages

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

The three packages are described [above](#what-goes-in-src).

### 3. Build the workspace

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

As in the container track, build from `~/litearm_ws`, never from `src/`, and do not pass
`--symlink-install`. The reasons are [above](#3-build-the-workspace-in-the-container).

### 4. Run the control stack

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_ros2_control litearm_control.launch.py
```

## Launch arguments

Both tracks run the same launch, and its arguments are below. It starts `robot_state_publisher`,
`controller_manager` and the two controllers. The manager has to come from a stack that owns both
the description and the hardware component, which is why this launch starts the manager itself and
needs the URDF entry this package ships (`urdf/litearm.urdf.xacro`).

| Argument | Default | Meaning |
| --- | --- | --- |
| `port` | `''` | USB CDC path. Empty auto-discovers by VID:PID `1d50:606f`; pass `port:=/dev/ttyACM1` when several boards are attached. |
| `controllers_file` | `''` | Controller parameter file. Empty uses the packaged `config/litearm_controllers.yaml`. |
| `start_joint_state_broadcaster` | `true` | Spawn and activate `joint_state_broadcaster`. |
| `start_joint_trajectory_controller` | `true` | Spawn and activate `joint_trajectory_controller`. |
| `clear_faults` | `true` | Clear a latched arm fault while configuring, so a bring-up after a fault is still one command. `false` lets a latched fault stop the launch. |
| `ros_domain_id` | `42` | ROS domain of this stack. Deliberately not 0. |
| `ros_localhost_only` | `true` | Localhost-only discovery: a second robot on the network otherwise shows up in RViz and as a second `/move_action` server. |

### Running the MoveIt stack

One command brings up the control stack **and** `move_group` with RViz. It needs
[litearm-moveit2](https://github.com/nexform-tech/litearm-moveit2) in the workspace, and the MoveIt
packages installed.

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
ros2 launch litearm_moveit_config litearm_moveit.launch.py use_rviz:=false
ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false
```

The second is for a machine without a display. The third is for when the control stack is already
running and only `move_group` and RViz are missing — **do not use it while this launch's own control
stack is up**, because two `controller_manager`s would fight over the same command interfaces.

## Seeing the stack from another terminal

Both launch files pin the ROS domain to 42 and enable localhost-only discovery, so nothing sees the
stack until it exports the same settings. In the container track, open the shell first:

```bash
docker exec -it litearm-control bash
```

Then, in either track:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # or /ws/install/setup.bash inside the container
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1

ros2 control list_hardware_components    # LitearmSystem, active
ros2 control list_controllers            # both controllers, active
ros2 control list_hardware_interfaces    # position/velocity command + state per joint
ros2 topic echo /joint_states --once     # data here means the firmware link is live
```

Without `ros-humble-ros2controlcli`, the same information comes from services:

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
ros2 service call /controller_manager/list_hardware_components \
  controller_manager_msgs/srv/ListHardwareComponents "{}"
```

**Do not read `/joint_states` positionally.** The order of the `name` array is not
`joint1`..`joint7` — it comes back in an arbitrary order, for instance
`joint2, joint3, joint5, joint6, joint1, joint4, joint7`. Match values to joints by name.

If a `ros2` command still sees nothing, try it with `--no-daemon`: the CLI otherwise answers from a
cached daemon holding the discovery settings from before your export.

## Moving the arm

ros2_control accepts joint targets, not Cartesian poses. The action is
`/joint_trajectory_controller/follow_joint_trajectory`, of type
`control_msgs/action/FollowJointTrajectory` — the same one `move_group` uses, so anything you send
here is what MoveIt would have sent.

### Pick a target

Read where the arm is before choosing one, and check the target against the joint limits in
[litearm-moveit2](https://github.com/nexform-tech/litearm-moveit2)'s
`litearm_moveit_config/config/joint_limits.yaml`.

| State | Joint values | Use it when |
| --- | --- | --- |
| `ready` | all joints 0 except `joint4` at `-1.5708` | the default choice: elbow up, well inside every limit |
| `zero` | all joints 0 | you specifically want the firmware zero; `joint4` lands 17 mrad from its upper limit and the arm is near a singularity |

**Do not send an arbitrary configuration without checking it first.** A target outside a joint's
range is refused or, worse, is reached by a route you did not intend. `joint4` in particular has a
narrow range, `-3.0715` to `+0.0175` rad.

**Do not start from a folded arm without a plan for the sweep.** The arm parks wherever it was left;
that is often `joint4` at its lower limit, and a move to `ready` unfolds the elbow through roughly
90 degrees.

### Send it

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    points:
      - positions: [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]
        time_from_start: {sec: 5, nanosec: 0}" --feedback
```

In the container track, this is that command with everything around it, so there is nothing to set up
first — paste it into any host terminal while the stack is running:

```bash
docker exec -it litearm-control bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
"trajectory:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
  points:
    - positions: [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]
      time_from_start: {sec: 5, nanosec: 0}" --feedback'
```

The YAML starts at the left margin inside the quotes. Keep it there: its indentation is relative to
`trajectory`, and re-indenting it to line up with the `ros2` line above breaks the parse.

**Do not wrap the goal in braces.** The help text shows `'{order: 10}'`, which is a single-line flow
mapping; mixing it with block-style keys below fails to parse:

```text
yaml.parser.ParserError: while parsing a flow mapping
  in "<unicode string>", line 1, column 1:
    {trajectory:
    ^
expected ',' or '}', but got ':'
```

Leave the outer `{` and `}` off, keep the inner `{sec: 5, nanosec: 0}` (that one is a valid
single-line flow mapping), and put no trailing commas after a list or a mapping.

**Do not hand-write `velocities` to slow the arm down.** The controller computes velocities from its
own spline and sends both interfaces, which is what the firmware needs. Filling in zeros instead
trips the trap documented in `config/litearm_controllers.yaml`: `velocity` is the slew-rate limit of
the position reference, so the firmware moves `q_ref` toward the target by at most `|dq_ref|·dt` per
cycle, and a zero `dq_ref` freezes the reference. The arm does not move, and nothing reports an
error.

`time_from_start` is a time budget for the whole motion, not a speed. Five seconds for a 1.5 rad
travel on `joint4` is about 0.3 rad/s, comfortably under that joint's 1.75 rad/s ceiling.

### Read the result

`--feedback` streams the measured positions until the goal completes, then prints the result:

| `error_code` | Meaning |
| --- | --- |
| `0` (SUCCESSFUL) | reached the target, within the 0.02 rad goal tolerance per joint |
| `1` (INVALID_GOAL) | rejected: joint names or dimensions do not match the controller |
| `-4` (PATH_TOLERANCE_VIOLATED) | fell behind the path; the arm is blocked or moving against a limit |
| `-5` (GOAL_TOLERANCE_VIOLATED) | stopped short: blocked, or a joint hit its limit |

A `ros2 topic pub` on `/joint_trajectory_controller/joint_trajectory` sends the same trajectory
type, but it has no result code and no timeout. **Do not use it for bring-up**: when something goes
wrong you cannot tell a rejected goal from a lost message.

### Driving a Cartesian pose

There is no IK in this stack. `ros2 service list` shows `controller_manager` and the two
controllers, and nothing else — so a Cartesian target has to be resolved into joint angles first:

- **Through MoveIt.** Start [the MoveIt stack](#running-the-moveit-stack), then call `/compute_ik`
  with `moveit_msgs/srv/GetPositionIK`, or plan in RViz and let `move_group` execute. The IK solver
  is the one in `litearm_moveit_config/config/kinematics.yaml`, so the result matches what RViz
  plans.
- **Through the SDK.** `litearm-cpp` carries its own kinematics, but it is not exposed on the ROS
  interface, so this needs a small node of your own.

## Handing the arm back

```bash
# In the terminal running the stack: Ctrl-C, and wait for the processes to exit.
fuser -v /dev/ttyACM0 2>/dev/null      # who still holds the port, if anything
```

The two deployments cannot overlap because of the port, not because of a ROS resource.

The component's `disable_on_shutdown` defaults to `false`, so **the firmware keeps holding the last
pose after the stack exits** — the arm does not go limp, and it is not safe to assume it will.
Ctrl-C frees the serial port, which is what lets the maintenance driver run next; it does not
de-energize the arm.

## When it does not come up

| Symptom | Cause and what to do |
| --- | --- |
| `install/setup.bash: No such file or directory`, then `Package 'litearm_ros2_control' not found` | The container's `-v` mount points at a workspace that was never built, or at a path that did not exist and was created empty by Docker. Check that the host directory has `src/`, `build/` and `install/`. |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | The board is unplugged, off, or in DFU. Check `lsusb`; a board in DFU enumerates as `0483:DF11` and needs a firmware image. |
| Configure fails while the driver ran a moment ago | The port is still held. Stop the other process and retry; nothing else can share it. |
| Configure fails on the port in a container | The device was not passed through, or the container is not in the group that owns it. Both `--device` and `--group-add` are required. |
| `ENABLE was refused: ... ERR{0x10,0x06}` | The firmware has a latched fault (EMERGENCY or `joint_fault`). The control launch clears it while configuring unless `clear_faults:=false`. |
| `First command frame failed: ... ERR{0x03,0x02}` | The arm was not energized yet when the first frame went out. Fixed in the component (it waits for the status frame's enabled bit); if it reappears, the firmware refused a zero-`dq` frame whose target is more than 5 mrad from the measurement. |
| `Package 'litearm_ros2_control' not found` on the host | The workspace was built from inside `src/`, so `install/` is in the wrong place. Rebuild from the workspace root. |
| A `docker exec` shell sees no nodes | `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1` are not exported in that shell. |
| No data on `/joint_states` | `joint_state_broadcaster` is not active, or the arm is in a fault. Check the controller state and the launch log. |
| `Kinematics solver could not be instantiated` | The solver named in `litearm_moveit_config/config/kinematics.yaml` is not installed. KDL ships with MoveIt; SNS-IK needs its two packages in the workspace. |

## Where to go next

- [`README.md`](../README.md) — the hardware component, its parameters, the deployment files it
  ships and the controller configuration (Chinese: [`README.zh-CN.md`](../README.zh-CN.md)).
- [`litearm_moveit2`](https://github.com/nexform-tech/litearm-moveit2) — the MoveIt configuration
  behind the combined launch, and its own quickstart.
- [`litearm-ros2`](https://github.com/nexform-tech/litearm-ros2) — the maintenance driver, for
  bring-up, licensing and parameter tuning outside the control loop.

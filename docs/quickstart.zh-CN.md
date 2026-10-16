# litearm ros2_control 快速上手

把一台 LiteArm 从空机器带到 ros2_control 下能动一个关节所需的全部内容，面向希望用
`joint_trajectory_controller` 或 MoveIt 2 驱动真机的 ROS 2 用户。请按顺序执行；某一步失败时看
《起不来时》一节。

**本栈驱动真机，没有空跑模式。** 硬件组件在配置阶段就会打开机械臂的 USB 链路并使能电机，因此下面的
launch 会真的让机械臂动起来。请托住臂、清空周围工作空间、把急停放在手边。

维护驱动不在本仓库里。如果你要的是链路里没有 `controller_manager` 的 SDK 管理类调用，请用
[litearm-ros2](https://github.com/nexform-tech/litearm-ros2)。两个入口**互斥**——都要打开同一个
USB CDC 串口，SDK 会对它加独占 `flock`——所以一次只跑一个。

**不要在本栈运行时跑 driver。** 症状是 launch 找不到设备，或 driver 的 configure 失败：没有第二个
进程能共用这个串口。

## 快速上手

建一个 colcon 工作区，把各个包放进 `src/`，在工作区根目录编译，然后启动。按你的机器选一条路径——
两条最后落在同样的命令上。

| 路径 | 适用场合 |
| --- | --- |
| [用容器快速上手](#用容器快速上手) | 宿主机没装 ROS 2，或者你不想装 |
| [不用容器快速上手](#不用容器快速上手) | 宿主机已经装好 ROS 2 Humble |

### 需要什么

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04 (jammy)，x86_64 |
| ROS 2 | Humble |
| Docker | 任意较新版本，仅容器路径需要 |
| 磁盘 | 容器路径约 8 GB，宿主机路径约 3 GB（其中大部分是 ROS 2 本身） |
| 机械臂 | 必需：本栈没有空跑模式 |
| 固件 | `Litearm1.5.0` 或更新，版本串形如 `Litearm<major.minor.patch>-{7J\|1J}` |
| 连接 | USB CDC 串口，`VID:PID 1d50:606f` |
| 串口权限 | 你的用户在 `dialout` 组里；容器路径下则是容器在该组里 |
| 显示 | 可选，仅 RViz 需要 |

低于 1.5.0 的固件在连接时就被拒绝：插件依赖 1.5.0 才引入的 6+21N 状态帧布局、`joint_fault` 字段
和 enabled 位。

### 工作区布局

两条路径的结果是同一套布局，因为两边都是克隆进 `src/`、再从工作区根目录编译：

```text
~/litearm_ws/          <- 工作区根目录：所有 colcon 命令都在这里跑
├── src/               <- 各个包只放在这里
│   ├── litearm-cpp/
│   ├── litearm-description/
│   └── litearm-ros2-control/
├── build/             <- colcon 产物；不要手改，不要提交
├── install/
└── log/
```

**不要在 `src/` 里面跑 `colcon build`。** 它必须在工作区根目录跑，这样才能找到 `src/`，并把
`build/`、`install/`、`log/` 建在它旁边。在 `src/` 里跑会得到一个嵌在源码树里的工作区，之后每一次
`source install/setup.bash` 都会指向错误的 prefix。

### src/ 里放什么

三个仓库，缺一不可：

| 克隆到 `src/` | 提供的包 | 为什么需要 |
| --- | --- | --- |
| `litearm-ros2-control` | `litearm_ros2_control` | 本仓库：插件、它的 URDF 入口与控制 launch |
| `litearm-description` | `litearm_description` | URDF 与网格文件；`litearm_ros2_control` 的 URDF 入口会 include `$(find litearm_description)/urdf/litearm_description.urdf` |
| `litearm-cpp` | 无——纯 CMake，没有 `package.xml` | 插件链接的 SDK |

`litearm-cpp` 必须是本仓库在 `src/` 下的**同级目录**：本包的 `CMakeLists.txt` 把
`LITEARM_CPP_DIR` 默认设为 `<package>/../../litearm-cpp`。要针对别处的 checkout 编译，不要去挪那棵树，
而是直接告诉 CMake：

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

**不需要 MoveIt 就不要放 `litearm-moveit2`。** 只有《启动 MoveIt 栈》那一节的第三条命令要用到它，
本文其余部分都用不到。

## 用容器快速上手

### 1. 构建运行镜像

本栈没有已发布的镜像，所以要自己构建一个。它就是在公开的 `osrf/ros:humble-desktop-full` 之上加上本
工作区需要的包：`desktop-full` 自带 RViz、xacro、`robot_state_publisher` 和 colcon，但没有
ros2_control。

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

`ros-humble-ros2controlcli` 提供下面用到的 `ros2 control` 子命令。它不是本仓库的依赖；没装的话就退回
用 `controller_manager_msgs` 的服务，那些不需要额外安装。

构建要几分钟，产出的镜像约 5.5 GB。

### 2. 建工作区并克隆各包（在宿主机上）

在宿主机上克隆，不要在容器里：工作区是 bind mount，能比 `--rm` 活得久，你的 clone 和编辑器也就留在
了 Docker 外面。

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

三个包的作用见上文《src/ 里放什么》。

**不要把 `-v` 指向一个还不存在的目录。** Docker 会替你把它建出来——空的、root 属主——容器随后会报
`install/setup.bash: No such file or directory` 和 `Package 'litearm_ros2_control' not found`。上面那句
`mkdir -p` 就是用来避免这个的。

### 3. 在容器里编译工作区

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

`--user "$(id -u):$(id -g)"` 和 `-e HOME=/tmp` 是为了不让容器往你的工作区里写 root 属主的文件。少了
它们，`build/` 和 `install/` 回来就是 root 属主，之后在宿主机上编辑或 `git clean` 都会因权限失败。

**不要加 `--symlink-install`。** 它会在这个工作区里失败：

```text
CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:272 (message):
  ament_cmake_symlink_install_targets() can't find
  '/ws/build/litearm_ros2_control/litearm-cpp/litearm-cpp'
```

`litearm_ros2_control` 用 `add_subdirectory(... EXCLUDE_FROM_ALL)` 把 `litearm-cpp` 源码树拉进来，而
`litearm-cpp` 为它的命令行工具注册了一条无条件的 `install(TARGETS litearm-cpp ...)`。纯 CMake 会跳过
安装一个从未构建过的 target；ament 的 symlink-install 宏则断言这个文件存在。本工作区支持的就是朴素的
`colcon build`。

### 4. 在容器里启动控制栈

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

- `--device /dev/ttyACM0` 把机械臂的 USB CDC 设备交给容器，`--group-add` 让容器进入拥有它的那个组。
  两者缺一，configure 阶段都会打不开串口。
- `--name litearm-control` 是为了让本文后面的命令能用 `docker exec` 找到这个容器。
- 容器在 Docker 默认的 bridge 网络上，而这条 launch 把发现范围钉在 loopback，所以这套栈**只有在这个
  容器内部才看得见**。这就是后面每条命令都写成 `docker exec` 进容器的原因。如果宿主机上也装了 ROS 2
  并且想让宿主机的终端看到这套栈，再加 `--net=host`。

一次成功的启动会依次打出：

```text
[LitearmSystem]: Initialized: 7 joints, port='auto-discover', diagnostics=on, auto_enable=on, reset_faults_on_configure=on.
[LitearmSystem]: Configured: firmware=Litearm1.10.0-7J port=/dev/ttyACM0 axes=7.
[LitearmSystem]: Activated: command reference seeded from the measured position, motors enabled.
[controller_manager]: update rate is 100 Hz
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
```

版本串会不一样。`Configured:` 那一行是板子确实应答了的证据；它没出现，说明机械臂不通，适用《起不来
时》一节。

## 不用容器快速上手

### 1. 安装 ROS 2 Humble

按官方说明为 Ubuntu 22.04 (jammy) 安装，选 `ros-humble-desktop` 变体：

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

然后在它之上补上本工作区需要的包：

```bash
sudo apt install ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2controlcli \
  ros-dev-tools
```

`ros-humble-ros2controlcli` 提供 `ros2 control` 子命令；它不是本仓库的依赖。没装的话改用
`controller_manager_msgs` 的服务。

**`colcon` 不在 Ubuntu 自己的软件源里。** 在 jammy 上，`sudo apt install colcon` 会失败并报
`E: Unable to locate package colcon`，`sudo apt install python3-colcon-common-extensions` 同样失败
——colcon 由 ROS 2 的 apt 源提供，`ros-dev-tools` 就是把它拉进来的那个包。ROS 2 本身没装之前，没有
colcon 可装。

给串口授权：

```bash
sudo usermod -aG dialout "$USER"    # 重新登录后生效
```

### 2. 建工作区并克隆各包

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

三个包的作用见上文《src/ 里放什么》。

### 3. 编译工作区

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

和容器路径一样，在 `~/litearm_ws` 里编译，绝不在 `src/` 里，也不要加 `--symlink-install`。原因见上文
《在容器里编译工作区》。

### 4. 启动控制栈

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_ros2_control litearm_control.launch.py
```

## launch 参数

两条路径跑的是同一条 launch，参数如下。它拉起 `robot_state_publisher`、`controller_manager` 和两个控制
器。manager 必须来自一个同时拥有描述文件与硬件组件的栈，这就是为什么这条 launch 自己拉起 manager，
因而需要本包随附的 URDF 入口（`urdf/litearm.urdf.xacro`）。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `''` | USB CDC 路径。留空按 VID:PID `1d50:606f` 自动发现；接多块板卡时写 `port:=/dev/ttyACM1`。 |
| `controllers_file` | `''` | 控制器参数文件。留空用包内的 `config/litearm_controllers.yaml`。 |
| `start_joint_state_broadcaster` | `true` | 拉起并激活 `joint_state_broadcaster`。 |
| `start_joint_trajectory_controller` | `true` | 拉起并激活 `joint_trajectory_controller`。 |
| `clear_faults` | `true` | 配置时清一次锁存故障，使"故障之后"仍然是一条命令。`false` 让锁存故障拦住启动。 |
| `ros_domain_id` | `42` | 本栈的 ROS 域，刻意避开 0。 |
| `ros_localhost_only` | `true` | 只在本机发现：否则同网段第二台机器人会出现在 RViz 里，并多出一个 `/move_action` 服务端。 |

### 启动 MoveIt 栈

一条命令同时拉起控制栈**和** `move_group` 与 RViz。它需要工作区里有
[litearm-moveit2](https://github.com/nexform-tech/litearm-moveit2)，并且装好 MoveIt 各包。

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
ros2 launch litearm_moveit_config litearm_moveit.launch.py use_rviz:=false
ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false
```

第二条用于无显示环境。第三条用于控制栈已经在跑、只缺 `move_group` 与 RViz 的场合——**不要在本条 launch
自己的控制栈还活着的时候用它**，两个 `controller_manager` 会抢同一批命令接口。

## 从另一个终端看到这套栈

两条 launch 都把域钉在 42 并开启仅本机发现，所以任何终端在导出同样变量前都看不到这套栈。容器路径下
先开一个 shell：

```bash
docker exec -it litearm-control bash
```

然后两条路径都一样：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # 容器里是 /ws/install/setup.bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1

ros2 control list_hardware_components    # LitearmSystem，active
ros2 control list_controllers            # 两个控制器，active
ros2 control list_hardware_interfaces    # 每个关节的 position/velocity 命令与状态接口
ros2 topic echo /joint_states --once     # 有数据 = 固件链路活着
```

没装 `ros-humble-ros2controlcli` 时，同样的信息从服务拿：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
ros2 service call /controller_manager/list_hardware_components \
  controller_manager_msgs/srv/ListHardwareComponents "{}"
```

**不要按位置读 `/joint_states`。** `name` 数组的顺序不是 `joint1`..`joint7`——它按任意顺序返回，例如
`joint2, joint3, joint5, joint6, joint1, joint4, joint7`。请按名字把数值对应到关节。

如果 `ros2` 命令仍然什么都看不到，试试加 `--no-daemon`：否则 CLI 会从一个缓存的 daemon 取答案，而那个
daemon 可能还持有你 export 之前的发现设置。

## 驱动机械臂

ros2_control 接受的是关节目标，不是笛卡尔位姿。action 是
`/joint_trajectory_controller/follow_joint_trajectory`，类型 `control_msgs/action/FollowJointTrajectory`
——和 `move_group` 用的是同一个，所以你在这里发的就是 MoveIt 本来会发的东西。

### 先选目标

选之前先读机械臂当前在哪，并对照
[litearm-moveit2](https://github.com/nexform-tech/litearm-moveit2) 里
`litearm_moveit_config/config/joint_limits.yaml` 的限位。

| 状态 | 关节值 | 什么时候用 |
| --- | --- | --- |
| `ready` | 除 `joint4` 为 `-1.5708` 外全为 0 | 默认选择：肘部抬起，离每个限位都很远 |
| `zero` | 全部为 0 | 你明确要固件零点；`joint4` 距其上限只有 17 mrad，且该位形附近是奇异点 |

**不要不加检查就发任意构型。** 超出某个关节范围的目标会被拒绝，或者，更糟的是以你没打算的路径抵达。
尤其 `joint4` 的范围很窄，`-3.0715` 到 `+0.0175` rad。

**不要在没有想好扫掠路径的情况下从一个折起的机械臂起步。** 机械臂会停在上次被留在的地方；那往往是
`joint4` 的下限，而走向 `ready` 会把肘部展开约 90 度。

### 发出去

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    points:
      - positions: [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]
        time_from_start: {sec: 5, nanosec: 0}" --feedback
```

容器路径下，把周边都补上的完整命令就是下面这条——不用先做任何准备，栈还跑着的时候直接粘进任意一个宿主机
终端即可：

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

YAML 从引号后面顶格开始。请保持顶格：它的缩进是相对 `trajectory` 算的，为了和上面那行 `ros2` 对齐而
往里推，解析就会失败。

**不要给 goal 套外层花括号。** 帮助文本里的 `'{order: 10}'` 是单行 flow 映射；把它和下面的块式键混在
一起会解析失败：

```text
yaml.parser.ParserError: while parsing a flow mapping
  in "<unicode string>", line 1, column 1:
    {trajectory:
    ^
expected ',' or '}', but got ':'
```

去掉外层的一对 `{` `}`，保留内层的 `{sec: 5, nanosec: 0}`（那一个是合法的单行 flow 映射），并且不要在
列表或映射后面加逗号。

**不要自己填 `velocities` 来给机械臂减速。** 控制器会从它自己的样条算出速度并同时下发两个接口，这正是
固件需要的。反过来填零会踩中 `config/litearm_controllers.yaml` 里记录的坑：`velocity` 是位置参考的限速，
固件每个周期最多把 `q_ref` 朝目标挪 `|dq_ref|·dt`，`dq_ref` 为零就等于冻住参考。机械臂不动，而且什么都
不报错。

`time_from_start` 是整段动作的时间预算，不是速度。`joint4` 走 1.5 rad 用 5 秒，约合 0.3 rad/s，远低于该
关节 1.75 rad/s 的上限。

### 读结果

`--feedback` 会一直流式打印实测位置，直到 goal 结束，然后打印结果：

| `error_code` | 含义 |
| --- | --- |
| `0`（SUCCESSFUL） | 到位，每个关节都在 0.02 rad 的目标容差内 |
| `1`（INVALID_GOAL） | 被拒：关节名或维度与控制器对不上 |
| `-4`（PATH_TOLERANCE_VIOLATED） | 跟不上路径；被挡住，或在顶着限位走 |
| `-5`（GOAL_TOLERANCE_VIOLATED） | 没走完：被挡住，或有某个关节顶到了限位 |

用 `ros2 topic pub` 往 `/joint_trajectory_controller/joint_trajectory` 发同样的轨迹类型也可以，但它
**没有结果码、没有超时**。**不要用它做开机验收**：出问题时你分不清是目标被拒还是消息丢了。

### 走笛卡尔位姿

本栈里没有 IK。`ros2 service list` 里只有 `controller_manager` 和两个控制器，没有别的——所以笛卡尔目标
必须先解成关节角：

- **借 MoveIt。** 起《启动 MoveIt 栈》，然后用 `moveit_msgs/srv/GetPositionIK` 调 `/compute_ik`，或者
  在 RViz 里规划、由 `move_group` 执行。求解器就是
  `litearm_moveit_config/config/kinematics.yaml` 里那一个，所以结果和 RViz 规划的一致。
- **借 SDK。** `litearm-cpp` 自带运动学，但没有暴露在 ROS 接口上，需要你自己写个小节点。

## 交还机械臂

```bash
# 在跑着栈的那个终端按 Ctrl-C，等进程真正退出。
fuser -v /dev/ttyACM0 2>/dev/null      # 必要时看谁还占着串口
```

两套部署不能重叠的原因是**串口**，不是某个 ROS 资源。

组件的 `disable_on_shutdown` 默认为 `false`，所以**栈退出后固件仍然保持最后一个姿态**——机械臂不会松
掉，也不要假设它会松。Ctrl-C 释放的是串口，那是让维护驱动能接着跑的条件；它不会给机械臂断电。

## 起不来时

| 现象 | 原因与处理 |
| --- | --- |
| `install/setup.bash: No such file or directory`，随后 `Package 'litearm_ros2_control' not found` | 容器的 `-v` 指向了一个从未编译过的工作区，或者指向了一个当时不存在、被 Docker 建成空目录的路径。确认宿主机那个目录里有 `src/`、`build/` 和 `install/`。 |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | 板子没插、没上电，或处在 DFU。先看 `lsusb`；DFU 下会枚举成 `0483:DF11`，需要刷固件才能回来。 |
| 刚才 driver 跑过，configure 就失败 | 串口仍被占用。停掉另一个进程再试，没有第二个进程能共用它。 |
| 容器里 configure 打不开串口 | 设备没透传进去，或容器不在拥有它的组里。`--device` 和 `--group-add` 两个都要有。这个失败是致命的：`ros2_control_node` 直接 abort，日志以 `Failed to set the initial state of the component` 之后的堆栈结尾。 |
| `ENABLE was refused: ... ERR{0x10,0x06}` | 固件里有锁存故障（EMERGENCY 或 `joint_fault`）。控制栈在配置时会清一次，除非 `clear_faults:=false`。 |
| `First command frame failed: ... ERR{0x03,0x02}` | 首帧发出时机械臂还没真正加磁。组件已修（会等状态帧的 enabled 位）；若再现，说明固件拒绝了一条 `dq` 全 0、目标离实测超过 5 mrad 的帧。 |
| 宿主机上报 `Package 'litearm_ros2_control' not found` | 工作区是在 `src/` 里面编译的，`install/` 落在了错误的位置。回到工作区根目录重新编译。 |
| `docker exec` 进去的 shell 看不到任何节点 | 那个 shell 里没有导出 `ROS_DOMAIN_ID=42` 和 `ROS_LOCALHOST_ONLY=1`。 |
| `/joint_states` 没有数据 | `joint_state_broadcaster` 没有激活，或者机械臂处于故障态。看控制器状态与 launch 日志。 |
| `Kinematics solver could not be instantiated` | `litearm_moveit_config/config/kinematics.yaml` 里写的求解器没装。KDL 随 MoveIt 自带；SNS-IK 需要在工作区里放那两个包。 |

## 接下来看什么

- [`README.zh-CN.md`](../README.zh-CN.md) —— 硬件组件、它的参数、随附的部署文件与控制器配置
  （英文原文：[`README.md`](../README.md)）。
- [`litearm-moveit2`](https://github.com/nexform-tech/litearm-moveit2) —— 组合 launch 背后的 MoveIt
  配置，以及它自己的快速上手。
- [`litearm-ros2`](https://github.com/nexform-tech/litearm-ros2) —— 维护驱动，用于控制回路之外的
  开机验收、授权与参数调试。

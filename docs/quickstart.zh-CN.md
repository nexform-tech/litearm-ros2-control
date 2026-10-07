# litearm 快速上手

到达机械臂的两条路，按你会用到的顺序：想要控制器、MoveIt 与 RViz 时用 ros2_control 栈；
想要 SDK 的管理类调用、链路里没有 `controller_manager` 时用 `litearm_driver`。两者**互斥**——
都要打开同一个 USB CDC 串口，SDK 会对它加独占 `flock`——所以一次只起一个。

**不要同时起。** 那样做的症状是：launch 找不到设备，或 driver 的 configure 失败——没有第二个
进程能共用这个串口。

## 0. 先做准备

```bash
source /opt/ros/humble/setup.bash
source ~/github/litearm/install/setup.bash

lsusb | grep 1d50          # 应看到 1d50:606f
ls /dev/ttyACM*            # 应看到 /dev/ttyACM0
```

## 1. ros2_control 栈

**只起控制栈**——`robot_state_publisher`、`controller_manager` 与两个控制器。manager 必须来自
一个拥有描述文件与硬件组件的栈；这条 launch 自己拉起 manager，因此需要本包随附的 URDF 入口
（`urdf/litearm.urdf.xacro`）。

```bash
ros2 launch litearm_ros2_control litearm_control.launch.py
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `''` | USB CDC 路径。留空按 VID:PID `1d50:606f` 自动发现；接多块板卡时写 `port:=/dev/ttyACM1`。 |
| `clear_faults` | `true` | 配置时清一次锁存故障，使"故障之后"仍然是一条命令。`false` 让锁存故障拦住启动。 |
| `controllers_file` | `''` | 控制器参数文件。留空用包内的 `config/litearm_controllers.yaml`。 |
| `start_joint_state_broadcaster` | `true` | 拉起并激活 `joint_state_broadcaster`。 |
| `start_joint_trajectory_controller` | `true` | 拉起并激活 `joint_trajectory_controller`。 |
| `ros_domain_id` | `42` | 本栈的 ROS 域，刻意避开 0。 |
| `ros_localhost_only` | `true` | 只在本机发现：否则同网段第二台机器人会出现在 RViz 里，并多出一个 `/move_action` 服务端。 |

**控制栈 + MoveIt + RViz**——一条命令，内部包含上面那条。

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
ros2 launch litearm_moveit_config litearm_moveit.launch.py use_rviz:=false
ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false
```

第二条用于无显示环境；第三条用于控制栈已在别处运行、只缺 `move_group` 与 RViz 的场合。

### 从另一个终端看到这套栈

两条 launch 都把域钉在 42 并开启仅本机发现，所以普通终端在导出同样变量前什么都看不到：

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
ros2 node list
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

`ros2 control list_controllers` 需要 `ros-humble-ros2controlcli`，它不是本仓的依赖；上面的服务调用
则不需要额外安装。

## 2. driver

`litearm_driver` 是独立的维护节点：它自己持有 USB 链路，把整套 SDK 指令作为 `/litearm` 下的服务暴露
出来。没有 `controller_manager`、没有控制器、没有 MoveIt。

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `namespace` | `litearm` | 服务所在命名空间，服务直接位于其中。 |
| `autostart` | `true` | 起节点后完成 configure 与 activate。`false` 则只起节点，手动驱动 lifecycle。 |
| `config` | 包内 `config/litearm_driver.yaml` | 参数文件。 |
| `log_level` | `info` | 节点日志级别。 |

### 只读启动

`auto_enable` 在参数文件里，默认 `true`。想在不给电机上电的情况下先看臂：

```bash
cp install/litearm_driver/share/litearm_driver/config/litearm_driver.yaml /tmp/driver_no_enable.yaml
sed -i 's/auto_enable: true/auto_enable: false/' /tmp/driver_no_enable.yaml
ros2 launch litearm_driver litearm_driver.launch.py config:=/tmp/driver_no_enable.yaml
```

driver 不隔离 ROS 域，普通终端可以直接访问。

### 不会让臂动的调用

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

`get_feedforward_catalog` 直接答 SDK 的静态表，完全不需要链路；`log_dump` 写进节点的 `log_dir`
（默认 `$HOME/.ros/litearm`），并且拒绝把路径当作 `filename`。

### 让机械臂动起来

每一个能给电机上电的服务，在 `allow_motion` 打开之前都会被拒。它与 `allow_dfu` 一样每次调用重新
读取，所以不需要重启：

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

`speed` 是满速的比例 `0 < speed <= 1`；`30` 表示 30 倍，不是 30%。百分比限速用
`/litearm/set_speed_scaling`，状态消息里的 `speed_scaling` 只是主机侧对上一次写入的记录。

⚠ 会让臂动之前先支撑好机械臂，并让急停在手边。`move_js`、`send_mit`、`send_mit_all` 都是单帧：
固件的 100 ms 看门狗会在没人持续重发时把臂交回持位。

## 3. 在两套之间切换

```bash
# 在跑着栈的那个终端按 Ctrl-C，等进程真正退出。
fuser -v /dev/ttyACM0 2>/dev/null      # 必要时看谁还占着串口
```

两者不能重叠的原因是**串口**，不是某个 ROS 资源：`litearm_driver` 并不持有本工作区另一套部署的
共享内存锁。

## 4. 起不来时

| 现象 | 原因与处理 |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | 板子没插、没上电，或处在 DFU。先看 `lsusb`；DFU 下会枚举成 `0483:DF11`，需要刷固件才能回来。 |
| 刚才还有别的栈跑过，configure 就失败 | 串口仍被占用。停掉另一个进程再试，没有第二个进程能共用它。 |
| `ENABLE was refused: ... ERR{0x10,0x06}` | 固件里有锁存故障（EMERGENCY 或 `joint_fault`）。控制栈在配置时会清一次，除非 `clear_faults:=false`；若是授权原因，driver 的状态消息里能看到授权字段。 |
| `First command frame failed: ... ERR{0x03,0x02}` | 首帧发出时机械臂还没真正加磁。组件已修（会等状态帧的 enabled 位）；若再现，说明固件拒绝了一条 `dq` 全 0、目标离实测超过 5 mrad 的帧。 |
| `Kinematics solver could not be instantiated` | `litearm_moveit_config/config/kinematics.yaml` 里写的求解器没装。KDL 随 MoveIt 自带；SNS-IK 需要在工作区里放那两个包。 |

## 5. 接下来看什么

- [`docs/command-set.zh-CN.md`](command-set.zh-CN.md) —— 每个 driver 服务、它的拒绝规则与背后的
  链路语义（英文原文：[`docs/command-set.md`](command-set.md)）。
- [`litearm_driver/README.zh-CN.md`](../litearm_driver/README.zh-CN.md) —— driver 的参数、话题与
  字段级服务参考。
- [`litearm_ros2_control/README.zh-CN.md`](../litearm_ros2_control/README.zh-CN.md) —— 硬件组件、
  它随附的部署文件与控制器配置。
- [`litearm_moveit_config`](https://github.com/thetooler/litearm-moveit2) —— 本快速上手第二个入口
  所属的 MoveIt 配置。

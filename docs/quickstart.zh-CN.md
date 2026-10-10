# litearm ros2_control 快速上手

把 ros2_control 栈在真机上跑起来的命令，面向希望用 `joint_trajectory_controller` 或 MoveIt 2
驱动机械臂的 ROS 2 用户。每一节都是按顺序执行的一步；第 4 节是某一步失败时该看的地方。

维护驱动不在本仓库里。如果你要的是链路里没有 `controller_manager` 的 SDK 管理类调用，请用
[litearm-ros2](https://github.com/nexform-tech/litearm-ros2)。两个入口**互斥**——都要打开同一个
USB CDC 串口，SDK 会对它加独占 `flock`——所以一次只跑一个。

**不要在本栈运行时跑 driver。** 症状是 launch 找不到设备，或 driver 的 configure 失败：没有第二个
进程能共用这个串口。

## 0. 先做准备

```bash
source /opt/ros/humble/setup.bash
source ~/litearm_ws/install/setup.bash

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

## 2. 交还机械臂

```bash
# 在跑着栈的那个终端按 Ctrl-C，等进程真正退出。
fuser -v /dev/ttyACM0 2>/dev/null      # 必要时看谁还占着串口
```

两套部署不能重叠的原因是**串口**，不是某个 ROS 资源。

## 3. 怎么驱动它

栈起来之后，机械臂就是一个普通的 ros2_control 系统：给 `joint_trajectory_controller` 发轨迹，
或者在 MoveIt 2 里规划、由 `move_group` 执行。launch 起来的两个控制器是 `joint_state_broadcaster`
与 `arm_controller`（一个 `joint_trajectory_controller`）；关节名是 `joint1`..`joint7`。

## 4. 起不来时

| 现象 | 原因与处理 |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | 板子没插、没上电，或处在 DFU。先看 `lsusb`；DFU 下会枚举成 `0483:DF11`，需要刷固件才能回来。 |
| 刚才 driver 跑过，configure 就失败 | 串口仍被占用。停掉另一个进程再试，没有第二个进程能共用它。 |
| `ENABLE was refused: ... ERR{0x10,0x06}` | 固件里有锁存故障（EMERGENCY 或 `joint_fault`）。控制栈在配置时会清一次，除非 `clear_faults:=false`。 |
| `First command frame failed: ... ERR{0x03,0x02}` | 首帧发出时机械臂还没真正加磁。组件已修（会等状态帧的 enabled 位）；若再现，说明固件拒绝了一条 `dq` 全 0、目标离实测超过 5 mrad 的帧。 |
| `Kinematics solver could not be instantiated` | `litearm_moveit_config/config/kinematics.yaml` 里写的求解器没装。KDL 随 MoveIt 自带；SNS-IK 需要在工作区里放那两个包。 |
| `/joint_states` 没有数据 | `joint_state_broadcaster` 没有激活，或者机械臂处于故障态。看控制器状态与 launch 日志。 |

## 5. 接下来看什么

- [`README.zh-CN.md`](../README.zh-CN.md) —— 硬件组件、它的参数、随附的部署文件与控制器配置
  （英文原文：[`README.md`](../README.md)）。
- [`litearm_moveit_config`](https://github.com/thetooler/litearm-moveit2) —— 本快速上手第二个入口
  所属的 MoveIt 配置。
- [`litearm-ros2`](https://github.com/nexform-tech/litearm-ros2) —— 维护驱动，用于控制回路之外的
  开机验收、授权与参数调试。

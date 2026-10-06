# litearm-ros2-control

把 LiteArm 机械臂接入 ros2_control 的 `SystemInterface` 插件，面向希望在 ROS 2 上用
`joint_trajectory_controller` 或 MoveIt 2 通过一根 USB 线驱动真机的用户。

插件自己持有机械臂的 USB CDC 链路，走 [litearm-cpp](https://github.com/nexform-tech/litearm-cpp)
SDK。没有辅助进程，也没有共享内存：`read()` 取 SDK 缓存好的 100 Hz 状态帧，`write()` 用固件的
`MOVE_JS` 伺服把指令参考发回去。轨迹规划、正逆运动学与动力学都留在固件里 —— 上位机只发关节参考、
读状态。

## 包组成

| 包 | 是什么 |
| --- | --- |
| `litearm_ros2_control` | ros2_control 的 `SystemInterface` 插件：控制通路，由 `joint_trajectory_controller` 或 MoveIt 2 驱动。 |
| `litearm_driver` | 独立生命周期节点，自己持有同一条 USB 链路，把 SDK 的管理类指令以服务形式暴露出来。只能**替代**控制栈运行，不能与之并存。 |
| `litearm_msgs` | `litearm_driver` 使用的消息与服务定义。 |

控制栈与驱动节点互斥：两者都会打开同一个串口，SDK 会对其加独占锁。驱动的完整指令集见
[docs/command-set.md](docs/command-set.md)。

## 特点

- **一根线，无需守护进程。** controller manager 直接与固件通信。
- **位置环跑在固件里**，`tau = kp*(q_ref - q) + kd*(dq_ref - dq) + feedforward(q, dq)`，
  `kp`/`kd` 取自固件自己的参数表。插件不发增益，也不算动力学。
- **标准接口**，`joint_state_broadcaster` 与 `joint_trajectory_controller` 无需改动即可使用。
- **离线测试**。整个生命周期都跑在 SDK 的 `FakeTransport` 上，测试套件不需要机械臂。

## 接口映射

每个关节导出三个状态接口和两个命令接口：

| 方向 | 接口 | 固件含义 |
| --- | --- | --- |
| command | `position` | `q_ref`（`MOVE_JS`） |
| command | `velocity` | `dq_ref`（`MOVE_JS`） |
| state | `position` | 实测 `q` |
| state | `velocity` | 实测 `dq` |
| state | `effort` | 实测 `tau` —— 电流估算值，噪声大，不可用于控制 |

当 `export_diagnostic_interfaces` 为 `true`（默认）时，每个关节还会导出四个诊断状态接口：

| 接口 | 含义 |
| --- | --- |
| `temperature_mos` | MOSFET 温度 |
| `temperature_coil` | 线圈温度 |
| `error_code` | 固件故障字 |
| `feedback_age` | 最近一帧状态帧的年龄，单位秒 |

## 环境要求

| 项目 | 要求 |
| --- | --- |
| ROS 2 | Humble |
| 构建 | `ament_cmake`，C++17 |
| SDK | `litearm-cpp`，已安装或作为同级源码树 |
| 固件 | `Litearm1.5.0` 或更新 —— 版本串形如 `Litearm<主.次.修>-{7J\|1J}` |
| 连接 | USB CDC 串口，`VID:PID 1d50:606f` |

低于 1.5.0 的固件在连接阶段会被拒绝：插件依赖 1.5.0 才引入的 6+21N 状态帧布局、`joint_fault`
字段和使能位。

Linux 下需要授予串口权限：

```bash
sudo usermod -aG dialout $USER      # 重新登录后生效
```

## 构建

除非 `CMAKE_PREFIX_PATH` 上已经有装好的 `litearm` CMake 包，否则本包需要 `litearm-cpp` 源码树
作为同级目录：

```text
~/litearm_ws/src/
├── litearm-cpp/              # C++ SDK
└── litearm-ros2-control/     # 本仓库
```

```bash
source /opt/ros/humble/setup.bash
cd ~/litearm_ws
colcon build
source install/setup.bash
```

如果 `litearm-cpp` 签出在别处，用 CMake 参数指过去：

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

不要把 `litearm-cpp` 按默认的静态库构建后再指望本插件能链上。ros2_control 插件是共享对象，它内嵌
的 SDK 必须位置无关。本包的处理办法是把同级源码树编成静态 **PIC** 库再链进来，因此装出来的插件是
自包含的。若已安装的 `litearm` 是非 PIC 静态库，构建会以一条解释性错误失败，而不是留到链接期 —— 用
`-DBUILD_SHARED_LIBS=ON` 重装 SDK，或者删掉它、让本包改用同级源码树。

## 使用

在 URDF 的 `<ros2_control>` 块里声明该组件。关节名必须是 `joint1..jointN`，每个轴一个，且 N 必须
等于固件上报的轴数 —— 机械臂是 7，台架是 1。名字决定轴映射，所以 `<joint>` 元素的顺序无关紧要。

```xml
<ros2_control name="LitearmSystem" type="system">
  <hardware>
    <plugin>litearm_ros2_control/LitearmSystem</plugin>
    <!-- 留空或省略：按 VID:PID 1d50:606f 自动发现 -->
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
  <!-- joint2 .. joint7：结构完全相同 -->
</ros2_control>
```

所有参数都是可选的：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | 空 | USB CDC 设备路径。留空则按 `VID:PID 1d50:606f` 自动发现。 |
| `export_diagnostic_interfaces` | `true` | 是否导出那四个诊断状态接口。 |
| `auto_enable` | `true` | 在 `on_activate` 里使能电机。设为 `false` 可自行控制使能时机。 |
| `disable_on_shutdown` | `false` | 关闭时保持电机使能，由固件维持姿态。设为 `true` 会失去力，机械臂会掉下来。 |
| `enable_attempts` | `12` | `enable()` 的重试次数。SDK 每次重试之间睡 300 ms，且只有可重试的错误码才消耗次数。 |

一套最小的控制器配置，用的是本包依赖的标准控制器：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz；不低于 100 即可满足固件看门狗

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

## 流式发送约定

固件的指令看门狗在 100 ms 收不到指令后跳闸，所以 `write()` 必须持续重发。controller manager 的
更新率只要不低于 100 Hz 就自动满足这一条。

**控制循环活跃时，不要让更新循环卡顿超过约 100 ms。** 超时后固件退回到失效软保持，机械臂会朝负载
拉拽的方向垂下去。

还要清楚在进程内直接驱动机械臂的取舍：`write()` 会进到 SDK，写一帧并等固件 ACK，上限是 SDK 自己的
1.2 s 超时。链路健康时这是亚毫秒级，但 USB 链路卡住时可能把控制循环堵到那个上限。若需要与 USB 隔离
的硬实时循环，请改用共享内存 + 守护进程的方案驱动机械臂。

## 维护驱动

`litearm_driver` 是接触机械臂的第二条通路。它自己打开 USB 链路，把 SDK 的管理类调用暴露为服务：
使能、park、清故障、进入零重力、调全局速度倍率与前馈、读写关节参数表、读取与激活授权、进入 DFU。

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
```

**不要让驱动节点与控制栈同时运行。** 驱动面向维护与开机验收，不做运动控制：轨迹、伺服、遥操作
仍然留在控制通路里。每个服务、每条拒绝规则及其原因见 [docs/command-set.md](docs/command-set.md)。

## 测试

测试套件完全离线 —— 它注入 SDK 的 `FakeTransport`，不接机械臂就能跑完整个生命周期：

```bash
cd ~/litearm_ws
colcon test --packages-select litearm_ros2_control --event-handlers console_direct+
colcon test-result --verbose
```

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。

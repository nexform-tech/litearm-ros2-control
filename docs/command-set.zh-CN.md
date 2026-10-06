# litearm 指令集

本文是 [`docs/command-set.md`](command-set.md) 的中文对照，逐条说明 `litearm_driver` 暴露的每个
ROS 2 接口对机械臂做了什么；在不跑 ros2_control 控制栈时操作机械臂，或为驱动写客户端时看它。
两份文档不一致时，以英文版为准。字段级接口参考见 [litearm_driver/README.zh-CN.md](../litearm_driver/README.zh-CN.md)。

验证状态：服务名、服务类型、请求处理、拒绝路径与状态消息都由离线测试套件覆盖
（`colcon test --packages-select litearm_driver`）。下文的 `ros2` 命令行是**经测试的接口契约**；
对真机执行这些命令不在本仓测试运行范围内。

## 1. 一台臂只允许一个进程

驱动会打开机械臂的 USB CDC 串口，SDK 会对其加独占 `flock`。因此两个进程不可能同时驱动一台臂：

- **控制栈运行期间不要启动 `litearm_driver`。** 第二个进程打不开串口，驱动会在 configure 阶段报错。
- 反向同理：先停掉驱动，再启动 `ros2_control_node`。

驱动是**维护通路**：授权、故障处理、参数整定、开机验收，链路里没有 `controller_manager`。
运动控制属于 `joint_trajectory_controller`（经 MoveIt 2），或属于自定义控制器，例如
`litearm_servo_control` 里的伺服控制器。

## 2. 启动与停止

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

launch 文件把节点起在 `litearm` 命名空间下、名为 `/litearm/driver`，因此下文所有服务都是
`/litearm/<name>`。除非传入 `autostart:=false`，它会自动 configure 并 activate。

```bash
ros2 launch litearm_driver litearm_driver.launch.py autostart:=false
ros2 lifecycle set /litearm/driver configure
ros2 lifecycle set /litearm/driver activate
ros2 lifecycle get /litearm/driver
```

deactivate（`ros2 lifecycle set /litearm/driver deactivate`）会撤销全部服务、退出零重力（若在
激活中），并**保持电机状态不变**：`park()` 与固件持位会让臂停在原地。deactivate 时失能电机会让
机械臂掉下来。

只有在你自己指定命名空间时才直接运行可执行文件；没有命名空间时，相对服务名会解析成 `/enable`、
`/status` 等顶层名字。

### 2.1 参数

每个参数都有默认值。节点在 configure 时读取它们，例外是 `allow_dfu`、
`allow_license_activation` 和 `zero_g_keepalive_period_s`：这三个在**每次调用时重新读取**，
所以 `ros2 param set` 立即生效。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `""` | USB CDC 设备路径。留空表示按 VID:PID `1d50:606f` 自动发现。 |
| `joint_names` | 未设置 | 按固件轴序给出的轴名。未设置时由固件上报的轴数推导 `joint1`..`jointN`；长度不对会让 configure 失败。 |
| `auto_enable` | `true` | activate 时使能电机。ENABLE 被拒会让 activate 失败，而不是留下一个"激活但电机没力"的节点。 |
| `enable_attempts` | `12` | `enable()` 的重试次数；只有固件标注"可重试"的错误码才消耗重试。 |
| `status_rate_hz` | `10.0` | `/litearm/status` 发布频率。 |
| `publish_joint_states` | `false` | 是否发布 `joint_states_topic`。只要有 `joint_state_broadcaster` 在跑就保持 false。 |
| `joint_states_topic` | `/joint_states` | 可选关节状态话题名。 |
| `publish_diagnostics` | `true` | 是否发布 `/diagnostics`。 |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` 发布频率。 |
| `zero_g_keepalive_period_s` | `0.04` | 零重力保活周期，取值 `[0.005, 0.10)`。 |
| `allow_dfu` | `false` | `enter_dfu` 的开关。 |
| `allow_license_activation` | `false` | `activate_license` 的开关。 |
| `frame_id` | `""` | 状态与关节状态消息的 header frame id。 |

## 3. 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | 只含缓存与本地 SDK 状态；发布器从不发帧，因此订阅不会干扰在途命令。`speed_scaling` 与 `zero_g_*` 是主机侧视图，不是固件回读值。 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 一条状态：链路、使能、故障、掉线持位推断、授权、反馈新鲜度。由本节点直接发布，没有走 `diagnostic_updater`，以把依赖集控制在与 ros2_control 栈相同的水位。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仅在 `publish_joint_states:=true` 时发布。 |

`LitearmStatus` 里的逐轴温度、错误码与 `joint_fault` 位图都按 `joint_names` 顺序排列。
当前帧没有覆盖到的轴，温度读 `NaN`、错误码读 `0`；首帧到达前 `feedback_age` 为 `-1`。

## 4. 服务

所有服务名都相对于节点命名空间，因此用 launch 文件启动时就是 `/litearm/<name>`。
`success=false` 一定同时给出说明原因的 `message`；若是被门禁拒绝，还会写明拒绝的理由。

### 4.1 状态与安全

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | `Arm::enable(enable_attempts)`。 |
| `disable` | `std_srvs/srv/Trigger` | `Arm::disable()`。臂不再被持位，先做好支撑。 |
| `reset` | `std_srvs/srv/Trigger` | `Arm::reset()`：清故障并重新锚定位姿。不是 MCU 重启。 |
| `clear_faults` | `std_srvs/srv/Trigger` | `Arm::clear_faults()`：只清 RAM 里的故障位。 |
| `emergency_stop` | `std_srvs/srv/Trigger` | `Arm::emergency_stop()`。这是软件请求；权威仍是硬件急停。 |
| `park` | `std_srvs/srv/Trigger` | `Arm::park()`：声明全刚度静态持位。 |
| `zero_g` | `std_srvs/srv/SetBool` | `data=true` 进入零重力并启动保活；`data=false` 退出。进入后可以手推机械臂，务必在净空环境下操作。 |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
```

### 4.2 状态与授权

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `get_status` | `litearm_msgs/srv/GetStatus` | 发一次 `GET_STATUS` 并返回一条 `LitearmStatus`。`timeout <= 0` 表示立刻返回缓存帧。被动状态流静默、又需要一个带超时上限的"还活着"证据时用它。 |
| `get_license` | `litearm_msgs/srv/GetLicense` | 读取设备授权记录。未激活的臂也正常应答："未激活"是一种状态，不是错误。 |
| `activate_license` | `litearm_msgs/srv/ActivateLicense` | 提交厂商签发的授权凭据。需要 `allow_license_activation:=true` 且电机已失能。先读 `get_license`：签发工具必须使用该记录里的设备 UID。 |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
ros2 param set /litearm/driver allow_license_activation true
ros2 service call /litearm/activate_license litearm_msgs/srv/ActivateLicense \
  "{cust_id: 7, issued: 20261006, flags: 0, mac: [171, 171, 171, 171, 171, 171, 171, 171, \
  171, 171, 171, 171, 171, 171, 171, 171]}"
```

`mac` 由厂商签发工具产生。本仓不计算、也不保存任何密钥材料。

### 4.3 运动配置

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | 写固件的全局速度倍率，整数百分比 0..100。它是全局且持续的，与 SDK `movej` 的单条轨迹 `speed` 参数不是一回事。 |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | 写 `SET_MOTION_MODE`。当前固件只识别 mode 0，也就是 `park()`；其他值 SDK 会直接报错，而不是 ACK 一个并未发生的模式切换。 |
| `set_payload` | `litearm_msgs/srv/SetPayload` | 声明末端载荷质量（0..20 kg）与质心（米，法兰坐标系）。 |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### 4.4 动力学与前馈

固件会静默钳制前馈数值，因此一次写入成功只代表"钳后的值写进去了"，不代表你要的值写进去了。
写完请回读确认。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | 写前馈使能掩码。`FF_ALL` 之外的位会被拒绝，而不是被折成"全关"——那等于关掉重力补偿。 |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | 0 全关，1 出厂，2 全开。 |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | 写一个标量（`item`、`sub`、`value`）。`sub` 只对 item 5、6 有意义。item 编号见 `litearm::Arm::ff_scalar_items()`。 |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | 写一个向量（`item`、`values`）：item 1..5、7、8 为 7 个值；重力向量（item 6）为 3 个值。长度不对由 SDK 拒绝；非有限值在这里就被拒。 |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | 回读一个标量。item 9 是只读扩展，返回 `ff_mask`。 |

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload "{mass: 0.8, com: [0, 0, 0.05]}"
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 4, sub: 0}"
```

前馈掩码与 item 编号来自固件自己的参数表；`ff_scalar_items()` 与 `ff_vec_items()` 是权威列表。

### 4.5 关节参数

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | 读一个轴（`joint >= 0`）或全部轴（`joint: -1`）。 |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | 写一个轴的 `kp`、`kd`、`tau_max`。这三个是固件自身位置环的增益。 |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | 写一个轴的软限位。主机在连接时缓存限位，因此 SDK 的本地预检要等下次 connect 才会用上新值。 |
| `reset_factory_params` | `std_srvs/srv/Trigger` | `JointParams::reset_factory()`。要求电机已失能。 |
| `save_params` | `std_srvs/srv/Trigger` | 把运行时参数写入 Flash。要求电机已失能。Flash 写入不可逆。 |

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### 4.6 维护

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `enter_dfu` | `std_srvs/srv/Trigger` | 让设备登记进入 ROM bootloader。需要 `allow_dfu:=true`、电机已失能，且 ROM 向量表有效。ACK 之后 CDC 设备会消失，本节点在重新用新链路 configure 之前不可用。 |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

**不要**为了"试一下"调用 `enter_dfu`：设备会重新枚举成 `0483:DF11`，必须烧入固件才能回来。

## 5. 拒绝规则及其含义

每条拒绝都会点名被拒的参数、数值或操作。门禁在构造任何帧**之前**执行，因此被拒的调用不会在链路上
留下任何字节（由测试 `RefusedArgumentsSendNoFrame` 覆盖）。

| 拒绝信息 | 触发条件 | 为什么在这里拒绝而不是转发 |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` 超出范围 | 固件收的是整数百分比：`1` 是百分之一速，不是全速。 |
| `preset must be 0 ... 2` | `set_feedforward_preset` 超出范围 | 只有三档预设。 |
| `period must be in [0.005, 0.10)` | `zero_g` 遇到越界的 `zero_g_keepalive_period_s` | 固件命令看门狗在 0.10 s 触发，更慢的保活会静默掉出零重力。 |
| `mass must be in [0, 20] kg` | `set_payload` 超出范围 | 固件会静默钳制；存进去的值将与请求值不同。 |
| `requires the motors to be disabled` | 使能状态下调用 `save_params`、`reset_factory_params`、`activate_license` | 写 Flash 与激活只有在没有力矩权限时才是安全的。 |
| `set the parameter ... to true` | 开关参数为 false 时调用 `enter_dfu`、`activate_license` | 两者都不可逆或涉及信任，默认关闭。 |
| `not connected` | 任何在链路不可用时调用的服务 | 点出常见原因：USB 线、24 V 供电，或控制栈占着串口。 |
| `axis index must be 0..N-1` | 关节服务传入了固件未上报的轴号 | 否则一个笔误就会写到另一个轴上。 |

这些预检是尽力而为：`save_params` 靠缓存的状态帧判断电机是否使能。SDK 与固件会在链路上执行同一条
规则，所以缓存过期最多把一次干净的拒绝变成固件自己的错误消息，绝不会变成一次不安全的写入。

## 6. 刻意不暴露的能力

| 能力 | 现在放在哪里 | 为什么 |
| --- | --- | --- |
| 轨迹执行（`movej`、`movej_sync`、`move_p`、`home`） | `litearm_ros2_control` + `joint_trajectory_controller` | 运动控制属于 ros2_control 栈，它持有实时循环与关节接口。 |
| 笛卡尔伺服与遥操作 | MoveIt Servo + 自定义控制器（工作区里的 `litearm_servo_control`） | 伺服需要命令通路上有控制器，而不是一个每次只交接一个位姿的服务。 |
| 固件笛卡尔规划（`move_l`、`move_c`、`move_path`） | 未接入 ROS | 它是一次性规划、且必须串行使用；未来用 action 接口才是正确的形态，并且不能与主机侧伺服在同一台臂上混用。 |
| 模型导入、日志采集、`kin_bench` | SDK CLI 与诊断脚本 | 长时间或大批量传输，不该放进一次服务调用。 |
| 控制栈运行时的关节状态发布 | `joint_state_broadcaster` | 同一个 `joint_states` 话题上两个发布者会让机械臂状态产生歧义；驱动自己的发布器默认关闭。 |

## 7. 错误、超时与链路

- 所有 SDK 调用由同一个互斥回调组串行化。命令要等固件 ACK，因此 USB 链路卡住时一次服务调用可能被
  拖到 SDK 自己的超时上限（1.2 s）；这段时间状态发布也会停，这是"两条命令绝不在链路上交错"的代价。
- 服务失败时返回 `success=false` 以及 SDK 自己的消息，例如寄存器被钳制、锁存故障要求"先 RESET"、
  未激活授权时 `ENABLE` 被拒。
- 从主机角度看这台臂不是实时系统：本节点用于维护与诊断，不用于控制回路。

## 8. 兼容性

| 项目 | 要求 |
| --- | --- |
| ROS 2 | Humble |
| 固件 | `Litearm1.5.0` 及以上（状态帧布局与 `joint_fault`）；`Litearm1.8.0` 及以上（授权记录与激活） |
| SDK | `litearm-cpp`，可以是已安装的包，也可以是同级源码树 |
| 硬件 | USB CDC `VID:PID 1d50:606f`，运动需要 24 V 供电 |

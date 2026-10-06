# litearm_driver

独立运行的 ROS 2 生命周期节点：自己持有 LiteArm 机械臂的 USB 链路，把 SDK 的管理类指令以服务形式
暴露出来，用于在没有 controller_manager 参与的情况下维护、授权或整定机械臂。

[English](README.md) · 简体中文

节点经 [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK 与固件通信。服务在 activate
时创建、deactivate 时销毁，所有 SDK 调用由同一个回调组串行化。

**本节点与 ros2_control 控制栈互斥。** 两者都要打开同一个串口，而 SDK 会对其加独占 `flock`，
所以第二个进程打不开串口。启用其中一个前请先停掉另一个。

## 运行

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

launch 文件把节点放在 `litearm` 命名空间下，因此相对服务名会变成 `/litearm/enable`、
`/litearm/clear_faults` 等。下文所有服务名都相对于该命名空间书写。

## 指令集

节点提供 6 组共 24 个服务、3 个话题与 13 个参数。每个服务都返回 `success` 与 `message`；失败时
message 是 SDK 自己的原文，若是被门禁在发帧前拦下，则会说明拒绝的理由。

三条规则决定了整个接口的形状：

- **门禁先于发帧。** 被拒的参数不会在链路上留下任何字节。节点拒绝的是那些"固件会静默改成另一个值"
  的请求：超出 `[0, 20]` kg 的载荷质量、超出 `0..100` 的速度百分比、超过 100 ms 命令看门狗的零重力
  保活周期、非有限的前馈数值。
- **同一时刻只有一条命令。** 所有 SDK 调用都在同一个互斥回调组里执行，因此两条命令绝不可能在链路上
  交错。命令需要等固件 ACK，所以链路卡住时一次服务调用最多会被拖到 SDK 自己的 1.2 s 超时。
- **读取走缓存。** 只有 `get_status`、`get_license`、`get_feedforward_scalar`、`get_joint_params`
  会按需与固件通信。`/litearm/status` 从不发帧，因此订阅它不会干扰在途命令。

### 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `""` | USB CDC 设备路径。留空表示按 VID:PID `1d50:606f` 自动发现。 |
| `joint_names` | 未设置 | 按固件轴序给出的轴名。未设置时由固件上报的轴数推导 `joint1`..`jointN`；长度不对会让 configure 失败。 |
| `auto_enable` | `true` | activate 时使能电机。ENABLE 被拒会让 activate 失败，而不是留下一个"激活但电机没力"的节点。 |
| `enable_attempts` | `12` | `enable` 服务的重试次数；只有固件标注"可重试"的错误码才消耗重试。 |
| `status_rate_hz` | `10.0` | `/litearm/status` 发布频率。 |
| `publish_joint_states` | `false` | 是否发布 `joint_states_topic`。只要有 `joint_state_broadcaster` 在跑就保持 false。 |
| `joint_states_topic` | `/joint_states` | 可选关节状态话题名。 |
| `publish_diagnostics` | `true` | 是否发布 `/diagnostics`。 |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` 发布频率。 |
| `zero_g_keepalive_period_s` | `0.04` | 零重力保活周期，取值 `[0.005, 0.10)`。每次 `zero_g` 调用都会重新读取。 |
| `allow_dfu` | `false` | `enter_dfu` 的开关，每次调用重新读取。 |
| `allow_license_activation` | `false` | `activate_license` 的开关，每次调用重新读取。 |
| `frame_id` | `""` | 状态与关节状态消息的 header frame id。 |

### 话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | 以 `status_rate_hz` 发布缓存与本地 SDK 状态，从不发帧。 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 一条状态：链路、使能、故障、掉线持位推断、授权、反馈新鲜度。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仅在 `publish_joint_states:=true` 时发布。默认关闭，避免与 `joint_state_broadcaster` 抢同一个话题。 |

`LitearmStatus` 按 `joint_names` 顺序携带 `temperature_mos`、`temperature_coil`、
`joint_error_code` 与 `joint_fault` 位图。当前帧没有覆盖到的轴，温度读 `NaN`、错误码读 `0`；
首帧到达前 `feedback_age` 为 `-1`。`speed_scaling` 与 `zero_g_*` 是主机侧视图，不是固件回读值。

### 服务索引

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | 使能全部轴。 |
| `disable` | `std_srvs/srv/Trigger` | 失能全部轴；机械臂不再被持位。 |
| `reset` | `std_srvs/srv/Trigger` | 清故障并重新锚定位姿。 |
| `clear_faults` | `std_srvs/srv/Trigger` | 只清 RAM 里的故障位。 |
| `emergency_stop` | `std_srvs/srv/Trigger` | 请求急停。 |
| `park` | `std_srvs/srv/Trigger` | 声明全刚度静态持位。 |
| `zero_g` | `std_srvs/srv/SetBool` | 进入或退出零重力（手动引导）。 |
| `get_status` | `litearm_msgs/srv/GetStatus` | 主动发 `GET_STATUS` 并返回一次快照。 |
| `get_license` | `litearm_msgs/srv/GetLicense` | 读取设备授权记录。 |
| `activate_license` | `litearm_msgs/srv/ActivateLicense` | 提交厂商签发的授权凭据。 |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | 写全局速度倍率（百分比）。 |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | 写固件运动模式。 |
| `set_payload` | `litearm_msgs/srv/SetPayload` | 声明末端载荷质量与质心。 |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | 写前馈使能掩码。 |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | 选择前馈预设。 |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | 写一个前馈标量。 |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | 写一个前馈向量。 |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | 回读一个标量。 |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | 读关节参数表。 |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | 写某个轴的 `kp`/`kd`/`tau_max`。 |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | 写某个轴的软限位。 |
| `reset_factory_params` | `std_srvs/srv/Trigger` | 恢复出厂关节参数。 |
| `save_params` | `std_srvs/srv/Trigger` | 把运行时参数写入 Flash。 |
| `enter_dfu` | `std_srvs/srv/Trigger` | 让设备登记进入 ROM bootloader。 |

### 状态与安全

#### `enable`

`Arm::enable(enable_attempts)`。只有固件标注可重试的错误码会被重试；锁存故障或缺授权会立即失败。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。授权必须已激活，否则固件以 `ERR{0x10,0x08}` 拒绝 ENABLE。 |

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger "{}"
```

#### `disable`

`Arm::disable()`。它绕过零重力守卫，因为"降低能量"是安全方向。

**不要**在机械臂没有支撑时调用 `disable`：位置环不再持位，负载会掉下来。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/disable std_srvs/srv/Trigger "{}"
```

#### `reset`

`Arm::reset()`：清故障并重新锚定位姿。不是 MCU 重启。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/reset std_srvs/srv/Trigger "{}"
```

#### `clear_faults`

`Arm::clear_faults()`：只清 RAM 里的故障位。需要同时重新锚定位姿参考时用 `reset`。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
```

#### `emergency_stop`

`Arm::emergency_stop()`。这是软件请求：权威仍是硬件急停；链路断开时固件自己的看门狗会持住机械臂。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/emergency_stop std_srvs/srv/Trigger "{}"
```

#### `park`

`Arm::park()`：声明全刚度静态持位，不依赖 100 ms 命令看门狗。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/park std_srvs/srv/Trigger "{}"
```

#### `zero_g`

`data=true` 调用 `Arm::zero_g_start(zero_g_keepalive_period_s)`；`data=false` 调用
`Arm::zero_g_stop(true)`，因此保活线程若曾中断会被如实报出而不是吞掉。保活线程由 SDK 持有。

**不要在零重力激活期间发送其它运动命令**：固件会拒绝，而且那一刻机械臂处于人力引导状态。
请先退出零重力。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `data`（请求） | `bool` | `true` 进入零重力，`false` 退出。 |
| `success`（响应） | `bool` | 被门禁或 SDK 拒绝时为 `false`。 |
| `message`（响应） | `string` | 退出时若保活曾失败，会带上原因。 |

```bash
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: false}"
```

### 状态与授权

#### `get_status`

主动发 `GET_STATUS`（`Arm::get_status_now(timeout)`）并返回一条 `LitearmStatus`。被动状态流静默、
又需要一个带超时上限的"还活着"证据时用它。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `timeout`（请求） | `float64` | 等待应答的秒数；`<= 0` 表示立刻返回缓存帧。 |
| `status`（响应） | `litearm_msgs/LitearmStatus` | 快照，字段与话题一致。 |
| `success`（响应） | `bool` | 未连接或固件未应答时为 `false`。 |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

#### `get_license`

`Arm::license()`：设备授权记录。未激活的臂也正常应答，因为"未激活"是一种状态而非错误。该调用同时
刷新节点缓存的记录，也就是 `/litearm/status` 报告的那份。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `state`（响应） | `uint8` | 0 未激活 / 1 已激活 / 2 已激活且产线模式。 |
| `state_name`（响应） | `string` | SDK 给该状态的可读名。 |
| `cust_id`（响应） | `uint32` | 客户号；未激活时为 0。 |
| `issued`（响应） | `uint32` | 签发日 `YYYYMMDD`；未激活时为 0。 |
| `flags`（响应） | `uint32` | bit0 为产线码。 |
| `uid_hex`（响应） | `string` | 24 位小写十六进制，即签发工具必须使用的 UID。 |

```bash
ros2 service call /litearm/get_license litearm_msgs/srv/GetLicense "{}"
```

#### `activate_license`

`Arm::activate(cust_id, issued, flags, mac, 16)`，随后回读一次：固件把"已经激活过"与"MAC 不符"
聚合进同一个错误码，只有状态才是可靠答案。

**不要**在电机使能时激活：本服务会拒绝，SDK 与固件也会。先读 `get_license`，用该记录里的 UID 去签发。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `cust_id`（请求） | `uint32` | 客户号。 |
| `issued`（请求） | `uint32` | 签发日 `YYYYMMDD`。 |
| `flags`（请求） | `uint32` | bit0 为产线码。 |
| `mac`（请求） | `uint8[16]` | 厂商签发工具产出的两个 SipHash-2-4 标签。 |
| `state`、`state_name`（响应） | `uint8`、`string` | 尝试后回读的授权状态。 |
| 前置条件 | | `allow_license_activation:=true` 且电机已失能。 |

```bash
ros2 param set /litearm/driver allow_license_activation true
ros2 service call /litearm/activate_license litearm_msgs/srv/ActivateLicense \
  "{cust_id: 7, issued: 20261006, flags: 0, mac: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

### 运动配置

#### `set_speed_scaling`

`Arm::set_speed(percent)`：固件的全局速度倍率。它是全局且持续的，与 SDK `movej` 的单条轨迹
`speed` 参数不是一回事。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `percent`（请求） | `uint8` | 0..100。这是整数百分比，`1` 表示百分之一速，不是全速。 |
| 门禁 | | 越界值在构造任何帧之前就被拒绝。 |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
```

#### `set_motion_mode`

`Arm::set_motion_mode(mode)`。当前固件只识别 mode 0，也就是 `park()`；其他值 SDK 会直接报错，
而不是 ACK 一个并未发生的模式切换。响应 message 里也写明了这一点。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mode`（请求） | `int32` | 模式号。目前只接受 0。 |

```bash
ros2 service call /litearm/set_motion_mode litearm_msgs/srv/SetMotionMode "{mode: 0}"
```

#### `set_payload`

`Arm::set_payload(mass, com)`：声明固件需要补偿的载荷。固件会把质量静默钳进 `[0, 20]` kg，
因此节点在越界时直接拒绝，而不是为一个与请求不同的数字报告成功。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mass`（请求） | `float64` | 千克，取值 `[0, 20]`。 |
| `com`（请求） | `float64[3]` | 质心，米，法兰坐标系，三个分量都必须有限。 |
| 门禁 | | 范围与有限性在这里检查；SDK 与固件也会在链路上执行同一条规则。 |

用 `get_feedforward_scalar`（`item: 4, sub: 0`）回读，拿到的是钳后的真值。

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### 动力学与前馈

固件会静默钳制前馈数值，因此"写入成功"只代表钳后的值被存下，不代表你要的值被存下。
**写完请回读确认。**

#### `set_feedforward_mask`

`Arm::set_ff_mask(mask)`：前馈使能掩码。`FF_ALL` 之外的位由 SDK 拒绝，而不是被折成"全关"——
那等于静默关掉重力补偿。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mask`（请求） | `uint32` | 只接受 `FF_ALL` 范围内的位。 |

```bash
ros2 service call /litearm/set_feedforward_mask litearm_msgs/srv/SetFeedforwardMask "{mask: 511}"
```

#### `set_feedforward_preset`

`Arm::ff_preset(preset)`。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `preset`（请求） | `int32` | 0 全关 / 1 出厂 / 2 全开。 |
| 门禁 | | 其它值在构造任何帧之前就被拒绝。 |

```bash
ros2 service call /litearm/set_feedforward_preset litearm_msgs/srv/SetFeedforwardPreset "{preset: 1}"
```

#### `set_feedforward_scalar`

`Arm::set_ff_scalar(item, sub, value)`。item 编号见 `litearm::Arm::ff_scalar_items()`；
`sub` 只对 item 5、6 有意义。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 标量 item 号。 |
| `sub`（请求） | `int32` | 子索引；只对 item 5、6 有意义（0..2）。 |
| `value`（请求） | `float64` | 要写入的数值；固件会钳制。 |

```bash
ros2 service call /litearm/set_feedforward_scalar litearm_msgs/srv/SetFeedforwardScalar \
  "{item: 4, sub: 0, value: 0.8}"
```

#### `set_feedforward_vector`

`Arm::set_ff_vec(item, values)`。item 编号见 `litearm::Arm::ff_vec_items()`：item 1..5、7、8 为
7 个值；重力向量（item 6）为 3 个值。长度不对由 SDK 拒绝；空列表或非有限值在这里就被拒。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 向量 item 号。 |
| `values`（请求） | `float64[]` | 非空且全部有限；长度必须与 item 匹配。 |

```bash
ros2 service call /litearm/set_feedforward_vector litearm_msgs/srv/SetFeedforwardVector \
  "{item: 7, values: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

#### `get_feedforward_scalar`

`Arm::get_ff_scalar(item, sub)`：回读通路。item 9 是只读扩展，返回 `ff_mask`。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 标量 item 号；9 读 `ff_mask`。 |
| `sub`（请求） | `int32` | 子索引；只对 item 5、6 有意义。 |
| `value`（响应） | `float64` | 固件里实际持有的值。 |

```bash
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 9, sub: 0}"
```

### 关节参数

#### `get_joint_params`

`JointParams::get_joint_param(joint)` 或 `all_joint_params()`。读一个轴或整张表。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引；`-1` 表示全部轴。 |
| `params`（响应） | `litearm_msgs/JointParam[]` | 每个被请求的轴一条记录。 |
| 门禁 | | 超出固件上报轴数的索引会被拒绝。 |

每条 `JointParam` 携带 `index`、`kp`、`kd`、`tau_max`、`q_min`、`q_max`。

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
```

#### `set_joint_gains`

`JointParams::set_joint_param(joint, kp, kd, tau_max)`。这三个是固件自身位置环的增益；
ros2_control 组件自己不发任何增益。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引。 |
| `kp`、`kd`、`tau_max`（请求） | `float64` | 该轴的位置增益、阻尼增益与力矩上限。 |

```bash
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
```

#### `set_joint_limits`

`JointParams::set_joint_limits(joint, q_min, q_max)`。主机在连接时缓存限位，因此 SDK 的本地预检
要等下次 connect 才会用上新值，不是立即生效。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引。 |
| `q_min`、`q_max`（请求） | `float64` | 软限位，弧度。 |

```bash
ros2 service call /litearm/set_joint_limits litearm_msgs/srv/SetJointLimits \
  "{joint: 0, q_min: -1.0, q_max: 1.0}"
```

#### `reset_factory_params`

`JointParams::reset_factory()`。要求电机已失能。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常且电机已失能。 |

```bash
ros2 service call /litearm/reset_factory_params std_srvs/srv/Trigger "{}"
```

#### `save_params`

`Arm::save_params()`：把运行时参数写入 Flash。要求电机已失能。Flash 写入不可逆。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常且电机已失能。 |

```bash
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### 维护

#### `enter_dfu`

`Arm::enter_dfu()`：让设备登记进入 ROM bootloader。ACK 之后 CDC 设备会消失，本节点在重新用新链路
configure 之前不可用；设备会重新枚举成 `0483:DF11`，必须烧入固件才能回来。

**不要**为了"试一下"调用 `enter_dfu`。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | `allow_dfu:=true`、电机已失能、ROM 向量表有效。 |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

### 拒绝规则

每条门禁都在构造任何帧之前拒绝，因此被拒的调用不会在链路上留下任何字节。测试
`RefusedArgumentsSendNoFrame` 覆盖了这一点。

| 拒绝信息 | 触发条件 | 为什么在这里拒绝而不是转发 |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` 超出范围 | 固件收的是整数百分比：`1` 是百分之一速，不是全速。 |
| `preset must be 0 ... 2` | `set_feedforward_preset` 超出范围 | 只有三档预设。 |
| `period must be in [0.005, 0.10)` | `zero_g` 遇到越界的保活周期 | 固件命令看门狗在 0.10 s 触发，更慢的保活会静默掉出零重力。 |
| `mass must be in [0, 20] kg` | `set_payload` 超出范围 | 固件会静默钳制；存进去的值将与请求值不同。 |
| `requires the motors to be disabled` | 使能状态下调用 `save_params`、`reset_factory_params`、`activate_license` | 写 Flash 与激活只有在没有力矩权限时才是安全的。 |
| `set the parameter ... to true` | 开关参数为 false 时调用 `enter_dfu`、`activate_license` | 两者都不可逆或涉及信任，默认关闭。 |
| `not connected` | 任何在链路不可用时调用的服务 | 点出常见原因：USB 线、24 V 供电，或控制栈占着串口。 |
| `axis index must be 0..N-1` | 关节服务传入了固件未上报的轴号 | 否则一个笔误就会写到另一个轴上。 |

这些预检是尽力而为：`save_params` 靠缓存的状态帧判断电机是否使能。SDK 与固件会在链路上执行同一条
规则，所以缓存过期最多把一次干净的拒绝变成固件自己的错误消息，绝不会变成一次不安全的写入。

### 未暴露的能力

轨迹执行、笛卡尔伺服与遥操作、固件笛卡尔规划、模型导入、日志采集、`kin_bench` 在这里都没有对应
服务。运动控制属于 ros2_control 栈（`joint_trajectory_controller`、MoveIt 2）；伺服需要命令通路上
有控制器，而不是一个每次只交接一个位姿的服务。原因写在
[`docs/command-set.zh-CN.md`](../docs/command-set.zh-CN.md) 里，那份文档还包含持有权规则、错误语义
与兼容性表。

## 测试

测试套件完全离线：它注入 SDK 的 `FakeTransport`，不接机械臂就能跑完整个生命周期与每个服务处理器：

```bash
colcon test --packages-select litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。

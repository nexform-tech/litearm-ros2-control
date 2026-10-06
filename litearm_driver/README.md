# litearm_driver

A standalone ROS 2 lifecycle node that owns the LiteArm arm's USB link and exposes the
SDK's administrative command set over services, for maintaining, licensing or tuning an
arm without a controller_manager in the loop.

The node talks to the firmware through the [litearm-cpp](https://github.com/nexform-tech/litearm-cpp)
SDK. It creates its services on activation and destroys them on deactivation, and it
serialises every SDK call through one callback group.

**This node is mutually exclusive with the ros2_control stack.** Both open the same serial
port and the SDK takes an exclusive `flock` on it, so the second process fails to open the
port. Stop one before starting the other.

## Run

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

The launch file puts the node in the `litearm` namespace, so its relative service names
become `/litearm/enable`, `/litearm/clear_faults` and so on.

## Interfaces

- Topics: `/litearm/status` (`litearm_msgs/msg/LitearmStatus`), `/diagnostics`,
  `/joint_states` (off by default).
- Services: 24, split across state and safety, motion configuration, feedforward, joint
  parameters, licence and DFU.
- Messages and services: [`litearm_msgs`](../litearm_msgs/README.md).

[`docs/command-set.md`](../docs/command-set.md) is the full reference: every service, its
request fields, the SDK call behind it, the refusals and the reason for each one.

## Parameters

[`config/litearm_driver.yaml`](config/litearm_driver.yaml) holds the defaults. The two
opt-in switches (`allow_dfu`, `allow_license_activation`) and the zero-gravity keep-alive
period are re-read on every call, so `ros2 param set` takes effect without a restart.

**Do not** set `publish_joint_states:=true` while a `joint_state_broadcaster` publishes the
same topic: two publishers on one `joint_states` topic make the arm's state ambiguous.

## Tests

The suite is offline. It injects the SDK's `FakeTransport` and drives the whole lifecycle
and every service handler without an arm attached:

```bash
colcon test --packages-select litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## License

Apache-2.0. See [LICENSE](../LICENSE).

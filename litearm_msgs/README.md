# litearm_msgs — the message and service definitions of the LiteArm maintenance driver

Interfaces used by `litearm_driver`, the standalone ROS 2 node that owns the arm's USB
link while the ros2_control stack is not running. Read this when you write a client
against the driver, or add an interface the driver must expose.

## Messages

| Message | Purpose |
| --- | --- |
| `LitearmStatus` | One snapshot of the arm: link, enable, fault, mode, per-axis diagnostics, licence. Published on `/litearm/status` and returned by the `get_status` service. |
| `JointParam` | One axis of the firmware's joint parameter table: `kp`, `kd`, `tau_max`, `q_min`, `q_max`. |

## Services

Fourteen services carry values; the rest of the driver's surface uses `std_srvs`.

| Service | Purpose |
| --- | --- |
| `GetStatus` | On-demand status snapshot with a timeout, instead of waiting for the topic. |
| `GetLicense` | Read the device licence record. |
| `ActivateLicense` | Submit a vendor-signed activation credential. |
| `GetJointParams` | Read one axis or the whole joint parameter table. |
| `SetJointGains` | Write `kp`/`kd`/`tau_max` for one axis. |
| `SetJointLimits` | Write the soft limits of one axis. |
| `SetMotionMode` | Write the firmware motion mode (mode 0 only in the current firmware). |
| `SetSpeedScaling` | Write the global speed governor, in percent. |
| `SetPayload` | Declare the end-effector mass and centre of mass. |
| `SetFeedforwardMask` | Write the feedforward enable mask. |
| `SetFeedforwardPreset` | Select a feedforward preset (off, factory, all on). |
| `SetFeedforwardScalar` | Write one feedforward scalar. |
| `SetFeedforwardVector` | Write one feedforward vector. |
| `GetFeedforwardScalar` | Read one feedforward scalar or the feedforward mask back. |

The full command set, including the `std_srvs` services and the safety rules behind them,
is in [`docs/command-set.md`](../docs/command-set.md).

## License

Apache-2.0. See [LICENSE](../LICENSE).

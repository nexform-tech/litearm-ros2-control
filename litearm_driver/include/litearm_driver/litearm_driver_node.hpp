// litearm_driver_node.hpp — the standalone maintenance driver for the LiteArm arm.
//
// This node owns the arm's USB CDC link itself, through the litearm C++ SDK, and exposes
// the SDK's administrative command set as ROS 2 services plus a status topic. It is the
// counterpart of the ros2_control hardware component in this repository, not a companion:
// both open the same serial port, and the SDK takes an exclusive flock on it, so running
// the two at once is impossible by construction. Start this node when you want to
// diagnose, calibrate, license or tune the arm without a controller_manager in the loop.
//
// Design notes
// ------------
// * Services are created on activation and destroyed on deactivation, so an inactive node
//   advertises nothing it cannot serve.
// * Every SDK call is serialised through one mutually exclusive callback group. The SDK
//   blocks on the firmware ACK for up to its own timeout, so a service call can delay the
//   status publication; that is the price of never interleaving two commands on the wire.
// * The status publisher reads only cached or local SDK state. It never sends a frame, so
//   subscribing cannot disturb a command in flight.

#ifndef LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_
#define LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <litearm_msgs/msg/litearm_status.hpp>
#include <litearm_msgs/srv/activate_license.hpp>
#include <litearm_msgs/srv/get_feedforward_scalar.hpp>
#include <litearm_msgs/srv/get_joint_params.hpp>
#include <litearm_msgs/srv/get_license.hpp>
#include <litearm_msgs/srv/get_status.hpp>
#include <litearm_msgs/srv/set_feedforward_mask.hpp>
#include <litearm_msgs/srv/set_feedforward_preset.hpp>
#include <litearm_msgs/srv/set_feedforward_scalar.hpp>
#include <litearm_msgs/srv/set_feedforward_vector.hpp>
#include <litearm_msgs/srv/set_joint_gains.hpp>
#include <litearm_msgs/srv/set_joint_limits.hpp>
#include <litearm_msgs/srv/set_motion_mode.hpp>
#include <litearm_msgs/srv/set_payload.hpp>
#include <litearm_msgs/srv/set_speed_scaling.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "litearm/arm.hpp"
#include "litearm_driver/safety_gates.hpp"

namespace litearm_driver
{

/// Lifecycle node that exposes the litearm SDK's administrative command set over ROS 2.
///
/// Parameters (all optional):
///
///   port                         USB CDC device path. Empty (the default) triggers the
///                                SDK's auto-discovery by VID:PID 1d50:606f.
///   joint_names                  Axis names, in firmware axis order. Empty (the default)
///                                derives joint1..jointN from what the firmware reports.
///                                A non-empty list of the wrong length fails configure.
///   auto_enable                  Enable the motors on activation. Default true. A failed
///                                enable fails the activation, which is what keeps a
///                                refused ENABLE visible instead of silently inactive.
///   enable_attempts              enable() retries. Default 12.
///   status_rate_hz               /litearm/status publication rate. Default 10.
///   publish_joint_states         Publish the optional joint_states topic. Default false:
///                                when the ros2_control stack runs, its
///                                joint_state_broadcaster owns that topic, and this node
///                                must not compete with it.
///   joint_states_topic           Topic for the optional joint states. Default
///                                /joint_states.
///   publish_diagnostics          Publish /diagnostics. Default true.
///   diagnostics_rate_hz          /diagnostics publication rate. Default 1.
///   zero_g_keepalive_period_s    Zero-gravity keep-alive period, [0.005, 0.10). Default
///                                0.04.
///   allow_dfu                    Allow the enter_dfu service. Default false.
///   allow_license_activation     Allow the activate_license service. Default false.
///   frame_id                     Header frame id of the status and joint state messages.
///                                Default empty.
class LitearmDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn;

  explicit LitearmDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LitearmDriverNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /// Factory seam for the SDK object.
  ///
  /// Production builds a real `litearm::Arm`; the offline test overrides this to inject
  /// `litearm::testing::FakeTransport` and run the whole lifecycle without hardware.
  virtual std::unique_ptr<litearm::Arm> create_arm(const litearm::ArmOptions & options);

  // ── service handlers ─────────────────────────────────────────────────────────────
  // Protected rather than private so the offline test can call them directly: no client,
  // no executor, no spin.

  void handle_enable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_disable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_clear_faults(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_emergency_stop(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_park(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_save_params(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_reset_factory_params(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_enter_dfu(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_zero_g(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  void handle_get_status(
    const litearm_msgs::srv::GetStatus::Request::SharedPtr request,
    litearm_msgs::srv::GetStatus::Response::SharedPtr response);
  void handle_get_license(
    const litearm_msgs::srv::GetLicense::Request::SharedPtr request,
    litearm_msgs::srv::GetLicense::Response::SharedPtr response);
  void handle_activate_license(
    const litearm_msgs::srv::ActivateLicense::Request::SharedPtr request,
    litearm_msgs::srv::ActivateLicense::Response::SharedPtr response);
  void handle_set_speed_scaling(
    const litearm_msgs::srv::SetSpeedScaling::Request::SharedPtr request,
    litearm_msgs::srv::SetSpeedScaling::Response::SharedPtr response);
  void handle_set_motion_mode(
    const litearm_msgs::srv::SetMotionMode::Request::SharedPtr request,
    litearm_msgs::srv::SetMotionMode::Response::SharedPtr response);
  void handle_set_payload(
    const litearm_msgs::srv::SetPayload::Request::SharedPtr request,
    litearm_msgs::srv::SetPayload::Response::SharedPtr response);
  void handle_set_feedforward_mask(
    const litearm_msgs::srv::SetFeedforwardMask::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardMask::Response::SharedPtr response);
  void handle_set_feedforward_preset(
    const litearm_msgs::srv::SetFeedforwardPreset::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardPreset::Response::SharedPtr response);
  void handle_set_feedforward_scalar(
    const litearm_msgs::srv::SetFeedforwardScalar::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardScalar::Response::SharedPtr response);
  void handle_set_feedforward_vector(
    const litearm_msgs::srv::SetFeedforwardVector::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardVector::Response::SharedPtr response);
  void handle_get_feedforward_scalar(
    const litearm_msgs::srv::GetFeedforwardScalar::Request::SharedPtr request,
    litearm_msgs::srv::GetFeedforwardScalar::Response::SharedPtr response);
  void handle_get_joint_params(
    const litearm_msgs::srv::GetJointParams::Request::SharedPtr request,
    litearm_msgs::srv::GetJointParams::Response::SharedPtr response);
  void handle_set_joint_gains(
    const litearm_msgs::srv::SetJointGains::Request::SharedPtr request,
    litearm_msgs::srv::SetJointGains::Response::SharedPtr response);
  void handle_set_joint_limits(
    const litearm_msgs::srv::SetJointLimits::Request::SharedPtr request,
    litearm_msgs::srv::SetJointLimits::Response::SharedPtr response);

  /// The SDK is reachable and must be used now.
  bool connected() const;

  /// True when the firmware currently reports the motors enabled.
  bool motors_enabled() const;

  /// Build a status message. `fresh` is used when it is set (the on-demand service), and
  /// the cached frame otherwise (the publisher).
  litearm_msgs::msg::LitearmStatus build_status(const litearm::RobotState * fresh);

  /// Fill the cached licence record by reading the device. Blocking, so it is called on
  /// configure, on activation and by the licence services — never by a timer.
  void refresh_license_record();

  /// Cached frame, or nullopt when this session has not seen one yet.
  const litearm::RobotState * cached_state();

private:
  void read_parameters();
  void create_service_servers();
  void destroy_service_servers();
  void activate_publishers();
  void deactivate_publishers();
  void start_timers();
  void stop_timers();
  void publish_status();
  void publish_diagnostics();
  void start_zero_g(double period_s);
  void stop_zero_g();

  // Parameters, declared in the constructor and read in on_configure. The two opt-in
  // switches are read again at call time, so `ros2 param set` can disable a service
  // without a restart.
  std::string port_;
  std::vector<std::string> joint_names_;
  bool auto_enable_ = true;
  int enable_attempts_ = 12;
  double status_rate_hz_ = 10.0;
  bool publish_joint_states_ = false;
  std::string joint_states_topic_ = "/joint_states";
  bool publish_diagnostics_ = true;
  double diagnostics_rate_hz_ = 1.0;
  std::string frame_id_;

  std::unique_ptr<litearm::Arm> arm_;

  // Advertised while active; destroyed on deactivate. The typed handles are not needed
  // after creation, so the base-class handle is enough to keep them alive.
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  rclcpp_lifecycle::LifecyclePublisher<litearm_msgs::msg::LitearmStatus>::SharedPtr
    status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  /// One mutually exclusive group for every SDK call: services and timers never run
  /// concurrently, so two commands can never interleave on the wire.
  rclcpp::CallbackGroup::SharedPtr sdk_group_;

  // Cached state. Nothing here is read from the firmware by a timer.
  bool license_valid_ = false;
  litearm::LicenseInfo license_;
  uint8_t speed_scaling_ = 100;
  litearm::RobotState last_state_;
  bool have_state_ = false;
  /// SDK arrival time of the cached frame, in the SDK's own seconds (monotonic).
  double last_state_stamp_ = 0.0;
};

}  // namespace litearm_driver

#endif  // LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_

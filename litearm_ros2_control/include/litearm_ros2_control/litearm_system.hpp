// litearm_system.hpp — ros2_control SystemInterface for the litearm arm (direct USB).
//
// This plugin owns the arm's USB CDC connection itself, through the litearm C++ SDK
// (litearm::Arm). There is no helper process and no shared memory: read() consumes the
// SDK's cached 100 Hz status frame; write() streams the command reference back with the
// SDK's `move_js` (joint position + velocity) primitive.
//
// Interface mapping
// -----------------
// Per joint (the URDF declares only these):
//
//   command  position  -> firmware q_ref   (MOVE_JS)
//   command  velocity  -> firmware dq_ref  (MOVE_JS)
//   state    position  -> measured q
//   state    velocity  -> measured dq
//   state    effort    -> measured tau     (current estimate; noisy, not for control)
//
// Opt-in diagnostic state interfaces (export_diagnostic_interfaces=true, the default):
//
//   temperature_mos, temperature_coil, error_code, feedback_age
//
// The firmware runs the position loop itself:
//   tau = kp*(q_ref - q) + kd*(dq_ref - dq) + feedforward(q, dq)
// with kp/kd and the gravity/friction/integral terms taken from the firmware parameter
// table. The plugin neither sends gains nor computes dynamics — it is a joint-level
// position/velocity servo, which is what joint_trajectory_controller / MoveIt expect.
//
// Streaming contract
// ------------------
// A MOVE_JS frame whose dq is all zero is rejected whole (ERR{0x03,0x02}) when any target
// is more than JS_ZERO_DQ_EPS (5 mrad, firmware 2026-09-24) from the measurement -- and a
// rejected frame does not feed the watchdog. Under dq=0 the firmware freezes its own
// reference, so stream_command() fills the position field with the measured position in
// that case; a real trajectory carries a non-zero dq and is sent as commanded.
//
// A command frame the firmware refuses does not feed that watchdog, but write() keeps
// streaming and only stops the controllers after kWriteFailureLimit consecutive refusals.
// PARK (SET_MOTION_MODE 0) is declared once the first frame is through, so a command stream
// that stops without this process running its shutdown still holds at full stiffness.
//
// The firmware command watchdog trips after 100 ms without a command, so write() has to
// keep re-sending. A controller_manager update rate of 100 Hz or more satisfies that
// automatically. DO NOT let the update loop stall for more than ~100 ms while active:
// the firmware drops to its fail-soft hold and the arm sags toward whatever the load
// pulls it.
//
// Known tradeoff of driving the arm in-process
// --------------------------------------------
// write() calls into the SDK, which writes a frame to the serial port and waits for the
// firmware ACK (bounded by the SDK's own 1.2 s timeout). On a healthy link that is
// sub-millisecond, but a stalled USB link can stall the controller loop for up to that
// timeout. If you need a hard real-time loop isolated from USB, run the arm through the
// shared-memory + daemon variant instead.
//
// URDF parameters (all optional)
// ------------------------------
//   port                           USB CDC device path. Empty (the default) triggers the
//                                  SDK's auto-discovery by VID:PID 1d50:606f.
//   export_diagnostic_interfaces   true (default) / false
//   auto_enable                    true (default): enable() the arm in on_activate.
//                                  Set false to control enablement yourself.
//   disable_on_shutdown            false (default): leave the motors enabled on shutdown
//                                  (the firmware holds). true loses force — the arm drops.
//   enable_attempts                enable() retries, default 12 (the SDK sleeps 300 ms
//                                  between retries, so only retryable codes consume it).
//
// Joints must be named joint1..jointN and declared in the <ros2_control> block, exactly N
// of them, matching the axis count the firmware reports (7 for the arm, 1 for the bench).
// The name fixes the axis mapping, so URDF order does not matter.

#ifndef LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_
#define LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "litearm/arm.hpp"

namespace litearm_ros2_control
{

/// Standard state-interface names exported by every joint.
inline constexpr char kStatePosition[] = "position";
inline constexpr char kStateVelocity[] = "velocity";
inline constexpr char kStateEffort[] = "effort";

/// Non-standard diagnostic state-interface names (exported only when enabled).
inline constexpr char kStateTemperatureMos[] = "temperature_mos";
inline constexpr char kStateTemperatureCoil[] = "temperature_coil";
inline constexpr char kStateErrorCode[] = "error_code";
inline constexpr char kStateFeedbackAge[] = "feedback_age";

/// Command-interface names exported by every joint.
inline constexpr char kCommandPosition[] = "position";
inline constexpr char kCommandVelocity[] = "velocity";

/**
 * litearm arm as a ros2_control system component, talking to the firmware over USB CDC.
 *
 * Lifecycle
 * ---------
 * on_init       Parse URDF parameters and the joint-name -> axis map; size the interface
 *               storage. No hardware is touched here.
 * on_configure  Build the SDK `Arm`, connect (handshake + firmware version gate), and
 *               check that the URDF joint count matches the firmware axis count.
 * on_activate   Seed the command reference from the measured position (so the arm holds
 *               instead of jumping to zero), then enable the motors and stream one frame.
 * read          Copy the latest cached status frame into the state interfaces.
 * write         Stream the command reference as one `move_js` frame.
 * on_deactivate Send a final hold frame and declare park (full-stiffness static hold that
 *               does not depend on the watchdog).
 * on_cleanup    Close the link and drop the SDK object.
 */
class LitearmSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LitearmSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Factory seam for the SDK object.
   *
   * Production builds a real `litearm::Arm` (which opens the serial port on connect()).
   * Tests override this to inject `litearm::testing::FakeTransport` and exercise the whole
   * lifecycle offline. Keep it virtual and keep the default implementation honest.
   */
  virtual std::unique_ptr<litearm::Arm> create_arm(const litearm::ArmOptions & options);

private:
  /// Where the component is in its own (small) state machine.
  enum class Mode { kUnconfigured, kInactive, kActive, kStopped };

  /// Number of joints declared in the URDF.
  std::size_t num_joints() const { return info_.joints.size(); }

  /// Copy one status frame, in URDF joint order, into the state interfaces.
  void apply_state(const litearm::RobotState & state);

  /// Read the cached status frame into the state interfaces and refuse to go on when it
  /// is missing or carries a non-finite reading. `when` names the point in the lifecycle
  /// for the message ('before enabling' / 'after enabling').
  bool refresh_state(const char * when);

  /// Set the command reference to the measured position (q_ref := q, dq_ref := 0).
  void latch_command_to_measured();

  /// Send one `move_js` frame built from the current command interface values.
  void stream_command();

  Mode mode_ = Mode::kUnconfigured;
  std::unique_ptr<litearm::Arm> arm_;

  // URDF parameters.
  std::string port_;
  bool export_diagnostics_ = true;
  bool auto_enable_ = true;
  // Clear a latched arm fault during on_configure. Off by default: a latched EMERGENCY or
  // joint_fault is the firmware asking for a human decision. A deployment that wants its
  // start-up to be one command turns it on through the URDF parameter.
  bool reset_faults_on_configure_ = false;
  bool disable_on_shutdown_ = false;
  int enable_attempts_ = 12;

  // URDF joint order -> firmware axis: joint_to_axis_[i] is the axis index the firmware
  // reports for info_.joints[i], derived from the joint name (jointN -> N-1).
  std::vector<std::size_t> joint_to_axis_;

  // State storage, indexed in URDF order. Interface pointers point into these vectors,
  // so their sizes are fixed in on_init and never changed afterwards.
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;
  std::vector<double> state_temperature_mos_;
  std::vector<double> state_temperature_coil_;
  std::vector<double> state_error_code_;
  std::vector<double> state_feedback_age_;

  // Command storage, indexed in URDF order.
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;

  // Scratch buffers in firmware axis order. Preallocated so write() does not allocate.
  std::vector<double> fw_position_;
  std::vector<double> fw_velocity_;

  bool have_state_ = false;
  // Whether the last status frame carried a finite reading for every declared joint. A
  // faulted or disabled axis reports NaN, and latching that as the command reference sends
  // a frame the firmware rejects (ERR 03,02) -- so activation checks this first.
  bool state_finite_ = false;
  // The status frame's enabled bit (firmware flags bit9). The ENABLE ACK only means the
  // request was registered; the frame is what says the motors are actually on.
  bool state_enabled_ = false;
  std::size_t nonfinite_axis_ = 0;   // 1-based axis of the first non-finite reading
  int nonfinite_error_ = 0;          // that axis's firmware error byte, for the log
  bool nonfinite_reported_ = false;  // one message per episode, not one per frame
  double last_feedback_stamp_ = 0.0;  // SDK arrival time of the last status frame (s)
  bool reported_disconnected_ = false;
  // Consecutive refused command frames. A refusal does not feed the firmware watchdog, so
  // one is survivable and the next tick retries; a run of them means the link or the
  // firmware really is gone and the controllers have to stop.
  int write_failures_ = 0;
  bool write_recovering_ = false;

  rclcpp::Logger logger_ = rclcpp::get_logger("LitearmSystem");
};

}  // namespace litearm_ros2_control

#endif  // LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_

// safety_gates.hpp — argument validation for the driver's service handlers.
//
// Every rule here exists because the firmware would otherwise accept the call and do
// something other than what was asked: clamp a value silently, fold an unknown bit into
// "all off", or ACK a mode change that never happens. The rules are free functions with
// no ROS and no SDK dependency, which is what makes them testable without hardware.

#ifndef LITEARM_DRIVER__SAFETY_GATES_HPP_
#define LITEARM_DRIVER__SAFETY_GATES_HPP_

#include <array>
#include <string>
#include <vector>

namespace litearm_driver
{

/// Pass or fail, plus the text a service response carries on failure.
struct GateResult
{
  bool ok = true;
  std::string message;
};

/// The speed governor is an integer percentage, so anything outside 0..100 is a mistake.
/// In particular 1 is one percent, not full speed.
GateResult check_speed_percent(int percent);

/// Feedforward presets are 0 (all off), 1 (factory) and 2 (all on).
GateResult check_ff_preset(int preset);

/// The zero-gravity keep-alive period has to satisfy the SDK contract [0.005, 0.10). The
/// upper bound exists because the firmware command watchdog trips at 0.10 s.
GateResult check_zero_g_period(double period_s);

/// An axis index has to address an axis the firmware actually reported.
GateResult check_joint_index(int index, int joint_count);

/// The firmware clamps mass into [0, 20] kg silently, so a value outside that range would
/// be stored as a different number than the one requested. Refuse it instead.
GateResult check_mass(double mass);

/// The centre of mass has to be finite in all three components.
GateResult check_com(const std::array<double, 3> & com);

/// Feedforward vectors must be non-empty and finite. The length the firmware expects
/// depends on the item (3 for the gravity vector, 7 elsewhere) and the SDK enforces it;
/// this gate only rejects values that could never be valid.
GateResult check_ff_values(const std::vector<double> & values);

/// Flash writes and the joint parameter reset require the motors to be disabled. The SDK
/// and the firmware both refuse otherwise; refusing here first gives a message that says
/// why, before a frame goes out.
GateResult check_requires_disabled(bool enabled, const std::string & operation);

/// Some services are dangerous enough that a parameter has to opt in explicitly.
GateResult check_parameter_allows(bool allowed, const std::string & parameter_name);

/// Most services need a live link; the message names the likely cause.
GateResult check_connected(bool connected);

}  // namespace litearm_driver

#endif  // LITEARM_DRIVER__SAFETY_GATES_HPP_

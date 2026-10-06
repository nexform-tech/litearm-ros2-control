// safety_gates.cpp — implementations of the driver's argument validation.

#include "litearm_driver/safety_gates.hpp"

#include <cmath>
#include <string>

namespace litearm_driver
{

namespace
{

GateResult fail(const std::string & message)
{
  return GateResult{false, message};
}

}  // namespace

GateResult check_speed_percent(int percent)
{
  if (percent < 0 || percent > 100) {
    return fail(
      "percent must be 0..100; it is an integer percentage, so 1 means one percent of "
      "full speed, not full speed");
  }
  return GateResult{};
}

GateResult check_ff_preset(int preset)
{
  if (preset < 0 || preset > 2) {
    return fail("preset must be 0 (all off), 1 (factory) or 2 (all on)");
  }
  return GateResult{};
}

GateResult check_zero_g_period(double period_s)
{
  if (!std::isfinite(period_s)) {
    return fail("zero-gravity keep-alive period must be finite");
  }
  if (period_s < 0.005 || period_s >= 0.10) {
    return fail(
      "zero-gravity keep-alive period must be in [0.005, 0.10) seconds; the firmware "
      "command watchdog trips at 0.10 s");
  }
  return GateResult{};
}

GateResult check_joint_index(int index, int joint_count)
{
  if (joint_count <= 0) {
    return fail("the firmware has not reported any axes yet; connect first");
  }
  if (index < 0 || index >= joint_count) {
    return fail(
      "axis index must be 0.." + std::to_string(joint_count - 1) + " (the firmware "
      "reports " + std::to_string(joint_count) + " axes)");
  }
  return GateResult{};
}

GateResult check_mass(double mass)
{
  if (!std::isfinite(mass)) {
    return fail("mass must be finite");
  }
  if (mass < 0.0 || mass > 20.0) {
    return fail(
      "mass must be in [0, 20] kg; the firmware clamps silently outside that range, so "
      "the stored value would differ from the one requested");
  }
  return GateResult{};
}

GateResult check_com(const std::array<double, 3> & com)
{
  for (const double value : com) {
    if (!std::isfinite(value)) {
      return fail("centre of mass must be finite in all three components");
    }
  }
  return GateResult{};
}

GateResult check_ff_values(const std::vector<double> & values)
{
  if (values.empty()) {
    return fail("values must not be empty");
  }
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return fail("every feedforward value must be finite");
    }
  }
  return GateResult{};
}

GateResult check_requires_disabled(bool enabled, const std::string & operation)
{
  if (enabled) {
    return fail(
      operation + " requires the motors to be disabled; call disable first "
      "(the SDK and the firmware both refuse while enabled)");
  }
  return GateResult{};
}

GateResult check_parameter_allows(bool allowed, const std::string & parameter_name)
{
  if (!allowed) {
    return fail(
      "refused: set the parameter " + parameter_name + " to true to allow this service");
  }
  return GateResult{};
}

GateResult check_connected(bool connected)
{
  if (!connected) {
    return fail(
      "the arm is not connected: check the USB cable and the 24 V supply, and make sure "
      "the ros2_control stack is not holding the port");
  }
  return GateResult{};
}

}  // namespace litearm_driver

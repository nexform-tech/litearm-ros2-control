// litearm_system.cpp — LitearmSystem implementation (direct-USB ros2_control system).

#include "litearm_ros2_control/litearm_system.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <exception>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "litearm/clock.hpp"
#include "litearm/protocol.hpp"
#include "litearm/state.hpp"

namespace litearm_ros2_control
{

namespace
{

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

/// Parse a URDF boolean. Unrecognised text falls back so a typo cannot silently disable a
/// safety-relevant flag while looking like it took effect.
bool parse_bool(const std::string & raw, bool fallback)
{
  const std::string value = to_lower(trim(raw));
  if (value.empty()) {
    return fallback;
  }
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }
  return fallback;
}

/// Map a joint name to a firmware axis: jointN -> N-1, with N in 1..MAX_JOINTS.
///
/// The name is the mapping contract (see the header): the firmware reports axes in a fixed
/// order, and a name-keyed map keeps the plugin correct even if the URDF lists the joints
/// out of order. Returns false for any name that does not fit the pattern.
bool joint_name_to_axis(const std::string & name, std::size_t * axis)
{
  constexpr const char * kPrefix = "joint";
  const std::string prefix(kPrefix);
  if (name.rfind(prefix, 0) != 0) {
    return false;
  }
  const std::string digits = name.substr(prefix.size());
  if (digits.empty() || digits.size() > 2) {
    return false;
  }
  for (const char c : digits) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
      return false;
    }
  }
  const int number = std::stoi(digits);
  if (number < 1 || number > litearm::proto::MAX_JOINTS) {
    return false;
  }
  *axis = static_cast<std::size_t>(number - 1);
  return true;
}

}  // namespace

// ---------------------------------------------------------------- factory seam

std::unique_ptr<litearm::Arm> LitearmSystem::create_arm(const litearm::ArmOptions & options)
{
  return std::make_unique<litearm::Arm>(options);
}

// ---------------------------------------------------------------- lifecycle

hardware_interface::CallbackReturn LitearmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger(info_.name.empty() ? "LitearmSystem" : info_.name);

  if (info_.joints.empty())
  {
    RCLCPP_FATAL(
      logger_, "No joints declared. The litearm hardware needs one <joint> per axis "
      "(joint1..jointN).");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() > static_cast<std::size_t>(litearm::proto::MAX_JOINTS))
  {
    RCLCPP_FATAL(
      logger_, "URDF declares %zu joints, but the litearm firmware has at most %d axes.",
      info_.joints.size(), litearm::proto::MAX_JOINTS);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Build the URDF order -> firmware axis map and reject duplicates. A duplicate would mean
  // two URDF joints commanding the same physical axis, which no controller would notice.
  joint_to_axis_.assign(num_joints(), 0);
  std::vector<bool> seen(num_joints(), false);
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    std::size_t axis = 0;
    if (!joint_name_to_axis(info_.joints[i].name, &axis) || axis >= num_joints())
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' does not map to a litearm axis. Names must be "
        "joint1..joint%zu, one per axis.", info_.joints[i].name.c_str(), num_joints());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (seen[axis])
    {
      RCLCPP_FATAL(logger_, "Joint %s is declared twice.", info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    seen[axis] = true;
    joint_to_axis_[i] = axis;
  }

  // ── URDF parameters ──
  const auto param = [this](const std::string & key) -> std::string {
      const auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? std::string() : trim(it->second);
    };

  port_ = param("port");
  export_diagnostics_ = parse_bool(param("export_diagnostic_interfaces"), true);
  auto_enable_ = parse_bool(param("auto_enable"), true);
  disable_on_shutdown_ = parse_bool(param("disable_on_shutdown"), false);

  const std::string attempts_raw = param("enable_attempts");
  if (!attempts_raw.empty())
  {
    try
    {
      enable_attempts_ = std::max(1, std::stoi(attempts_raw));
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(
        logger_, "enable_attempts='%s' is not an integer, using %d.",
        attempts_raw.c_str(), enable_attempts_);
    }
  }

  // ── Interface storage ──
  // Sizes are fixed here: export_state_interfaces() / export_command_interfaces() hand out
  // pointers into these vectors, so they must never be resized after this point.
  state_position_.assign(num_joints(), 0.0);
  state_velocity_.assign(num_joints(), 0.0);
  state_effort_.assign(num_joints(), 0.0);
  state_temperature_mos_.assign(num_joints(), 0.0);
  state_temperature_coil_.assign(num_joints(), 0.0);
  state_error_code_.assign(num_joints(), 0.0);
  state_feedback_age_.assign(num_joints(), -1.0);
  command_position_.assign(num_joints(), 0.0);
  command_velocity_.assign(num_joints(), 0.0);
  fw_position_.assign(num_joints(), 0.0);
  fw_velocity_.assign(num_joints(), 0.0);

  RCLCPP_INFO(
    logger_, "Initialized: %zu joints, port='%s', diagnostics=%s, auto_enable=%s.",
    num_joints(), port_.empty() ? "auto-discover" : port_.c_str(),
    export_diagnostics_ ? "on" : "off", auto_enable_ ? "on" : "off");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  litearm::ArmOptions options;
  if (!port_.empty())
  {
    options.port = port_;
  }

  // connect() opens the serial port, handshakes the firmware version, reads one status frame
  // to learn the axis count, and probes cartesian support. It throws on any failure.
  try
  {
    arm_ = create_arm(options);
    arm_->connect();
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(
      logger_, "Cannot connect to the litearm hardware: %s. Check the USB cable "
      "(lsusb | grep 1d50:606f), serial permissions (dialout group), that no other process "
      "holds the port, and that the firmware license is activated.", e.what());
    arm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (static_cast<std::size_t>(arm_->n()) != num_joints())
  {
    RCLCPP_FATAL(
      logger_, "URDF declares %zu joints but the firmware reports %d axes (%s). "
      "Declare exactly one <joint> per axis, named joint1..joint%d.",
      num_joints(), arm_->n(), arm_->firmware().c_str(), arm_->n());
    arm_->close();
    arm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  mode_ = Mode::kInactive;
  have_state_ = false;
  reported_disconnected_ = false;
  RCLCPP_INFO(
    logger_, "Configured: firmware=%s port=%s axes=%d.",
    arm_->firmware().c_str(), arm_->port_string().c_str(), arm_->n());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_ == nullptr || !arm_->is_connected())
  {
    RCLCPP_ERROR(logger_, "Not configured or link down; cannot activate.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Latch the command reference to the measured position before enabling. Without this the
  // first frame would carry the storage default (0 rad) and the arm would snap toward its
  // zero pose the instant the motors energize.
  try
  {
    const auto msg = arm_->get_state(false);
    if (msg.value)
    {
      last_feedback_stamp_ = msg.timestamp;
      apply_state(*msg.value);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Cannot read the initial state: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!have_state_)
  {
    RCLCPP_ERROR(
      logger_, "No status frame yet; refusing to activate, since the command reference "
      "cannot be seeded from the measured position.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  latch_command_to_measured();

  if (auto_enable_)
  {
    try
    {
      arm_->enable(enable_attempts_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(
        logger_, "Enable failed: %s. Clear the fault (arm.reset()/clear_faults()) or check "
        "the license, then re-activate.", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  mode_ = Mode::kActive;
  // Stream one frame now so the firmware watchdog is fed before the first write() tick.
  try
  {
    stream_command();
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(logger_, "First command frame failed: %s", e.what());
    mode_ = Mode::kStopped;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Activated: command reference seeded from the measured position%s.",
    auto_enable_ ? ", motors enabled" : " (auto_enable=false: motors left as-is)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (mode_ == Mode::kActive && arm_ != nullptr && arm_->is_connected())
  {
    // Hand control back at the last measured position, then declare park. park() selects the
    // full-stiffness static hold, so the arm keeps holding even if nothing streams again —
    // unlike relying on the 100 ms watchdog fail-soft.
    latch_command_to_measured();
    try
    {
      stream_command();
      arm_->park();
      RCLCPP_INFO(logger_, "Deactivated: held at the measured position (motors stay enabled).");
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(
        logger_, "Could not hand back a clean hold frame (%s). The firmware watchdog will "
        "fall back to its fail-soft hold.", e.what());
    }
  }
  mode_ = Mode::kStopped;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_ != nullptr)
  {
    try
    {
      arm_->close();
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(logger_, "Ignoring error while closing the link: %s", e.what());
    }
    arm_.reset();
  }
  have_state_ = false;
  reported_disconnected_ = false;
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_ != nullptr && arm_->is_connected() && disable_on_shutdown_)
  {
    try
    {
      arm_->disable();
      RCLCPP_WARN(
        logger_, "disable_on_shutdown=true: motors are now disabled and the arm is no "
        "longer held — make sure it is supported.");
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(logger_, "disable on shutdown failed: %s", e.what());
    }
  }

  if (arm_ != nullptr)
  {
    try
    {
      arm_->close();
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(logger_, "Ignoring error while closing the link: %s", e.what());
    }
    arm_.reset();
  }
  have_state_ = false;
  reported_disconnected_ = false;
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Be honest about who is holding the arm. If the link is down, nothing is streaming and
  // the firmware is on its fail-soft hold; if the process lost the port entirely, the arm
  // is only held by whatever the firmware last latched.
  if (arm_ != nullptr && arm_->is_connected())
  {
    RCLCPP_ERROR(
      logger_, "Hardware error. The link is up, so the firmware keeps the arm on its hold "
      "(park/fail-soft). Re-configure and re-activate once the cause is cleared.");
  }
  else
  {
    RCLCPP_ERROR(
      logger_, "Hardware error with the link down. The arm is held only by the firmware's "
      "last latched hold — support it if gravity, not the hold, is what keeps it up.");
  }
  mode_ = Mode::kStopped;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- interfaces

std::vector<hardware_interface::StateInterface> LitearmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(num_joints() * (export_diagnostics_ ? 7u : 3u));
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    const std::string & name = info_.joints[i].name;
    interfaces.emplace_back(name, kStatePosition, &state_position_[i]);
    interfaces.emplace_back(name, kStateVelocity, &state_velocity_[i]);
    interfaces.emplace_back(name, kStateEffort, &state_effort_[i]);
    if (!export_diagnostics_)
    {
      continue;
    }
    // joint_state_broadcaster does not claim these, so controller_manager reports them as
    // unclaimed on activation — expected, and the reason the flag exists.
    interfaces.emplace_back(name, kStateTemperatureMos, &state_temperature_mos_[i]);
    interfaces.emplace_back(name, kStateTemperatureCoil, &state_temperature_coil_[i]);
    interfaces.emplace_back(name, kStateErrorCode, &state_error_code_[i]);
    interfaces.emplace_back(name, kStateFeedbackAge, &state_feedback_age_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> LitearmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(num_joints() * 2u);
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    const std::string & name = info_.joints[i].name;
    interfaces.emplace_back(name, kCommandPosition, &command_position_[i]);
    interfaces.emplace_back(name, kCommandVelocity, &command_velocity_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------- data exchange

void LitearmSystem::apply_state(const litearm::RobotState & state)
{
  const double now = litearm::steady_clock_instance().now_s();
  const double age = last_feedback_stamp_ > 0.0 ? now - last_feedback_stamp_ : -1.0;
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    const std::size_t axis = joint_to_axis_[i];
    if (axis >= state.joints.size())
    {
      continue;  // firmware reported fewer axes than the URDF declared
    }
    const litearm::JointState & j = state.joints[axis];
    state_position_[i] = j.q;
    state_velocity_[i] = j.dq;
    state_effort_[i] = j.tau;
    state_temperature_mos_[i] = j.t_mos;
    state_temperature_coil_[i] = j.t_coil;
    state_error_code_[i] = static_cast<double>(j.err);
    // The SDK exposes one arrival timestamp per status frame, so every joint reports the
    // same age. It is the freshness of the frame, not a per-axis measurement.
    state_feedback_age_[i] = age;
  }
  have_state_ = true;
}

void LitearmSystem::latch_command_to_measured()
{
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    command_position_[i] = have_state_ ? state_position_[i] : 0.0;
    command_velocity_[i] = 0.0;
  }
}

void LitearmSystem::stream_command()
{
  for (std::size_t i = 0; i < num_joints(); ++i)
  {
    const std::size_t axis = joint_to_axis_[i];
    fw_position_[axis] = command_position_[i];
    fw_velocity_[axis] = command_velocity_[i];
  }
  arm_->move_js(fw_position_, fw_velocity_);
}

hardware_interface::return_type LitearmSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (mode_ != Mode::kActive || arm_ == nullptr)
  {
    return hardware_interface::return_type::OK;
  }

  // is_connected() is false when the reader thread died or no status frame arrived within
  // the SDK's staleness window. Reporting ERROR here stops the controllers instead of
  // letting them plan against a frozen pose.
  if (!arm_->is_connected())
  {
    if (!reported_disconnected_)
    {
      reported_disconnected_ = true;
      RCLCPP_ERROR(
        logger_, "Link to the litearm hardware is down (reader thread died or no status "
        "frame for over 2 s). Reporting ERROR so the controllers stop.");
    }
    return hardware_interface::return_type::ERROR;
  }
  reported_disconnected_ = false;

  try
  {
    const auto msg = arm_->get_state(false);  // cached frame; does not block
    if (!msg.value)
    {
      // Link is up but no frame decoded yet this session: keep the last values.
      return hardware_interface::return_type::OK;
    }
    last_feedback_stamp_ = msg.timestamp;
    apply_state(*msg.value);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Failed to read the hardware state: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LitearmSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (mode_ != Mode::kActive || arm_ == nullptr)
  {
    return hardware_interface::return_type::OK;
  }

  // MOVE_JS is a single-shot frame: the firmware watchdog needs one every <100 ms, so this
  // runs every control tick. The SDK blocks on the firmware ACK (bounded by its own 1.2 s
  // timeout) — see the tradeoff note in the header.
  try
  {
    stream_command();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Failed to write the command frame: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace litearm_ros2_control

PLUGINLIB_EXPORT_CLASS(
  litearm_ros2_control::LitearmSystem, hardware_interface::SystemInterface)

// Tests for LitearmSystem: URDF parameter parsing, joint-name -> axis mapping, exported
// interface sets, and a full lifecycle run against the SDK's FakeTransport (no hardware).

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>

#include "litearm/arm.hpp"
#include "litearm/protocol.hpp"
#include "litearm/testing.hpp"
#include "litearm/transport.hpp"

#include "litearm_ros2_control/litearm_system.hpp"

namespace litearm_ros2_control
{
namespace
{

using hardware_interface::CallbackReturn;

/// Delegates to a shared FakeTransport so the test keeps a handle on the fake while the
/// Arm owns the Transport handed to it. (The SDK's own `lt::SharedFake` lives in its test
/// tree and is not installed, so this package carries its own copy.)
class SharedFake : public litearm::Transport
{
public:
  explicit SharedFake(std::shared_ptr<litearm::testing::FakeTransport> fake)
  : fake_(std::move(fake)) {}

  void write_frame(uint8_t cmd, const uint8_t * payload, size_t len) override
  {
    fake_->write_frame(cmd, payload, len);
  }
  std::optional<litearm::proto::Frame> read_frame(double timeout) override
  {
    return fake_->read_frame(timeout);
  }
  void close() override { fake_->close(); }
  bool is_open() const override { return fake_->is_open(); }
  std::string text_log() const override { return fake_->text_log(); }
  std::string port_name() const override { return fake_->port_name(); }
  uint64_t flush_failures() const override { return fake_->flush_failures(); }

private:
  std::shared_ptr<litearm::testing::FakeTransport> fake_;
};

/// LitearmSystem that builds its Arm against a FakeTransport instead of a serial port.
class TestSystem : public LitearmSystem
{
public:
  explicit TestSystem(std::vector<double> initial_q)
  : initial_q_(std::move(initial_q)) {}

  std::shared_ptr<litearm::testing::FakeTransport> fake() const { return fake_; }

protected:
  std::unique_ptr<litearm::Arm> create_arm(const litearm::ArmOptions & options) override
  {
    litearm::ArmOptions opts = options;
    opts.port = std::string("fake");
    auto shared = std::make_shared<litearm::testing::FakeTransport>(
      "fake", 0.2, "Litearm1.7.0-7J", static_cast<int>(initial_q_.size()));
    shared->set_q(initial_q_);
    fake_ = shared;
    opts.transport_factory =
      [shared](const std::string &) -> std::unique_ptr<litearm::Transport> {
        return std::make_unique<SharedFake>(shared);
      };
    return std::make_unique<litearm::Arm>(opts);
  }

private:
  std::vector<double> initial_q_;
  std::shared_ptr<litearm::testing::FakeTransport> fake_;
};

hardware_interface::InterfaceInfo make_interface(const std::string & name)
{
  hardware_interface::InterfaceInfo out;
  out.name = name;
  return out;
}

hardware_interface::HardwareInfo make_info(
  const std::vector<std::string> & joint_names,
  const std::unordered_map<std::string, std::string> & params = {})
{
  hardware_interface::HardwareInfo info;
  info.name = "LitearmSystem";
  info.type = "system";
  info.hardware_parameters = params;
  for (const std::string & name : joint_names)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces = {make_interface("position"), make_interface("velocity")};
    joint.state_interfaces = {
      make_interface("position"), make_interface("velocity"), make_interface("effort")};
    info.joints.push_back(joint);
  }
  return info;
}

std::vector<std::string> seven_joints()
{
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
}

/// Payload of the last CMD_MOVE_JS frame the (fake) firmware received, as f32 values.
std::vector<double> last_move_js(const TestSystem & sys, std::size_t expected)
{
  std::vector<uint8_t> payload;
  for (const auto & entry : sys.fake()->tx_snapshot())
  {
    if (entry.first == litearm::proto::CMD_MOVE_JS)
    {
      payload = entry.second;
    }
  }
  if (payload.size() != expected * sizeof(float))
  {
    return {};
  }
  return litearm::proto::unpack_f32s(payload.data(), 0, expected);
}

// ---------------------------------------------------------------- interface sets

TEST(LitearmSystemInit, ExportsStandardPlusDiagnosticInterfaces)
{
  LitearmSystem sys;
  ASSERT_EQ(sys.on_init(make_info(seven_joints())), CallbackReturn::SUCCESS);

  const auto state = sys.export_state_interfaces();
  const auto command = sys.export_command_interfaces();
  EXPECT_EQ(state.size(), 7u * 7u);      // position/velocity/effort + 4 diagnostics
  EXPECT_EQ(command.size(), 7u * 2u);    // position + velocity
}

TEST(LitearmSystemInit, DiagnosticInterfacesCanBeDisabled)
{
  LitearmSystem sys;
  const std::unordered_map<std::string, std::string> params{
    {"export_diagnostic_interfaces", "false"}};
  ASSERT_EQ(sys.on_init(make_info(seven_joints(), params)), CallbackReturn::SUCCESS);

  const auto state = sys.export_state_interfaces();
  EXPECT_EQ(state.size(), 7u * 3u);
  for (const auto & iface : state)
  {
    EXPECT_TRUE(
      iface.get_interface_name() == "position" ||
      iface.get_interface_name() == "velocity" ||
      iface.get_interface_name() == "effort") << iface.get_name();
  }
}

TEST(LitearmSystemInit, RejectsEmptyJointList)
{
  LitearmSystem sys;
  EXPECT_EQ(sys.on_init(make_info({})), CallbackReturn::ERROR);
}

TEST(LitearmSystemInit, RejectsJointNameOutsideJointNPattern)
{
  LitearmSystem sys;
  std::vector<std::string> names = seven_joints();
  names[3] = "shoulder_pan";
  EXPECT_EQ(sys.on_init(make_info(names)), CallbackReturn::ERROR);
}

TEST(LitearmSystemInit, RejectsAxisBeyondDeclaredJointCount)
{
  LitearmSystem sys;
  std::vector<std::string> names = seven_joints();
  names[6] = "joint8";  // valid axis number, but it claims an eighth axis
  EXPECT_EQ(sys.on_init(make_info(names)), CallbackReturn::ERROR);
}

TEST(LitearmSystemInit, RejectsDuplicateAxis)
{
  LitearmSystem sys;
  std::vector<std::string> names = seven_joints();
  names[5] = "joint7";  // joint7 declared twice, joint6 missing
  EXPECT_EQ(sys.on_init(make_info(names)), CallbackReturn::ERROR);
}

// ---------------------------------------------------------------- lifecycle

TEST(LitearmSystemLifecycle, FullRunAgainstFakeFirmware)
{
  const std::vector<double> measured{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  TestSystem sys(measured);
  const auto info = make_info(seven_joints(), {{"port", "fake"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  const auto state = sys.export_state_interfaces();

  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const auto time = rclcpp::Time(0);
  const auto period = rclcpp::Duration::from_seconds(0.01);

  ASSERT_EQ(sys.read(time, period), hardware_interface::return_type::OK);
  // No controller ever wrote a command, so the reference is the measured position: the arm
  // holds where it is instead of snapping to zero. With diagnostics enabled the state
  // interfaces come in blocks of 7 per joint, position first — hence the stride.
  constexpr std::size_t kStride = 7;
  ASSERT_EQ(state.size(), measured.size() * kStride);
  // The measured pose round-trips through the firmware's f32 status frame, so compare at
  // float precision rather than exactly.
  for (std::size_t i = 0; i < measured.size(); ++i)
  {
    EXPECT_NEAR(state[i * kStride].get_value(), measured[i], 1e-6) << "joint " << i + 1;
  }

  ASSERT_EQ(sys.write(time, period), hardware_interface::return_type::OK);
  const auto frame = last_move_js(sys, 2u * measured.size());
  ASSERT_EQ(frame.size(), 2u * measured.size());
  for (std::size_t i = 0; i < measured.size(); ++i)
  {
    EXPECT_NEAR(frame[i], measured[i], 1e-6) << "q_ref axis " << i;
    EXPECT_DOUBLE_EQ(frame[measured.size() + i], 0.0) << "dq_ref axis " << i;
  }

  ASSERT_EQ(sys.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, CommandReachesAxisOrderedByNameNotByUrdfPosition)
{
  const std::vector<double> measured(7, 0.0);
  TestSystem sys(measured);
  // A permuted declaration order. Name-based mapping must still land each command on its
  // physical axis; a naive "URDF index == axis" implementation would swap them here.
  const std::vector<std::string> permuted{
    "joint3", "joint1", "joint7", "joint2", "joint6", "joint4", "joint5"};
  const auto info = make_info(permuted, {{"port", "fake"}, {"auto_enable", "false"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  auto command = sys.export_command_interfaces();
  ASSERT_EQ(command.size(), 7u * 2u);

  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // Distinct reference per URDF joint: 100 for permuted index 0, 101 for index 1, ...
  //
  // A non-zero dq is required for the position field to be sent as written: with dq all
  // zero the firmware rejects any target that is more than 5 mrad from the measurement
  // (JS_ZERO_DQ_EPS), and stream_command() therefore substitutes the measurement.
  for (std::size_t i = 0; i < permuted.size(); ++i)
  {
    command[2u * i].set_value(100.0 + static_cast<double>(i));
    command[2u * i + 1u].set_value(0.5);
  }

  const auto time = rclcpp::Time(0);
  const auto period = rclcpp::Duration::from_seconds(0.01);
  ASSERT_EQ(sys.write(time, period), hardware_interface::return_type::OK);

  const auto frame = last_move_js(sys, 2u * permuted.size());
  ASSERT_EQ(frame.size(), 2u * permuted.size());
  // axis0=joint1 was declared 2nd (101), axis1=joint2 4th (103), axis2=joint3 1st (100),
  // axis3=joint4 6th (105), axis4=joint5 7th (106), axis5=joint6 5th (104), axis6=joint7
  // 3rd (102).
  const std::vector<double> expected{101.0, 103.0, 100.0, 105.0, 106.0, 104.0, 102.0};
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(frame[i], expected[i]) << "axis " << i;
  }

  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, WriteToleratesARefusedFrame)
{
  // A refused frame does not feed the watchdog, but it must not stop the controllers: the
  // next tick feeds again. Only a run of failures escalates (kWriteFailureLimit).
  TestSystem sys(std::vector<double>(7, 0.1));
  const auto info = make_info(seven_joints(), {{"port", "fake"}, {"auto_enable", "false"}});
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const auto time = rclcpp::Time(0);
  const auto period = rclcpp::Duration::from_seconds(0.01);
  sys.fake()->push_frame(
    litearm::proto::RSP_ERR,
    std::vector<uint8_t>{litearm::proto::CMD_MOVE_JS, 0x02});
  EXPECT_EQ(sys.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(sys.write(time, period), hardware_interface::return_type::OK);

  ASSERT_EQ(sys.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, DeclaresParkOnceActivated)
{
  // PARK is what makes an unexpected stop (crash, cable) hold at full stiffness instead of
  // the fail-soft 0.6x sag, so it has to be declared at activation, not only on shutdown.
  TestSystem sys(std::vector<double>(7, 0.1));
  const auto info = make_info(seven_joints(), {{"port", "fake"}, {"auto_enable", "false"}});
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.fake()->stamps_of(litearm::proto::CMD_SET_MOTION_MODE).size(), 1u);
  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, RetriesTheSeedFrameAfterARefusal)
{
  // The firmware refuses the first frame after enabling when a zero-dq target is more than
  // 5 mrad from its measurement (ERR{0x03,0x02}). One refusal must not fail the activation:
  // Humble's controller_manager aborts the whole node when hardware activation fails.
  const std::vector<double> measured(7, 0.1);
  TestSystem sys(measured);
  const auto info = make_info(seven_joints(), {{"port", "fake"}, {"auto_enable", "false"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  // Queue the refusal so the first move_js meets it; the retry then sees a normal ACK.
  sys.fake()->push_frame(
    litearm::proto::RSP_ERR,
    std::vector<uint8_t>{litearm::proto::CMD_MOVE_JS, 0x02});

  EXPECT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_FALSE(sys.fake()->stamps_of(litearm::proto::CMD_MOVE_JS).empty());
  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, ZeroVelocityFrameCarriesTheMeasuredPosition)
{
  // Firmware gate (2026-09-24): a MOVE_JS frame with dq all zero and a target more than
  // JS_ZERO_DQ_EPS (5 mrad) from the measurement is rejected whole with ERR{0x03,0x02}.
  // Under dq=0 the firmware freezes its own reference and ignores the position field, so
  // the frame must carry the measurement instead of the unreachable target.
  const std::vector<double> measured{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  TestSystem sys(measured);
  const auto info = make_info(seven_joints(), {{"port", "fake"}, {"auto_enable", "false"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  auto command = sys.export_command_interfaces();
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  for (std::size_t i = 0; i < measured.size(); ++i)
  {
    command[2u * i].set_value(measured[i] + 1.0);  // far beyond the 5 mrad gate
  }
  ASSERT_EQ(sys.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);

  const auto frame = last_move_js(sys, 2u * measured.size());
  ASSERT_EQ(frame.size(), 2u * measured.size());
  for (std::size_t i = 0; i < measured.size(); ++i)
  {
    EXPECT_NEAR(frame[i], measured[i], 1e-6) << "q_ref axis " << i;
    EXPECT_DOUBLE_EQ(frame[measured.size() + i], 0.0) << "dq_ref axis " << i;
  }

  ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(LitearmSystemLifecycle, ResetFaultsOnConfigureIsOptIn)
{
  const std::vector<double> measured(7, 0.1);
  const auto time = rclcpp::Time(0);
  const auto period = rclcpp::Duration::from_seconds(0.01);

  // Off (the default): configure must not touch the firmware's fault latch.
  {
    TestSystem sys(measured);
    ASSERT_EQ(sys.on_init(make_info(seven_joints(), {{"port", "fake"}})),
              CallbackReturn::SUCCESS);
    ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_EQ(sys.fake()->stamps_of(litearm::proto::CMD_RESET).size(), 0u);
    ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  // On: one CMD_RESET, sent while configuring and before anything is enabled.
  {
    TestSystem sys(measured);
    const auto info = make_info(
      seven_joints(), {{"port", "fake"}, {"reset_faults_on_configure", "true"},
                       {"auto_enable", "false"}});
    ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
    ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_EQ(sys.fake()->stamps_of(litearm::proto::CMD_RESET).size(), 1u);
    EXPECT_EQ(sys.fake()->stamps_of(litearm::proto::CMD_ENABLE).size(), 0u);
    ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(sys.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(sys.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(sys.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(LitearmSystemLifecycle, RefusesToActivateOnANonFiniteStatusFrame)
{
  // A faulted axis reports NaN. Before the guard existed, this NaN was latched as the
  // command reference and the firmware rejected the first frame (ERR 03,02), which on a
  // real arm ended in controller_manager aborting with an unrelated-looking message.
  std::vector<double> measured{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  measured[2] = std::numeric_limits<double>::quiet_NaN();
  TestSystem sys(measured);
  const auto info = make_info(seven_joints(), {{"port", "fake"}, {"auto_enable", "false"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  // The refusal has to come before anything is streamed: no MOVE_JS frame may exist.
  EXPECT_TRUE(last_move_js(sys, 2u * 7u).empty());
}

TEST(LitearmSystemLifecycle, ConfigureFailsWhenAxisCountDoesNotMatchUrdf)
{
  const std::vector<double> measured{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  TestSystem sys(measured);  // fake firmware reports 7 axes
  // URDF declares only six joints named joint1..joint6 but includes joint7 so the axis
  // numbers stay in range while the count is wrong.
  const std::vector<std::string> names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const auto info = make_info(names, {{"port", "fake"}});

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

}  // namespace
}  // namespace litearm_ros2_control

#include "rubi_control_core/gazebo_legacy_policy_adapter.hpp"
#include "rubi_control_core/gazebo_legacy_policy_runner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace rc = rubi_control_core;

namespace {

constexpr double kTolerance = 1.0e-5;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void expect_near(double actual, double expected, const std::string& message,
                 double tolerance = kTolerance) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

template <typename Container>
bool exact_zero(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return value == 0; });
}

template <typename Container>
bool all_finite(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return std::isfinite(value); });
}

class FakePolicy final : public rc::GazeboLegacyPolicyInterface {
 public:
  rc::GazeboLegacyNetworkOutput infer(
      const rc::GazeboLegacyHistory& input) override {
    ++calls;
    last_input = input;
    return output;
  }

  int calls{0};
  rc::GazeboLegacyHistory last_input{};
  rc::GazeboLegacyNetworkOutput output{};
};

rc::RobotState nominal_state() {
  rc::RobotState state;
  state.joint_position = rc::GazeboLegacyContract{}.default_pose;
  return state;
}

rc::UserCommand mode_command(rc::ControllerMode mode, std::uint64_t sequence) {
  rc::UserCommand command;
  command.requested_mode = mode;
  command.mode_sequence = sequence;
  return command;
}

void finish_walk_ready(rc::GazeboLegacyPolicyAdapter& controller,
                       const rc::RobotState& state,
                       rc::UserCommand& command) {
  command = mode_command(rc::ControllerMode::kWalkReady, 1);
  for (int i = 0; i < 252; ++i) {
    controller.update(state, command);
  }
  expect(controller.walk_ready_complete(), "walk-ready did not complete");
}

rc::ControllerOutput enter_policy_and_wait_for_inference(
    rc::GazeboLegacyPolicyAdapter& controller, const rc::RobotState& state,
    rc::UserCommand& command) {
  command.requested_mode = rc::ControllerMode::kPolicyOn;
  command.mode_sequence = 2;
  for (int i = 0; i < 5; ++i) {
    auto output = controller.update(state, command);
    if (output.inference_ran) {
      return output;
    }
  }
  throw std::runtime_error("policy inference did not run within decimation window");
}

void test_onnx_contract_320_to_7() {
  rc::GazeboLegacyPolicyRunner runner(RUBI_GAZEBO_LEGACY_MODEL);
  rc::GazeboLegacyHistory input{};
  const auto output = runner.infer(input);
  expect(input.size() == 320, "legacy input is not 320-D");
  expect(output.size() == 7, "legacy output is not 7-D");
  expect(all_finite(output), "legacy zero inference is nonfinite");
}

void test_deterministic_finite_inference() {
  rc::GazeboLegacyPolicyRunner runner(RUBI_GAZEBO_LEGACY_MODEL);
  rc::GazeboLegacyHistory input{};
  for (std::size_t i = 0; i < input.size(); ++i) {
    input[i] = static_cast<float>((static_cast<int>(i % 19) - 9) * 0.03125);
  }
  const auto first = runner.infer(input);
  for (int repeat = 0; repeat < 10; ++repeat) {
    expect(runner.infer(input) == first,
           "legacy synthetic inference is not deterministic");
  }
  expect(all_finite(first), "legacy synthetic inference is nonfinite");
}

void test_observation_32d_offsets_scales() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboLegacyPolicyAdapter controller(fake);
  auto state = nominal_state();
  state.base_angular_velocity = {4.0, 8.0, 12.0};
  for (std::size_t i = 0; i < 6; ++i) {
    state.joint_position[i] = static_cast<double>(i + 1);
    state.joint_velocity[i] = static_cast<double>((i + 1) * 10);
  }
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  command.linear_x = 2.0;
  command.linear_y = 0.75;
  command.angular_z = 1.5;
  command.command_sequence = 1;
  enter_policy_and_wait_for_inference(controller, state, command);
  const auto& obs = controller.last_observation();
  expect_near(obs[0], 1.0, "angular velocity x scale");
  expect_near(obs[2], 3.0, "angular velocity z scale");
  expect_near(obs[3], 0.0, "gravity x");
  expect_near(obs[5], -1.0, "gravity z");
  expect_near(obs[6], 2.0, "command x");
  expect_near(obs[8], 1.5, "command z");
  expect_near(obs[9], 1.0, "absolute joint position first");
  expect_near(obs[14], 6.0, "absolute joint position last");
  expect_near(obs[15], 1.0, "joint velocity first scale");
  expect_near(obs[20], 6.0, "joint velocity last scale");
  expect_near(obs[31], 0.4, "initial cycle period");
}

void test_history_oldest_to_newest() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboLegacyPolicyAdapter controller(fake);
  auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  state.joint_position[0] = 1.0;
  enter_policy_and_wait_for_inference(controller, state, command);
  state.joint_position[0] = 2.0;
  for (int i = 0; i < 5; ++i) {
    controller.update(state, command);
  }
  expect_near(fake->last_input[8 * 32 + 9], 1.0,
              "older legacy frame placement");
  expect_near(fake->last_input[9 * 32 + 9], 2.0,
              "newest legacy frame placement");
  expect(exact_zero(std::array<float, 2>{fake->last_input[0],
                                         fake->last_input[7 * 32 + 9]}),
         "legacy history did not start at deterministic zero");
}

void test_phase_and_cycle_output_order() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output[6] = 5.0F;
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  enter_policy_and_wait_for_inference(controller, state, command);
  expect_near(controller.last_observation()[27], 0.0,
              "first phase was not computed before increment");
  expect_near(controller.last_observation()[29], 1.0,
              "first cosine phase mismatch");
  expect_near(controller.cycle_time(), 0.01, "cycle time increment");
  expect_near(controller.cycle_period(), 0.45, "cycle output scale/clamp");
  for (int i = 0; i < 5; ++i) {
    controller.update(state, command);
  }
  expect_near(controller.last_observation()[31], 0.45,
              "updated period was not observed on next inference");
}

void test_action_mapping_and_scale() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output = {1, 2, 3, 4, 5, 6, 0};
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  const auto output = enter_policy_and_wait_for_inference(controller, state, command);
  const rc::JointArray expected{
      1.0, 1.872665, 2.75467, 4.0, 3.372665, 7.25467};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    expect_near(output.target_position[i], expected[i],
                "legacy action mapping/scale", 2.0e-5);
    expect(std::abs(output.effort[i]) <= 90.0,
           "legacy safety torque clamp exceeded");
  }
}

void test_policy_decimation_five() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  enter_policy_and_wait_for_inference(controller, state, command);
  const auto start = controller.inference_count();
  for (int i = 0; i < 10; ++i) {
    controller.update(state, command);
  }
  expect(controller.inference_count() - start == 2,
         "ten physics ticks did not run two legacy inferences");
}

void test_walk_ready_source_contract() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboLegacyPolicyAdapter controller(fake);
  rc::RobotState state;
  auto command = mode_command(rc::ControllerMode::kWalkReady, 1);
  auto output = controller.update(state, command);
  expect(exact_zero(output.target_position),
         "legacy walk-ready did not start at zero target");
  for (int i = 1; i < 251; ++i) {
    output = controller.update(state, command);
  }
  const auto expected = rc::GazeboLegacyContract{}.walk_ready_pose;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    expect_near(output.target_position[i], expected[i],
                "legacy 0.5 s walk-ready target");
  }
  controller.update(state, command);
  expect(controller.walk_ready_complete(),
         "legacy walk-ready completion flag was not set");
  expect(controller.mode() == rc::ControllerMode::kWalkReady,
         "ROS 2 explicit policy-enable boundary was not retained");
}

void test_torque_off_same_tick_zero() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output[0] = 1.0F;
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  auto output = enter_policy_and_wait_for_inference(controller, state, command);
  expect(!exact_zero(output.effort), "legacy policy effort stayed zero");
  output = controller.update(
      state, mode_command(rc::ControllerMode::kTorqueOff, 3));
  expect(exact_zero(output.effort), "legacy torque-off was not same-tick zero");
}

void test_emergency_stop_same_tick_zero() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output[0] = 1.0F;
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  auto output = enter_policy_and_wait_for_inference(controller, state, command);
  expect(!exact_zero(output.effort), "legacy policy effort stayed zero");
  command.emergency_stop = true;
  output = controller.update(state, command);
  expect(exact_zero(output.effort), "legacy E-stop was not same-tick zero");
  expect(output.mode == rc::ControllerMode::kEmergencyStop,
         "legacy E-stop did not latch");
}

void test_nan_state_same_tick_zero() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboLegacyPolicyAdapter controller(fake);
  auto state = nominal_state();
  state.joint_velocity[4] = std::numeric_limits<double>::quiet_NaN();
  const auto output = controller.update(state, rc::UserCommand{});
  expect(output.faulted && output.safe_zero,
         "legacy NaN state did not fault safe-zero");
  expect(exact_zero(output.effort), "legacy NaN state effort was not zero");
}

void test_reset_clears_history_action_phase() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output[0] = 1.0F;
  rc::GazeboLegacyPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  enter_policy_and_wait_for_inference(controller, state, command);
  expect(!exact_zero(controller.history()), "legacy history stayed zero");
  expect(!exact_zero(controller.held_action()), "legacy action stayed zero");
  expect(controller.cycle_time() > 0.0, "legacy phase did not advance");
  command.reset = true;
  command.reset_sequence = 1;
  const auto output = controller.update(state, command);
  expect(exact_zero(controller.history()), "legacy reset did not clear history");
  expect(exact_zero(controller.held_action()), "legacy reset did not clear action");
  expect_near(controller.cycle_time(), 0.0, "legacy reset did not clear phase");
  expect_near(controller.cycle_period(), 0.4,
              "legacy reset did not restore cycle period");
  expect(controller.mode() == rc::ControllerMode::kTorqueOff,
         "legacy reset did not restore torque-off");
  expect(exact_zero(output.effort), "legacy reset effort was not zero");
}

void test_wrong_canonical_policy_fail_fast() {
  bool rejected = false;
  try {
    rc::GazeboLegacyPolicyRunner runner(RUBI_CANONICAL_POLICY_MODEL);
  } catch (const std::exception&) {
    rejected = true;
  }
  expect(rejected, "36->6 canonical policy was accepted as legacy 320->7");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "usage: rubi_gazebo_legacy_core_test --case NAME\n";
    return 64;
  }
  const std::map<std::string, std::function<void()>> tests{
      {"onnx_contract_320_to_7", test_onnx_contract_320_to_7},
      {"deterministic_finite_inference", test_deterministic_finite_inference},
      {"observation_32d_offsets_scales", test_observation_32d_offsets_scales},
      {"history_oldest_to_newest", test_history_oldest_to_newest},
      {"phase_and_cycle_output_order", test_phase_and_cycle_output_order},
      {"action_mapping_and_scale", test_action_mapping_and_scale},
      {"policy_decimation_five", test_policy_decimation_five},
      {"walk_ready_source_contract", test_walk_ready_source_contract},
      {"torque_off_same_tick_zero", test_torque_off_same_tick_zero},
      {"emergency_stop_same_tick_zero", test_emergency_stop_same_tick_zero},
      {"nan_state_same_tick_zero", test_nan_state_same_tick_zero},
      {"reset_clears_history_action_phase", test_reset_clears_history_action_phase},
      {"wrong_canonical_policy_fail_fast", test_wrong_canonical_policy_fail_fast},
  };
  const auto found = tests.find(argv[2]);
  if (found == tests.end()) {
    std::cerr << "unknown test case: " << argv[2] << '\n';
    return 64;
  }
  try {
    found->second();
    std::cout << "PASS " << found->first << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << found->first << ": " << error.what() << '\n';
    return 1;
  }
}

#include "rubi_control_core/controller.hpp"
#include "rubi_control_core/policy_contract.hpp"
#include "rubi_control_core/policy_runner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

class FakePolicy final : public rc::PolicyInterface {
 public:
  rc::Latent encode(const rc::History& history) override {
    ++encode_calls;
    last_history = history;
    return latent;
  }

  rc::Action act(const rc::PolicyInput& input) override {
    ++act_calls;
    last_policy_input = input;
    if (increment_action) {
      rc::Action generated{};
      generated.fill(static_cast<float>(act_calls));
      return generated;
    }
    return action;
  }

  int encode_calls{0};
  int act_calls{0};
  bool increment_action{false};
  rc::History last_history{};
  rc::PolicyInput last_policy_input{};
  rc::Latent latent{};
  rc::Action action{};
};

rc::RobotState nominal_state() {
  rc::RobotState state;
  state.joint_position = rc::PolicyContract{}.default_pose;
  return state;
}

rc::UserCommand mode_command(rc::ControllerMode mode, std::uint64_t sequence = 1) {
  rc::UserCommand command;
  command.requested_mode = mode;
  command.mode_sequence = sequence;
  return command;
}

std::shared_ptr<FakePolicy> fake_with_action(float value = 0.0F) {
  auto fake = std::make_shared<FakePolicy>();
  fake->action.fill(value);
  return fake;
}

void test_canonical_onnx_contract() {
  rc::PolicyRunner runner(RUBI_ENCODER_MODEL, RUBI_POLICY_MODEL);
  rc::History history{};
  const auto latent = runner.encode(history);
  rc::PolicyInput input{};
  const auto action = runner.act(input);
  expect(latent.size() == 3, "encoder output is not 3-D");
  expect(action.size() == 6, "policy output is not 6-D");
}

void test_deterministic_zero_synthetic_inference() {
  rc::PolicyRunner runner(RUBI_ENCODER_MODEL, RUBI_POLICY_MODEL);
  rc::History zero_history{};
  const auto zero_a = runner.encode(zero_history);
  const auto zero_b = runner.encode(zero_history);
  expect(zero_a == zero_b, "zero encoder inference is not deterministic");

  rc::History synthetic_history{};
  for (std::size_t i = 0; i < synthetic_history.size(); ++i) {
    synthetic_history[i] = static_cast<float>((static_cast<int>(i % 17) - 8) * 0.03125);
  }
  const auto latent_a = runner.encode(synthetic_history);
  const auto latent_b = runner.encode(synthetic_history);
  expect(latent_a == latent_b, "synthetic encoder inference is not deterministic");

  rc::PolicyInput synthetic_input{};
  for (std::size_t i = 0; i < synthetic_input.size(); ++i) {
    synthetic_input[i] = static_cast<float>((static_cast<int>(i % 11) - 5) * 0.0625);
  }
  const auto action_a = runner.act(synthetic_input);
  const auto action_b = runner.act(synthetic_input);
  expect(action_a == action_b, "synthetic policy inference is not deterministic");
  for (float value : action_a) {
    expect(std::isfinite(value), "policy produced nonfinite output");
  }
}

void test_observation_30d_offsets_and_scales() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  auto state = nominal_state();
  state.base_angular_velocity = {4.0, 8.0, 12.0};
  for (std::size_t i = 0; i < 6; ++i) {
    state.joint_position[i] += static_cast<double>(i + 1);
    state.joint_velocity[i] = static_cast<double>((i + 1) * 10);
  }
  controller.update(state, mode_command(rc::ControllerMode::kPolicyOn));
  const auto& obs = controller.last_observation();
  expect(obs.size() == 30, "observation is not 30-D");
  expect_near(obs[0], 1.0, "angular velocity x offset/scale");
  expect_near(obs[2], 3.0, "angular velocity z offset/scale");
  expect_near(obs[3], 0.0, "gravity x offset");
  expect_near(obs[5], -1.0, "gravity z offset");
  expect_near(obs[6], 1.0, "joint position first offset");
  expect_near(obs[11], 6.0, "joint position last offset");
  expect_near(obs[12], 1.0, "joint velocity first scale");
  expect_near(obs[17], 6.0, "joint velocity last scale");
  expect(exact_zero(std::array<float, 6>{obs[18], obs[19], obs[20], obs[21], obs[22], obs[23]}),
         "previous action offsets are not zero initially");
  expect_near(obs[26], 2.5, "gait frequency offset");
  expect_near(obs[29], 0.0, "gait swing height offset");
}

void test_identity_quaternion_projected_gravity() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  controller.update(nominal_state(), mode_command(rc::ControllerMode::kPolicyOn));
  const auto& obs = controller.last_observation();
  expect_near(obs[3], 0.0, "identity gravity x");
  expect_near(obs[4], 0.0, "identity gravity y");
  expect_near(obs[5], -1.0, "identity gravity z");
}

void test_rotated_quaternion_projected_gravity() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  auto state = nominal_state();
  const double half = std::sqrt(0.5);
  state.base_orientation_xyzw = {0.0, half, 0.0, half};
  controller.update(state, mode_command(rc::ControllerMode::kPolicyOn));
  const auto& obs = controller.last_observation();
  expect_near(obs[3], 1.0, "rotated gravity x");
  expect_near(obs[4], 0.0, "rotated gravity y");
  expect_near(obs[5], 0.0, "rotated gravity z");
}

void test_joint_position_relative_default() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  auto state = nominal_state();
  state.joint_position[1] += 0.25;
  state.joint_position[2] -= 0.5;
  controller.update(state, mode_command(rc::ControllerMode::kPolicyOn));
  expect_near(controller.last_observation()[7], 0.25, "relative hip position");
  expect_near(controller.last_observation()[8], -0.5, "relative knee position");
}

void test_history_oldest_to_newest_shift() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  auto state = nominal_state();
  state.joint_position[0] += 1.0;
  auto command = mode_command(rc::ControllerMode::kPolicyOn);
  controller.update(state, command);
  state.joint_position[0] += 1.0;
  for (int i = 0; i < 5; ++i) {
    controller.update(state, command);
  }
  expect_near(fake->last_history[8 * 30 + 6], 1.0, "older observation placement");
  expect_near(fake->last_history[9 * 30 + 6], 2.0, "newest observation placement");
  expect(exact_zero(std::array<float, 2>{fake->last_history[0], fake->last_history[7 * 30 + 6]}),
         "initial history frames were not zero");
}

void test_encoder_input_dimension_300() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  controller.update(nominal_state(), mode_command(rc::ControllerMode::kPolicyOn));
  expect(fake->last_history.size() == 300, "encoder input is not 300-D");
}

void test_policy_concatenation_dimension_36() {
  auto fake = fake_with_action();
  fake->latent = {1.0F, 2.0F, 3.0F};
  rc::Controller controller(fake);
  auto command = mode_command(rc::ControllerMode::kPolicyOn);
  command.linear_x = 0.5;
  command.linear_y = 0.5;
  command.angular_z = 1.0;
  command.command_sequence = 1;
  controller.update(nominal_state(), command);
  expect(fake->last_policy_input.size() == 36, "policy input is not 36-D");
  expect_near(fake->last_policy_input[0], 1.0, "latent concat start");
  expect_near(fake->last_policy_input[2], 3.0, "latent concat end");
  expect_near(fake->last_policy_input[3 + 5], -1.0, "observation concat placement");
  expect_near(fake->last_policy_input[33], 2.0, "linear x command scale");
  expect_near(fake->last_policy_input[34], 1.5, "linear y command scale");
  expect_near(fake->last_policy_input[35], 0.375, "yaw command scale");
}

void test_action_dimension_6() {
  auto fake = fake_with_action();
  fake->action = {1, 2, 3, 4, 5, 6};
  rc::Controller controller(fake);
  const auto output = controller.update(
      nominal_state(), mode_command(rc::ControllerMode::kPolicyOn));
  expect(output.action.size() == 6, "action is not 6-D");
  expect(output.action == fake->action, "policy action was not propagated");
}

void test_ten_physics_ticks_two_inferences() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  auto command = mode_command(rc::ControllerMode::kPolicyOn);
  for (int i = 0; i < 10; ++i) {
    controller.update(nominal_state(), command);
  }
  expect(fake->act_calls == 2, "10 physics ticks did not run exactly two inferences");
  expect(controller.inference_count() == 2, "inference counter mismatch");
}

void test_action_hold_between_inferences() {
  auto fake = fake_with_action();
  fake->increment_action = true;
  rc::Controller controller(fake);
  auto command = mode_command(rc::ControllerMode::kPolicyOn);
  auto output = controller.update(nominal_state(), command);
  expect_near(output.action[0], 1.0, "first inferred action");
  for (int i = 0; i < 4; ++i) {
    output = controller.update(nominal_state(), command);
    expect_near(output.action[0], 1.0, "action changed between inference ticks");
    expect(!output.inference_ran, "inference unexpectedly ran between decimated ticks");
  }
  output = controller.update(nominal_state(), command);
  expect_near(output.action[0], 2.0, "second inferred action was not applied");
}

void test_walk_ready_duration_0_5_sec() {
  auto fake = fake_with_action();
  rc::Controller controller(fake);
  rc::RobotState state;
  auto command = mode_command(rc::ControllerMode::kWalkReady);
  rc::ControllerOutput output;
  for (int i = 0; i < 250; ++i) {
    output = controller.update(state, command);
  }
  const auto expected = rc::PolicyContract{}.default_pose;
  for (std::size_t i = 0; i < 6; ++i) {
    expect_near(output.target_position[i], expected[i], "walk-ready final target");
  }
}

void test_torque_off_same_tick_exact_zero() {
  auto fake = fake_with_action(1.0F);
  rc::Controller controller(fake);
  auto state = nominal_state();
  auto output = controller.update(state, mode_command(rc::ControllerMode::kPolicyOn, 1));
  expect(!exact_zero(output.effort), "policy did not produce prior nonzero effort");
  output = controller.update(state, mode_command(rc::ControllerMode::kTorqueOff, 2));
  expect(exact_zero(output.effort), "torque-off did not overwrite all effort with exact zero");
}

void test_emergency_stop_same_tick_exact_zero() {
  auto fake = fake_with_action(1.0F);
  rc::Controller controller(fake);
  auto state = nominal_state();
  auto output = controller.update(state, mode_command(rc::ControllerMode::kPolicyOn, 1));
  expect(!exact_zero(output.effort), "policy did not produce prior nonzero effort");
  auto emergency = mode_command(rc::ControllerMode::kPolicyOn, 1);
  emergency.emergency_stop = true;
  output = controller.update(state, emergency);
  expect(exact_zero(output.effort), "E-stop did not overwrite all effort with exact zero");
  expect(output.mode == rc::ControllerMode::kEmergencyStop, "E-stop was not latched");
}

void test_torque_clamp_plus_minus_90() {
  auto fake = fake_with_action(10.0F);
  rc::Controller controller(fake);
  auto output = controller.update(
      nominal_state(), mode_command(rc::ControllerMode::kPolicyOn));
  for (double effort : output.effort) {
    expect_near(effort, 90.0, "positive torque clamp");
  }
  fake->action.fill(-10.0F);
  rc::Controller negative_controller(fake);
  output = negative_controller.update(
      nominal_state(), mode_command(rc::ControllerMode::kPolicyOn));
  for (double effort : output.effort) {
    expect_near(effort, -90.0, "negative torque clamp");
  }
}

void test_nan_state_safe_zero() {
  auto fake = fake_with_action(1.0F);
  rc::Controller controller(fake);
  auto state = nominal_state();
  state.joint_position[2] = std::numeric_limits<double>::quiet_NaN();
  const auto output = controller.update(
      state, mode_command(rc::ControllerMode::kPolicyOn));
  expect(output.safe_zero && output.faulted, "NaN state did not fault safe-zero");
  expect(exact_zero(output.effort), "NaN state output was not exact zero");
}

void test_reset_clears_history_action_phase() {
  auto fake = fake_with_action(1.0F);
  rc::Controller controller(fake);
  auto command = mode_command(rc::ControllerMode::kPolicyOn);
  controller.update(nominal_state(), command);
  expect(!exact_zero(controller.history()), "history did not receive observation");
  expect(!exact_zero(controller.held_action()), "action did not become nonzero");
  expect(controller.gait_phase() > 0.0, "gait phase did not advance");
  command.reset = true;
  command.reset_sequence = 1;
  const auto output = controller.update(nominal_state(), command);
  expect(exact_zero(controller.history()), "reset did not clear history");
  expect(exact_zero(controller.held_action()), "reset did not clear action");
  expect_near(controller.gait_phase(), 0.0, "reset did not clear gait phase");
  expect(controller.mode() == rc::ControllerMode::kTorqueOff,
         "reset did not restore torque-off mode");
  expect(exact_zero(output.effort), "reset output was not exact zero");
}

void test_wrong_joint_manifest_fail_fast() {
  std::vector<std::string> names(rc::canonical_joint_order().begin(),
                                 rc::canonical_joint_order().end());
  std::swap(names[0], names[1]);
  bool rejected = false;
  try {
    rc::validate_joint_order(names);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  expect(rejected, "wrong joint order was accepted");
}

void test_missing_corrupt_policy_fail_fast() {
  bool missing_rejected = false;
  try {
    rc::PolicyRunner runner("/definitely/missing/rubi_encoder.onnx", RUBI_POLICY_MODEL);
  } catch (const std::exception&) {
    missing_rejected = true;
  }
  expect(missing_rejected, "missing encoder was accepted");

  bool corrupt_rejected = false;
  try {
    rc::PolicyRunner runner(RUBI_ENCODER_MODEL, "/dev/null");
  } catch (const std::exception&) {
    corrupt_rejected = true;
  }
  expect(corrupt_rejected, "corrupt policy was accepted");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "usage: rubi_control_core_test --case NAME\n";
    return 64;
  }

  const std::map<std::string, std::function<void()>> tests{
      {"canonical_onnx_contract", test_canonical_onnx_contract},
      {"deterministic_zero_synthetic_inference", test_deterministic_zero_synthetic_inference},
      {"observation_30d_offsets_and_scales", test_observation_30d_offsets_and_scales},
      {"identity_quaternion_projected_gravity", test_identity_quaternion_projected_gravity},
      {"rotated_quaternion_projected_gravity", test_rotated_quaternion_projected_gravity},
      {"joint_position_relative_default", test_joint_position_relative_default},
      {"history_oldest_to_newest_shift", test_history_oldest_to_newest_shift},
      {"encoder_input_dimension_300", test_encoder_input_dimension_300},
      {"policy_concatenation_dimension_36", test_policy_concatenation_dimension_36},
      {"action_dimension_6", test_action_dimension_6},
      {"ten_physics_ticks_two_inferences", test_ten_physics_ticks_two_inferences},
      {"action_hold_between_inferences", test_action_hold_between_inferences},
      {"walk_ready_duration_0_5_sec", test_walk_ready_duration_0_5_sec},
      {"torque_off_same_tick_exact_zero", test_torque_off_same_tick_exact_zero},
      {"emergency_stop_same_tick_exact_zero", test_emergency_stop_same_tick_exact_zero},
      {"torque_clamp_plus_minus_90", test_torque_clamp_plus_minus_90},
      {"nan_state_safe_zero", test_nan_state_safe_zero},
      {"reset_clears_history_action_phase", test_reset_clears_history_action_phase},
      {"wrong_joint_manifest_fail_fast", test_wrong_joint_manifest_fail_fast},
      {"missing_corrupt_policy_fail_fast", test_missing_corrupt_policy_fail_fast},
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

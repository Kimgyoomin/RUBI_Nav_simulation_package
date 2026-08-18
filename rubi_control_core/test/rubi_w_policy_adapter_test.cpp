#include "rubi_control_core/policy_contract.hpp"
#include "rubi_control_core/rubi_w_policy_adapter.hpp"
#include "rubi_control_core/rubi_w_policy_runner.hpp"

#include <algorithm>
#include <cmath>
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

template <typename Container>
bool all_finite(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return std::isfinite(value); });
}

class FakeRubiWPolicy final : public rc::RubiWPolicyInterface {
 public:
  rc::RubiWLatent encode(const rc::RubiWHistory& history) override {
    ++encode_calls;
    last_history = history;
    return latent;
  }

  rc::RubiWAction act(const rc::RubiWPolicyInput& input) override {
    ++act_calls;
    last_policy_input = input;
    return action;
  }

  int encode_calls{0};
  int act_calls{0};
  rc::RubiWHistory last_history{};
  rc::RubiWPolicyInput last_policy_input{};
  rc::RubiWLatent latent{};
  rc::RubiWAction action{};
};

rc::RubiWRobotState nominal_state() {
  rc::RubiWRobotState state;
  state.joint_position = rc::RubiWContract{}.default_pose;
  return state;
}

rc::RubiWUserCommand mode_command(rc::ControllerMode mode,
                                  std::uint64_t sequence) {
  rc::RubiWUserCommand command;
  command.requested_mode = mode;
  command.mode_sequence = sequence;
  return command;
}

std::shared_ptr<FakeRubiWPolicy> fake_with_action(float value = 0.0F) {
  auto fake = std::make_shared<FakeRubiWPolicy>();
  fake->action.fill(value);
  return fake;
}

void finish_walk_ready(rc::RubiWPolicyAdapter& adapter,
                       const rc::RubiWRobotState& state) {
  const auto command = mode_command(rc::ControllerMode::kWalkReady, 1);
  for (int i = 0; i < 252; ++i) {
    adapter.update(state, command);
  }
  expect(adapter.walk_ready_complete(), "walk-ready did not complete at source timing");
}

rc::RubiWControllerOutput enter_policy(
    rc::RubiWPolicyAdapter& adapter, const rc::RubiWRobotState& state,
    rc::RubiWUserCommand command = mode_command(rc::ControllerMode::kPolicyOn, 2)) {
  for (int i = 0; i < 6; ++i) {
    const auto output = adapter.update(state, command);
    if (output.inference_ran || output.faulted) {
      return output;
    }
  }
  throw std::runtime_error("RUBI_W policy inference did not run within one decimation");
}

void test_onnx_contract() {
  rc::RubiWPolicyRunner runner(RUBI_W_ENCODER_MODEL, RUBI_W_POLICY_MODEL);
  rc::RubiWHistory history{};
  rc::RubiWPolicyInput input{};
  expect(runner.encode(history).size() == 3, "RUBI_W encoder output is not 3-D");
  expect(runner.act(input).size() == 8, "RUBI_W policy output is not 8-D");
}

void test_deterministic_finite_inference() {
  rc::RubiWPolicyRunner runner(RUBI_W_ENCODER_MODEL, RUBI_W_POLICY_MODEL);
  rc::RubiWHistory history{};
  for (std::size_t i = 0; i < history.size(); ++i) {
    history[i] = static_cast<float>((static_cast<int>(i % 17) - 8) * 0.03125);
  }
  const auto latent_a = runner.encode(history);
  const auto latent_b = runner.encode(history);
  expect(latent_a == latent_b, "RUBI_W encoder is not deterministic");
  expect(all_finite(latent_a), "RUBI_W encoder output is nonfinite");

  rc::RubiWPolicyInput input{};
  for (std::size_t i = 0; i < input.size(); ++i) {
    input[i] = static_cast<float>((static_cast<int>(i % 11) - 5) * 0.0625);
  }
  const auto action_a = runner.act(input);
  const auto action_b = runner.act(input);
  expect(action_a == action_b, "RUBI_W policy is not deterministic");
  expect(all_finite(action_a), "RUBI_W policy output is nonfinite");
}

void test_observation_offsets_scales() {
  auto fake = fake_with_action();
  rc::RubiWPolicyAdapter adapter(fake);
  auto state = nominal_state();
  state.base_angular_velocity = {4.0, 8.0, 12.0};
  for (std::size_t i = 0; i < rc::kRubiWJointCount; ++i) {
    state.joint_position[i] += static_cast<double>(i + 1);
    state.joint_velocity[i] = static_cast<double>((i + 1) * 10);
  }
  finish_walk_ready(adapter, state);
  enter_policy(adapter, state);
  const auto& obs = adapter.last_observation();
  expect(obs.size() == 34, "RUBI_W observation is not 34-D");
  expect_near(obs[0], 1.0, "angular velocity x scale");
  expect_near(obs[2], 3.0, "angular velocity z scale");
  expect_near(obs[3], 0.0, "projected gravity x");
  expect_near(obs[5], -1.0, "projected gravity z");
  expect_near(obs[6], 1.0, "left roll position offset");
  expect_near(obs[8], 3.0, "left knee position offset");
  expect_near(obs[9], 5.0, "right roll position subset mapping");
  expect_near(obs[11], 7.0, "right knee position subset mapping");
  expect_near(obs[12], 1.0, "first joint velocity scale");
  expect_near(obs[19], 8.0, "last joint velocity scale");
  expect(exact_zero(std::array<float, 8>{obs[20], obs[21], obs[22], obs[23],
                                         obs[24], obs[25], obs[26], obs[27]}),
         "initial previous-action block is not zero");
  expect_near(obs[30], 1.5, "gait frequency field");
  expect_near(obs[31], 0.5, "gait offset field");
  expect_near(obs[32], 0.5, "gait duration field");
  expect_near(obs[33], 0.0, "gait swing-height field");
}

void test_history_order() {
  auto fake = fake_with_action();
  rc::RubiWPolicyAdapter adapter(fake);
  auto state = nominal_state();
  finish_walk_ready(adapter, state);
  state.joint_position[0] += 1.0;
  auto command = mode_command(rc::ControllerMode::kPolicyOn, 2);
  enter_policy(adapter, state, command);
  state.joint_position[0] += 1.0;
  for (int i = 0; i < 6 && fake->encode_calls < 2; ++i) {
    adapter.update(state, command);
  }
  expect(fake->encode_calls == 2, "second RUBI_W history inference did not run");
  expect_near(fake->last_history[8 * 34 + 6], 1.0,
              "older RUBI_W observation placement");
  expect_near(fake->last_history[9 * 34 + 6], 2.0,
              "newest RUBI_W observation placement");
  expect_near(fake->last_history[7 * 34 + 6], 0.0,
              "zero-initialized RUBI_W history frame");
}

void test_policy_concatenation() {
  auto fake = fake_with_action();
  fake->latent = {1.0F, 2.0F, 3.0F};
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  auto command = mode_command(rc::ControllerMode::kPolicyOn, 2);
  command.linear_x = 0.5;
  command.linear_y = 0.5;
  command.angular_z = 1.0;
  command.command_sequence = 1;
  enter_policy(adapter, state, command);
  expect(fake->last_policy_input.size() == 40, "RUBI_W policy input is not 40-D");
  expect_near(fake->last_policy_input[0], 1.0, "latent concat start");
  expect_near(fake->last_policy_input[2], 3.0, "latent concat end");
  expect_near(fake->last_policy_input[3 + 5], -1.0,
              "observation concat placement");
  expect_near(fake->last_policy_input[37], 2.5, "linear x command contract");
  expect_near(fake->last_policy_input[38], 1.5, "linear y command contract");
  expect_near(fake->last_policy_input[39], 0.3925, "yaw command contract");
}

void test_action_mapping() {
  auto fake = fake_with_action();
  fake->action = {1, 2, 3, 4, 5, 6, 7, 8};
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  const auto output = enter_policy(adapter, state);
  expect(output.action == fake->action, "8-D RUBI_W action mapping changed");
}

void test_leg_wheel_split() {
  auto fake = fake_with_action();
  fake->action = {1, 2, 3, 4, 5, 6, 7, 8};
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  const auto output = enter_policy(adapter, state);
  const auto defaults = rc::RubiWContract{}.default_pose;
  expect_near(output.target_position[0], defaults[0] + 1.0, "LHR position target");
  expect_near(output.target_position[2], defaults[2] + 3.0, "LKN position target");
  expect_near(output.target_position[4], defaults[4] + 5.0, "RHR position target");
  expect_near(output.target_position[6], defaults[6] + 7.0, "RKN position target");
  expect_near(output.target_position[3], defaults[3], "LWH must not use position action");
  expect_near(output.target_position[7], defaults[7], "RWH must not use position action");
  expect_near(output.target_velocity[3], 2.0, "LWH velocity target");
  expect_near(output.target_velocity[7], 4.0, "RWH velocity target");
}

void test_action_scale() {
  const auto scale = rc::RubiWContract{}.action_scale;
  const rc::RubiWJointArray expected{1.0, 1.0, 1.0, 0.5,
                                     1.0, 1.0, 1.0, 0.5};
  expect(scale == expected, "RUBI_W action scale differs from active source");
}

void test_leg_wheel_gains() {
  const rc::RubiWContract contract;
  for (const auto index : {0U, 1U, 2U, 4U, 5U, 6U}) {
    expect_near(contract.kp[index], 40.0, "leg Kp");
    expect_near(contract.kd[index], 2.0, "leg Kd");
  }
  for (const auto index : {3U, 7U}) {
    expect_near(contract.kp[index], 0.0, "wheel Kp");
    expect_near(contract.kd[index], 4.8, "wheel Kd");
  }
}

void test_decimation() {
  auto fake = fake_with_action();
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  const auto command = mode_command(rc::ControllerMode::kPolicyOn, 2);
  for (int i = 0; i < 10; ++i) {
    adapter.update(state, command);
  }
  expect(fake->act_calls == 2, "10 RUBI_W physics ticks did not run two inferences");
  expect(adapter.inference_count() == 2, "RUBI_W inference counter mismatch");
}

void test_torque_off_estop_zero() {
  auto fake = fake_with_action(1.0F);
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  auto output = enter_policy(adapter, state);
  expect(!exact_zero(output.effort), "RUBI_W policy did not produce prior effort");
  output = adapter.update(state, mode_command(rc::ControllerMode::kTorqueOff, 3));
  expect(exact_zero(output.effort), "RUBI_W torque-off was not same-tick eight-zero");

  auto estop_fake = fake_with_action(1.0F);
  rc::RubiWPolicyAdapter estop_adapter(estop_fake);
  finish_walk_ready(estop_adapter, state);
  output = enter_policy(estop_adapter, state);
  rc::RubiWUserCommand estop = mode_command(rc::ControllerMode::kPolicyOn, 2);
  estop.emergency_stop = true;
  output = estop_adapter.update(state, estop);
  expect(exact_zero(output.effort), "RUBI_W E-stop was not same-tick eight-zero");
  expect(output.mode == rc::ControllerMode::kEmergencyStop,
         "RUBI_W E-stop did not latch");
}

void test_nonfinite_zero() {
  auto fake = fake_with_action(1.0F);
  fake->action[4] = std::numeric_limits<float>::quiet_NaN();
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  const auto output = enter_policy(adapter, state);
  expect(output.faulted && output.safe_zero,
         "nonfinite RUBI_W action did not fault safe-zero");
  expect(exact_zero(output.effort), "nonfinite RUBI_W action was not eight-zero");
}

void test_reset() {
  auto fake = fake_with_action(1.0F);
  rc::RubiWPolicyAdapter adapter(fake);
  const auto state = nominal_state();
  finish_walk_ready(adapter, state);
  enter_policy(adapter, state);
  expect(!exact_zero(adapter.history()), "RUBI_W history stayed zero before reset");
  expect(!exact_zero(adapter.held_action()), "RUBI_W action stayed zero before reset");
  expect(adapter.gait_index() > 0.0, "RUBI_W gait phase did not advance");
  auto reset = mode_command(rc::ControllerMode::kPolicyOn, 2);
  reset.reset = true;
  reset.reset_sequence = 1;
  const auto output = adapter.update(state, reset);
  expect(exact_zero(adapter.history()), "RUBI_W reset did not clear history");
  expect(exact_zero(adapter.held_action()), "RUBI_W reset did not clear action");
  expect_near(adapter.gait_index(), 0.0, "RUBI_W reset did not clear gait phase");
  expect(adapter.mode() == rc::ControllerMode::kTorqueOff,
         "RUBI_W reset did not restore torque-off");
  expect(exact_zero(output.effort), "RUBI_W reset output was not eight-zero");
}

void test_six_dof_manifest_fail_fast() {
  const std::vector<std::string> six_dof(rc::canonical_joint_order().begin(),
                                         rc::canonical_joint_order().end());
  bool rejected = false;
  try {
    rc::validate_rubi_w_controller_order(six_dof);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  expect(rejected, "six-DoF RUBI manifest was accepted by RUBI_W adapter");
}

void test_wrong_pair_fail_fast() {
  bool rejected = false;
  try {
    rc::RubiWPolicyRunner runner(RUBI_W_WRONG_ENCODER_MODEL,
                                 RUBI_W_WRONG_POLICY_MODEL);
  } catch (const std::exception&) {
    rejected = true;
  }
  expect(rejected, "unidentified 280/35/8 pair was accepted as RUBI_W policy");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "usage: rubi_w_control_core_test --case NAME\n";
    return 64;
  }

  const std::map<std::string, std::function<void()>> tests{
      {"onnx_contract_340_to_3_40_to_8", test_onnx_contract},
      {"deterministic_finite_inference", test_deterministic_finite_inference},
      {"observation_34d_offsets_scales", test_observation_offsets_scales},
      {"history_340_oldest_to_newest", test_history_order},
      {"policy_concatenation_40", test_policy_concatenation},
      {"action_mapping_8", test_action_mapping},
      {"leg_position_wheel_velocity_split", test_leg_wheel_split},
      {"action_scale_source_contract", test_action_scale},
      {"leg_wheel_gains_source_contract", test_leg_wheel_gains},
      {"policy_decimation_five", test_decimation},
      {"torque_off_estop_same_tick_eight_zero", test_torque_off_estop_zero},
      {"nonfinite_same_tick_eight_zero", test_nonfinite_zero},
      {"reset_clears_history_action_phase", test_reset},
      {"six_dof_manifest_fail_fast", test_six_dof_manifest_fail_fast},
      {"wrong_280_35_8_pair_fail_fast", test_wrong_pair_fail_fast},
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

#include "rubi_control_core/gazebo_terrain_policy_adapter.hpp"
#include "rubi_control_core/gazebo_terrain_policy_runner.hpp"

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

class FakePolicy final : public rc::GazeboTerrainPolicyInterface {
 public:
  rc::GazeboTerrainLatent encode(
      const rc::GazeboTerrainHistory& input) override {
    ++encode_calls;
    last_encoder_input = input;
    return latent;
  }

  rc::Action act(const rc::GazeboTerrainPolicyInput& input) override {
    ++act_calls;
    last_policy_input = input;
    return output;
  }

  int encode_calls{0};
  int act_calls{0};
  rc::GazeboTerrainHistory last_encoder_input{};
  rc::GazeboTerrainPolicyInput last_policy_input{};
  rc::GazeboTerrainLatent latent{};
  rc::Action output{};
};

rc::RobotState nominal_state() {
  rc::RobotState state;
  state.joint_position = rc::GazeboTerrainContract{}.default_pose;
  return state;
}

rc::UserCommand mode_command(rc::ControllerMode mode, std::uint64_t sequence) {
  rc::UserCommand command;
  command.requested_mode = mode;
  command.mode_sequence = sequence;
  return command;
}

void finish_walk_ready(rc::GazeboTerrainPolicyAdapter& controller,
                       const rc::RobotState& state,
                       rc::UserCommand& command) {
  command = mode_command(rc::ControllerMode::kWalkReady, 1);
  for (int i = 0; i < 252; ++i) {
    controller.update(state, command);
  }
  expect(controller.walk_ready_complete(), "terrain walk-ready did not complete");
}

rc::ControllerOutput enter_policy_and_wait_for_inference(
    rc::GazeboTerrainPolicyAdapter& controller, const rc::RobotState& state,
    rc::UserCommand& command) {
  command.requested_mode = rc::ControllerMode::kPolicyOn;
  command.mode_sequence = 2;
  for (int i = 0; i < 5; ++i) {
    auto output = controller.update(state, command);
    if (output.inference_ran) {
      return output;
    }
  }
  throw std::runtime_error(
      "terrain policy inference did not run within decimation window");
}

void test_onnx_contract_330_to_32_65_to_6() {
  rc::GazeboTerrainPolicyRunner runner(RUBI_TERRAIN_ENCODER_MODEL,
                                       RUBI_TERRAIN_POLICY_MODEL);
  rc::GazeboTerrainHistory history{};
  const auto latent = runner.encode(history);
  rc::GazeboTerrainPolicyInput input{};
  std::copy(latent.begin(), latent.end(), input.begin());
  const auto action = runner.act(input);
  expect(history.size() == 330, "terrain encoder input is not 330-D");
  expect(latent.size() == 32, "terrain latent is not 32-D");
  expect(input.size() == 65, "terrain policy input is not 65-D");
  expect(action.size() == 6, "terrain action is not 6-D");
  expect(all_finite(latent) && all_finite(action),
         "terrain zero inference is nonfinite");
}

void test_observation_actor_history_and_policy_order() {
  auto fake = std::make_shared<FakePolicy>();
  for (std::size_t i = 0; i < fake->latent.size(); ++i) {
    fake->latent[i] = static_cast<float>(100 + i);
  }
  rc::GazeboTerrainPolicyAdapter controller(fake);
  auto state = nominal_state();
  state.base_angular_velocity = {4.0, 8.0, 12.0};
  for (std::size_t i = 0; i < rc::kJointCount; ++i) {
    state.joint_position[i] += static_cast<double>(i + 1);
    state.joint_velocity[i] = static_cast<double>((i + 1) * 10);
  }
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  command.linear_x = 1.0;
  command.linear_y = 0.75;
  command.angular_z = 1.5;
  command.command_sequence = 1;
  enter_policy_and_wait_for_inference(controller, state, command);

  const auto& obs = controller.last_observation();
  const auto& actor = controller.last_actor_observation();
  expect_near(obs[0], 1.0, "terrain angular velocity x scale");
  expect_near(obs[2], 3.0, "terrain angular velocity z scale");
  expect_near(obs[3], 0.0, "terrain gravity x");
  expect_near(obs[5], -1.0, "terrain gravity z");
  expect_near(obs[6], 1.0, "terrain relative joint position first");
  expect_near(obs[11], 6.0, "terrain relative joint position last");
  expect_near(obs[12], 1.0, "terrain joint velocity first scale");
  expect_near(obs[17], 6.0, "terrain joint velocity last scale");
  expect_near(obs[24], std::sin(0.04 * M_PI), "terrain first clock sine");
  expect_near(obs[25], std::cos(0.04 * M_PI), "terrain first clock cosine");
  expect_near(obs[26], 2.0, "terrain gait frequency");
  expect_near(actor[30], 2.0, "terrain scaled command x");
  expect_near(actor[31], 1.5, "terrain scaled command y");
  expect_near(actor[32], 0.375, "terrain scaled command yaw");

  const std::size_t latest = 9 * rc::kGazeboTerrainActorObservationDim;
  expect_near(fake->last_encoder_input[latest + 30], 2.0,
              "terrain command missing from newest history frame");
  expect(exact_zero(std::array<float, 2>{fake->last_encoder_input[0],
                                         fake->last_encoder_input[8 * 33]}),
         "terrain history did not start from deterministic zero");
  expect_near(fake->last_policy_input[0], 100.0,
              "terrain policy latent placement");
  expect_near(fake->last_policy_input[31], 131.0,
              "terrain policy latent tail placement");
  expect_near(fake->last_policy_input[32 + 30], 2.0,
              "terrain current actor observation placement");
}

void test_history_oldest_to_newest() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboTerrainPolicyAdapter controller(fake);
  auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  state.joint_position[0] += 1.0;
  enter_policy_and_wait_for_inference(controller, state, command);
  state.joint_position[0] += 1.0;
  for (int i = 0; i < 5; ++i) {
    controller.update(state, command);
  }
  expect_near(fake->last_encoder_input[8 * 33 + 6], 1.0,
              "older terrain frame placement");
  expect_near(fake->last_encoder_input[9 * 33 + 6], 2.0,
              "newest terrain frame placement");
}

void test_action_torque_limit_clipping() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output.fill(100.0F);
  rc::GazeboTerrainPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  const auto output = enter_policy_and_wait_for_inference(controller, state, command);
  for (std::size_t i = 0; i < rc::kJointCount; ++i) {
    expect_near(controller.last_network_output()[i], 100.0,
                "terrain raw policy output changed");
    expect_near(output.action[i], 2.25,
                "terrain action was not clipped from torque limit");
    expect_near(output.effort[i], 90.0,
                "terrain clipped action did not produce 90 Nm");
  }
}

void test_policy_decimation_five() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboTerrainPolicyAdapter controller(fake);
  const auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  enter_policy_and_wait_for_inference(controller, state, command);
  const auto start = controller.inference_count();
  for (int i = 0; i < 10; ++i) {
    controller.update(state, command);
  }
  expect(controller.inference_count() - start == 2,
         "ten physics ticks did not run two terrain inferences");
}

void test_policy_before_walk_ready_is_rejected() {
  auto fake = std::make_shared<FakePolicy>();
  rc::GazeboTerrainPolicyAdapter controller(fake);
  const auto output = controller.update(
      nominal_state(), mode_command(rc::ControllerMode::kPolicyOn, 1));
  expect(controller.mode() == rc::ControllerMode::kTorqueOff,
         "terrain policy entered before walk-ready completion");
  expect(output.safe_zero && exact_zero(output.effort),
         "rejected terrain policy request did not stay safe-zero");
  expect(controller.inference_count() == 0,
         "terrain inference ran before walk-ready completion");
}

void test_reset_and_nonfinite_safety() {
  auto fake = std::make_shared<FakePolicy>();
  fake->output[0] = 1.0F;
  rc::GazeboTerrainPolicyAdapter controller(fake);
  auto state = nominal_state();
  rc::UserCommand command;
  finish_walk_ready(controller, state, command);
  enter_policy_and_wait_for_inference(controller, state, command);
  expect(!exact_zero(controller.history()), "terrain history stayed zero");
  command.reset = true;
  command.reset_sequence = 1;
  auto output = controller.update(state, command);
  expect(exact_zero(controller.history()) &&
             exact_zero(controller.held_action()) &&
             controller.gait_phase() == 0.0 &&
             controller.mode() == rc::ControllerMode::kTorqueOff &&
             exact_zero(output.effort),
         "terrain reset contract mismatch");

  state.joint_velocity[4] = std::numeric_limits<double>::quiet_NaN();
  output = controller.update(state, rc::UserCommand{});
  expect(output.faulted && output.safe_zero && exact_zero(output.effort),
         "terrain NaN state did not fault safe-zero");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "usage: rubi_gazebo_terrain_core_test --case NAME\n";
    return 64;
  }
  const std::map<std::string, std::function<void()>> tests{
      {"onnx_contract_330_to_32_65_to_6",
       test_onnx_contract_330_to_32_65_to_6},
      {"observation_actor_history_and_policy_order",
       test_observation_actor_history_and_policy_order},
      {"history_oldest_to_newest", test_history_oldest_to_newest},
      {"action_torque_limit_clipping", test_action_torque_limit_clipping},
      {"policy_decimation_five", test_policy_decimation_five},
      {"policy_before_walk_ready_is_rejected",
       test_policy_before_walk_ready_is_rejected},
      {"reset_and_nonfinite_safety", test_reset_and_nonfinite_safety},
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

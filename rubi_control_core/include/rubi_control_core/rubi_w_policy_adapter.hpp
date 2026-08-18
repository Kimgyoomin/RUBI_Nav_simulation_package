#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rubi_control_core/rubi_w_policy_runner.hpp"
#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

enum class RubiWJointIndex : std::size_t {
  kLhr = 0,
  kLhp,
  kLkn,
  kLwh,
  kRhr,
  kRhp,
  kRkn,
  kRwh,
};

struct RubiWContract {
  double physics_dt{0.002};
  std::size_t inference_decimation{5};
  double inference_dt{0.01};
  double walk_ready_duration{0.5};
  double dof_position_scale{1.0};
  double dof_velocity_scale{0.1};
  double angular_velocity_scale{0.25};
  double gait_frequency{1.5};
  std::array<double, 4> gait_parameters{1.5, 0.5, 0.5, 0.0};
  std::array<double, 3> command_input_limit{0.5, 0.5, 1.0};
  std::array<double, 3> command_cmd_vel_scale{1.25, 0.75, 1.57};
  std::array<double, 3> command_policy_scale{2.0, 2.0, 0.25};
  RubiWJointArray walk_ready_pose{0.0, 0.65, -1.3, 0.0,
                                  0.0, 0.65, -1.3, 0.0};
  RubiWJointArray default_pose{0.0, 0.65, -1.3, 0.0,
                               0.0, 0.65, -1.3, 0.0};
  RubiWJointArray action_scale{1.0, 1.0, 1.0, 0.5,
                               1.0, 1.0, 1.0, 0.5};
  RubiWJointArray kp{40.0, 40.0, 40.0, 0.0,
                     40.0, 40.0, 40.0, 0.0};
  RubiWJointArray kd{2.0, 2.0, 2.0, 4.8,
                     2.0, 2.0, 2.0, 4.8};
  RubiWJointArray torque_limit{90.0, 90.0, 90.0, 90.0,
                               90.0, 90.0, 90.0, 90.0};
};

struct RubiWRobotState {
  double sim_time_sec{0.0};
  RubiWJointArray joint_position{};
  RubiWJointArray joint_velocity{};
  RubiWJointArray measured_effort{};
  std::array<double, 4> base_orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> base_angular_velocity{};
};

struct RubiWUserCommand {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  ControllerMode requested_mode{ControllerMode::kTorqueOff};
  bool reset{false};
  bool emergency_stop{false};
  std::uint64_t command_sequence{0};
  std::uint64_t mode_sequence{0};
  std::uint64_t reset_sequence{0};
};

struct RubiWControllerOutput {
  RubiWAction action{};
  RubiWJointArray target_position{};
  RubiWJointArray target_velocity{};
  RubiWJointArray effort{};
  ControllerMode mode{ControllerMode::kTorqueOff};
  bool inference_ran{false};
  bool safe_zero{true};
  bool faulted{false};
  std::string fault_reason;
};

const std::array<std::string, kRubiWJointCount>& rubi_w_controller_order();
void validate_rubi_w_controller_order(
    const std::vector<std::string>& controller_order);

class RubiWPolicyAdapter {
 public:
  explicit RubiWPolicyAdapter(
      std::shared_ptr<RubiWPolicyInterface> policy,
      RubiWContract contract = RubiWContract{});

  RubiWControllerOutput update(const RubiWRobotState& state,
                               const RubiWUserCommand& command);
  void reset();

  const RubiWObservation& last_observation() const noexcept {
    return observation_;
  }
  const RubiWHistory& history() const noexcept { return history_; }
  const RubiWLatent& last_latent() const noexcept { return latent_; }
  const RubiWPolicyInput& last_policy_input() const noexcept {
    return policy_input_;
  }
  const RubiWAction& held_action() const noexcept { return action_; }
  std::uint64_t inference_count() const noexcept { return inference_count_; }
  std::uint64_t physics_tick_count() const noexcept {
    return physics_tick_count_;
  }
  double gait_index() const noexcept { return gait_index_; }
  bool walk_ready_complete() const noexcept { return walk_ready_complete_; }
  ControllerMode mode() const noexcept { return mode_; }

 private:
  bool state_is_finite(const RubiWRobotState& state) const;
  void build_observation(const RubiWRobotState& state);
  void run_inference(const RubiWRobotState& state);
  void enter_mode(ControllerMode mode);
  RubiWControllerOutput safe_zero(const std::string& reason, bool faulted);
  RubiWControllerOutput apply_pd(const RubiWRobotState& state,
                                 const RubiWJointArray& target_position,
                                 const RubiWJointArray& target_velocity);

  std::shared_ptr<RubiWPolicyInterface> policy_;
  RubiWContract contract_;
  ControllerMode mode_{ControllerMode::kTorqueOff};
  bool emergency_latched_{false};
  bool walk_ready_complete_{false};
  std::uint64_t last_command_sequence_{0};
  std::uint64_t last_mode_sequence_{0};
  std::uint64_t last_reset_sequence_{0};
  std::uint64_t physics_tick_count_{0};
  std::uint64_t mode_tick_count_{0};
  std::uint64_t inference_count_{0};
  std::array<double, 3> held_command_{};
  double gait_index_{0.0};
  RubiWObservation observation_{};
  RubiWHistory history_{};
  RubiWLatent latent_{};
  RubiWPolicyInput policy_input_{};
  RubiWAction action_{};
  RubiWControllerOutput output_{};
};

}  // namespace rubi_control_core

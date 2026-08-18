#pragma once

#include <cstdint>
#include <memory>

#include "rubi_control_core/policy_contract.hpp"
#include "rubi_control_core/policy_runner.hpp"
#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

class Controller {
 public:
  explicit Controller(std::shared_ptr<PolicyInterface> policy,
                      PolicyContract contract = PolicyContract{});

  ControllerOutput update(const RobotState& state, const UserCommand& command);
  void reset();

  const Observation& last_observation() const noexcept { return observation_; }
  const History& history() const noexcept { return history_; }
  const Latent& last_latent() const noexcept { return latent_; }
  const PolicyInput& last_policy_input() const noexcept { return policy_input_; }
  const Action& held_action() const noexcept { return action_; }
  std::uint64_t inference_count() const noexcept { return inference_count_; }
  std::uint64_t physics_tick_count() const noexcept { return physics_tick_count_; }
  double gait_phase() const noexcept { return gait_phase_; }
  ControllerMode mode() const noexcept { return mode_; }

 private:
  bool state_is_finite(const RobotState& state) const;
  bool build_observation(const RobotState& state);
  void run_inference();
  void enter_mode(ControllerMode mode);
  ControllerOutput safe_zero(const std::string& reason, bool faulted);
  ControllerOutput apply_pd(const RobotState& state, const JointArray& target);

  std::shared_ptr<PolicyInterface> policy_;
  PolicyContract contract_;
  ControllerMode mode_{ControllerMode::kTorqueOff};
  bool emergency_latched_{false};
  std::uint64_t last_command_sequence_{0};
  std::uint64_t last_mode_sequence_{0};
  std::uint64_t last_reset_sequence_{0};
  std::uint64_t physics_tick_count_{0};
  std::uint64_t mode_tick_count_{0};
  std::uint64_t policy_tick_count_{0};
  std::uint64_t inference_count_{0};
  double command_age_{0.0};
  double gait_phase_{0.0};
  std::array<double, 3> held_command_{};
  Observation observation_{};
  History history_{};
  Latent latent_{};
  PolicyInput policy_input_{};
  Action action_{};
  ControllerOutput output_{};
};

}  // namespace rubi_control_core

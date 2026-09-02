#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "rubi_control_core/gazebo_terrain_policy_runner.hpp"
#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

struct GazeboTerrainContract {
  double physics_dt{0.002};
  std::size_t inference_decimation{5};
  double inference_dt{0.01};
  double walk_ready_duration{0.5};
  double angular_velocity_scale{0.25};
  double dof_position_scale{1.0};
  double dof_velocity_scale{0.1};
  double gait_frequency{2.0};
  std::array<double, 4> gait_parameters{2.0, 0.5, 0.5, 0.0};
  std::array<double, 3> command_policy_scale{2.0, 2.0, 0.25};
  JointArray walk_ready_pose{0.0, 0.65, -1.3, 0.0, 0.65, -1.3};
  JointArray default_pose{0.0, 0.65, -1.3, 0.0, 0.65, -1.3};
  JointArray action_scale{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  JointArray kp{40.0, 40.0, 40.0, 40.0, 40.0, 40.0};
  JointArray kd{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  JointArray torque_limit{90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
};

class GazeboTerrainPolicyAdapter {
 public:
  explicit GazeboTerrainPolicyAdapter(
      std::shared_ptr<GazeboTerrainPolicyInterface> policy,
      GazeboTerrainContract contract = GazeboTerrainContract{});

  ControllerOutput update(const RobotState& state, const UserCommand& command);
  void reset();

  const GazeboTerrainObservation& last_observation() const noexcept {
    return observation_;
  }
  const GazeboTerrainActorObservation& last_actor_observation() const noexcept {
    return actor_observation_;
  }
  const GazeboTerrainHistory& history() const noexcept { return history_; }
  const GazeboTerrainLatent& last_latent() const noexcept { return latent_; }
  const GazeboTerrainPolicyInput& last_policy_input() const noexcept {
    return policy_input_;
  }
  const Action& last_network_output() const noexcept { return network_output_; }
  const Action& held_action() const noexcept { return action_; }
  std::uint64_t inference_count() const noexcept { return inference_count_; }
  std::uint64_t physics_tick_count() const noexcept {
    return physics_tick_count_;
  }
  double gait_phase() const noexcept { return gait_phase_; }
  double cycle_time() const noexcept {
    return gait_frequency_is_valid() ? gait_phase_ / contract_.gait_frequency
                                     : 0.0;
  }
  double cycle_period() const noexcept {
    return gait_frequency_is_valid() ? 1.0 / contract_.gait_frequency : 0.0;
  }
  bool walk_ready_complete() const noexcept { return walk_ready_complete_; }
  ControllerMode mode() const noexcept { return mode_; }

 private:
  bool gait_frequency_is_valid() const noexcept;
  bool state_is_finite(const RobotState& state) const;
  void build_observation(const RobotState& state);
  void run_inference(const RobotState& state);
  void enter_mode(ControllerMode mode);
  ControllerOutput safe_zero(const std::string& reason, bool faulted);
  ControllerOutput apply_pd(const RobotState& state, const JointArray& target);

  std::shared_ptr<GazeboTerrainPolicyInterface> policy_;
  GazeboTerrainContract contract_;
  ControllerMode mode_{ControllerMode::kTorqueOff};
  bool emergency_latched_{false};
  bool walk_ready_complete_{false};
  std::uint64_t last_command_sequence_{0};
  std::uint64_t last_mode_sequence_{0};
  std::uint64_t last_reset_sequence_{0};
  std::uint64_t physics_tick_count_{0};
  std::uint64_t mode_tick_count_{0};
  std::uint64_t inference_count_{0};
  double gait_phase_{0.0};
  std::array<double, 3> held_command_{};
  GazeboTerrainObservation observation_{};
  GazeboTerrainActorObservation actor_observation_{};
  GazeboTerrainHistory history_{};
  GazeboTerrainLatent latent_{};
  GazeboTerrainPolicyInput policy_input_{};
  Action network_output_{};
  Action action_{};
  ControllerOutput output_{};
};

}  // namespace rubi_control_core

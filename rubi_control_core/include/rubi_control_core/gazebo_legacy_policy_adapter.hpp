#pragma once

#include <cstdint>
#include <memory>

#include "rubi_control_core/gazebo_legacy_policy_runner.hpp"
#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

struct GazeboLegacyContract {
  double physics_dt{0.002};
  std::size_t inference_decimation{5};
  double inference_dt{0.01};
  double walk_ready_duration{0.5};
  double angular_velocity_scale{0.25};
  double dof_position_scale{1.0};
  double dof_velocity_scale{0.1};
  double cycle_period_initial{0.4};
  double cycle_period_min{0.35};
  double cycle_period_max{0.45};
  double cycle_output_scale{0.01};
  JointArray walk_ready_pose{0.0, 0.65, -1.3, 0.0, 0.65, -1.3};
  JointArray default_pose{0.0, 0.872665, -1.74533,
                          0.0, 0.872665, -1.74533};
  JointArray action_scale{1.0, 0.5, 1.5, 1.0, 0.5, 1.5};
  JointArray kp{40.0, 40.0, 40.0, 40.0, 40.0, 40.0};
  JointArray kd{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  JointArray torque_limit{90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
};

class GazeboLegacyPolicyAdapter {
 public:
  explicit GazeboLegacyPolicyAdapter(
      std::shared_ptr<GazeboLegacyPolicyInterface> policy,
      GazeboLegacyContract contract = GazeboLegacyContract{});

  ControllerOutput update(const RobotState& state, const UserCommand& command);
  void reset();

  const GazeboLegacyObservation& last_observation() const noexcept {
    return observation_;
  }
  const GazeboLegacyHistory& history() const noexcept { return history_; }
  const GazeboLegacyNetworkOutput& last_network_output() const noexcept {
    return network_output_;
  }
  const Action& held_action() const noexcept { return action_; }
  std::uint64_t inference_count() const noexcept { return inference_count_; }
  std::uint64_t physics_tick_count() const noexcept {
    return physics_tick_count_;
  }
  double cycle_time() const noexcept { return cycle_time_; }
  double cycle_period() const noexcept { return cycle_period_; }
  bool walk_ready_complete() const noexcept { return walk_ready_complete_; }
  ControllerMode mode() const noexcept { return mode_; }

 private:
  bool state_is_finite(const RobotState& state) const;
  void build_observation(const RobotState& state);
  void run_inference(const RobotState& state);
  void enter_mode(ControllerMode mode);
  ControllerOutput safe_zero(const std::string& reason, bool faulted);
  ControllerOutput apply_pd(const RobotState& state, const JointArray& target);

  std::shared_ptr<GazeboLegacyPolicyInterface> policy_;
  GazeboLegacyContract contract_;
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
  double cycle_time_{0.0};
  double cycle_period_{0.4};
  GazeboLegacyObservation observation_{};
  GazeboLegacyHistory history_{};
  GazeboLegacyNetworkOutput network_output_{};
  Action action_{};
  ControllerOutput output_{};
};

}  // namespace rubi_control_core

#include "rubi_control_core/gazebo_legacy_policy_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rubi_control_core {
namespace {

constexpr double kPi = 3.14159265358979323846;

template <typename Container>
bool all_finite(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return std::isfinite(value); });
}

std::array<double, 3> projected_gravity(
    const std::array<double, 4>& quaternion_xyzw) {
  double x = quaternion_xyzw[0];
  double y = quaternion_xyzw[1];
  double z = quaternion_xyzw[2];
  double w = quaternion_xyzw[3];
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error("invalid base orientation quaternion");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return {2.0 * (y * w - x * z),
          -2.0 * (y * z + x * w),
          -(1.0 - 2.0 * (x * x + y * y))};
}

}  // namespace

GazeboLegacyPolicyAdapter::GazeboLegacyPolicyAdapter(
    std::shared_ptr<GazeboLegacyPolicyInterface> policy,
    GazeboLegacyContract contract)
    : policy_(std::move(policy)), contract_(std::move(contract)) {
  if (!policy_) {
    throw std::invalid_argument("Gazebo legacy policy must not be null");
  }
  if (contract_.physics_dt <= 0.0 || contract_.inference_decimation == 0 ||
      contract_.inference_dt <= 0.0 || contract_.walk_ready_duration <= 0.0 ||
      contract_.cycle_period_min <= 0.0 ||
      contract_.cycle_period_min > contract_.cycle_period_initial ||
      contract_.cycle_period_initial > contract_.cycle_period_max) {
    throw std::invalid_argument("invalid Gazebo legacy controller contract");
  }
  reset();
}

void GazeboLegacyPolicyAdapter::reset() {
  mode_ = ControllerMode::kTorqueOff;
  emergency_latched_ = false;
  walk_ready_complete_ = false;
  physics_tick_count_ = 0;
  mode_tick_count_ = 0;
  inference_count_ = 0;
  held_command_.fill(0.0);
  cycle_time_ = 0.0;
  cycle_period_ = contract_.cycle_period_initial;
  observation_.fill(0.0F);
  history_.fill(0.0F);
  network_output_.fill(0.0F);
  action_.fill(0.0F);
  output_ = ControllerOutput{};
}

bool GazeboLegacyPolicyAdapter::state_is_finite(
    const RobotState& state) const {
  return std::isfinite(state.sim_time_sec) && all_finite(state.joint_position) &&
         all_finite(state.joint_velocity) && all_finite(state.measured_effort) &&
         all_finite(state.base_orientation_xyzw) &&
         all_finite(state.base_angular_velocity);
}

void GazeboLegacyPolicyAdapter::build_observation(const RobotState& state) {
  for (std::size_t i = 0; i < 3; ++i) {
    observation_[i] = static_cast<float>(
        state.base_angular_velocity[i] * contract_.angular_velocity_scale);
  }
  const auto gravity = projected_gravity(state.base_orientation_xyzw);
  for (std::size_t i = 0; i < 3; ++i) {
    observation_[3 + i] = static_cast<float>(gravity[i]);
    observation_[6 + i] = static_cast<float>(held_command_[i]);
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    observation_[9 + i] = static_cast<float>(
        state.joint_position[i] * contract_.dof_position_scale);
    observation_[15 + i] = static_cast<float>(
        state.joint_velocity[i] * contract_.dof_velocity_scale);
    observation_[21 + i] = action_[i];
  }
  const double phase = 2.0 * kPi * cycle_time_ / cycle_period_;
  observation_[27] = static_cast<float>(std::sin(phase));
  observation_[28] = static_cast<float>(std::sin(phase + kPi));
  observation_[29] = static_cast<float>(std::cos(phase));
  observation_[30] = static_cast<float>(std::cos(phase + kPi));
  observation_[31] = static_cast<float>(cycle_period_);

  cycle_time_ += contract_.inference_dt;
  if (cycle_time_ > cycle_period_) {
    cycle_time_ = contract_.inference_dt;
  }
  if (!all_finite(observation_)) {
    throw std::runtime_error("nonfinite Gazebo legacy observation");
  }
}

void GazeboLegacyPolicyAdapter::run_inference(const RobotState& state) {
  build_observation(state);
  std::move(history_.begin() + kGazeboLegacyObservationDim, history_.end(),
            history_.begin());
  std::copy(observation_.begin(), observation_.end(),
            history_.end() - kGazeboLegacyObservationDim);
  network_output_ = policy_->infer(history_);
  if (!all_finite(network_output_)) {
    throw std::runtime_error("nonfinite Gazebo legacy policy output");
  }
  std::copy_n(network_output_.begin(), kJointCount, action_.begin());
  cycle_period_ = std::clamp(
      cycle_period_ + static_cast<double>(network_output_[6]) *
                          contract_.cycle_output_scale,
      contract_.cycle_period_min, contract_.cycle_period_max);
  ++inference_count_;
}

void GazeboLegacyPolicyAdapter::enter_mode(ControllerMode mode) {
  if (emergency_latched_ && mode != ControllerMode::kEmergencyStop) {
    return;
  }
  if (mode == ControllerMode::kPolicyOn && !walk_ready_complete_) {
    return;
  }
  if (mode_ == mode) {
    return;
  }
  mode_ = mode;
  mode_tick_count_ = 0;
  if (mode == ControllerMode::kEmergencyStop) {
    emergency_latched_ = true;
  }
}

ControllerOutput GazeboLegacyPolicyAdapter::safe_zero(
    const std::string& reason, bool faulted) {
  output_.action = action_;
  output_.target_position.fill(0.0);
  output_.effort.fill(0.0);
  output_.mode = mode_;
  output_.inference_ran = false;
  output_.safe_zero = true;
  output_.faulted = faulted;
  output_.fault_reason = reason;
  return output_;
}

ControllerOutput GazeboLegacyPolicyAdapter::apply_pd(
    const RobotState& state, const JointArray& target) {
  output_.action = action_;
  output_.target_position = target;
  output_.mode = mode_;
  output_.safe_zero = false;
  output_.faulted = false;
  output_.fault_reason.clear();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double effort =
        contract_.kp[i] * (target[i] - state.joint_position[i]) +
        contract_.kd[i] * (0.0 - state.joint_velocity[i]);
    if (!std::isfinite(effort)) {
      emergency_latched_ = true;
      mode_ = ControllerMode::kEmergencyStop;
      return safe_zero("nonfinite Gazebo legacy PD effort", true);
    }
    output_.effort[i] = std::clamp(
        effort, -contract_.torque_limit[i], contract_.torque_limit[i]);
  }
  return output_;
}

ControllerOutput GazeboLegacyPolicyAdapter::update(
    const RobotState& state, const UserCommand& command) {
  output_.inference_ran = false;
  if (command.reset && command.reset_sequence != last_reset_sequence_) {
    const auto reset_sequence = command.reset_sequence;
    const auto command_sequence = command.command_sequence;
    const auto mode_sequence = command.mode_sequence;
    reset();
    last_reset_sequence_ = reset_sequence;
    last_command_sequence_ = command_sequence;
    last_mode_sequence_ = mode_sequence;
    return safe_zero("reset", false);
  }
  if (command.command_sequence != last_command_sequence_) {
    held_command_ = {command.linear_x, command.linear_y, command.angular_z};
    last_command_sequence_ = command.command_sequence;
  }
  if (command.mode_sequence != last_mode_sequence_) {
    enter_mode(command.requested_mode);
    last_mode_sequence_ = command.mode_sequence;
  }
  if (command.emergency_stop ||
      command.requested_mode == ControllerMode::kEmergencyStop) {
    enter_mode(ControllerMode::kEmergencyStop);
  }

  if (!state_is_finite(state) || !all_finite(held_command_)) {
    emergency_latched_ = true;
    mode_ = ControllerMode::kEmergencyStop;
    ++physics_tick_count_;
    return safe_zero("nonfinite Gazebo legacy state or command", true);
  }
  if (emergency_latched_ || mode_ == ControllerMode::kEmergencyStop) {
    ++physics_tick_count_;
    return safe_zero("emergency stop latched", false);
  }
  if (mode_ == ControllerMode::kTorqueOff) {
    ++physics_tick_count_;
    return safe_zero("torque off", false);
  }

  JointArray target{};
  if (mode_ == ControllerMode::kWalkReady) {
    const double elapsed =
        static_cast<double>(mode_tick_count_) * contract_.physics_dt;
    if (elapsed <= contract_.walk_ready_duration) {
      const double blend = 0.5 *
          (1.0 - std::cos(kPi * elapsed / contract_.walk_ready_duration));
      for (std::size_t i = 0; i < kJointCount; ++i) {
        target[i] = blend * contract_.walk_ready_pose[i];
      }
      ++mode_tick_count_;
    } else {
      target = contract_.walk_ready_pose;
      walk_ready_complete_ = true;
    }
  } else {
    if (physics_tick_count_ % contract_.inference_decimation == 0) {
      try {
        run_inference(state);
        output_.inference_ran = true;
      } catch (const std::exception& error) {
        emergency_latched_ = true;
        mode_ = ControllerMode::kEmergencyStop;
        ++physics_tick_count_;
        return safe_zero(error.what(), true);
      }
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] = contract_.default_pose[i] +
                  static_cast<double>(action_[i]) * contract_.action_scale[i];
    }
  }

  ++physics_tick_count_;
  auto result = apply_pd(state, target);
  result.inference_ran = output_.inference_ran;
  output_ = result;
  return output_;
}

}  // namespace rubi_control_core

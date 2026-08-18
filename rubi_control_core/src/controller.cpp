#include "rubi_control_core/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

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

  // R(q)^T * [0, 0, -1], with q stored as xyzw.
  return {2.0 * (y * w - x * z),
          -2.0 * (y * z + x * w),
          -(1.0 - 2.0 * (x * x + y * y))};
}

}  // namespace

void validate_joint_order(const std::vector<std::string>& joint_names) {
  const auto& expected = canonical_joint_order();
  if (joint_names.size() != expected.size() ||
      !std::equal(joint_names.begin(), joint_names.end(), expected.begin())) {
    throw std::invalid_argument("joint order does not match canonical RUBI manifest");
  }
}

Controller::Controller(std::shared_ptr<PolicyInterface> policy, PolicyContract contract)
    : policy_(std::move(policy)), contract_(std::move(contract)) {
  if (!policy_) {
    throw std::invalid_argument("policy runner must not be null");
  }
  if (contract_.physics_dt <= 0.0 || contract_.inference_decimation == 0 ||
      contract_.walk_ready_duration <= 0.0 || contract_.command_timeout < 0.0) {
    throw std::invalid_argument("invalid controller timing contract");
  }
  reset();
}

void Controller::reset() {
  mode_ = ControllerMode::kTorqueOff;
  emergency_latched_ = false;
  physics_tick_count_ = 0;
  mode_tick_count_ = 0;
  policy_tick_count_ = 0;
  inference_count_ = 0;
  command_age_ = 0.0;
  gait_phase_ = 0.0;
  held_command_.fill(0.0);
  observation_.fill(0.0F);
  history_.fill(0.0F);
  latent_.fill(0.0F);
  policy_input_.fill(0.0F);
  action_.fill(0.0F);
  output_ = ControllerOutput{};
}

bool Controller::state_is_finite(const RobotState& state) const {
  return std::isfinite(state.sim_time_sec) && all_finite(state.joint_position) &&
         all_finite(state.joint_velocity) && all_finite(state.measured_effort) &&
         all_finite(state.base_orientation_xyzw) &&
         all_finite(state.base_angular_velocity);
}

bool Controller::build_observation(const RobotState& state) {
  for (std::size_t i = 0; i < 3; ++i) {
    observation_[i] = static_cast<float>(
        state.base_angular_velocity[i] * contract_.angular_velocity_scale);
  }
  const auto gravity = projected_gravity(state.base_orientation_xyzw);
  for (std::size_t i = 0; i < 3; ++i) {
    observation_[3 + i] = static_cast<float>(gravity[i]);
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    observation_[6 + i] = static_cast<float>(
        (state.joint_position[i] - contract_.default_pose[i]) *
        contract_.dof_position_scale);
    observation_[12 + i] = static_cast<float>(
        state.joint_velocity[i] * contract_.dof_velocity_scale);
    observation_[18 + i] = action_[i];
  }
  observation_[24] = static_cast<float>(std::sin(gait_phase_ * 2.0 * kPi));
  observation_[25] = static_cast<float>(std::cos(gait_phase_ * 2.0 * kPi));
  for (std::size_t i = 0; i < contract_.gait_parameters.size(); ++i) {
    observation_[26 + i] = static_cast<float>(contract_.gait_parameters[i]);
  }
  return all_finite(observation_);
}

void Controller::run_inference() {
  std::move(history_.begin() + kObservationDim, history_.end(), history_.begin());
  std::copy(observation_.begin(), observation_.end(),
            history_.end() - kObservationDim);

  latent_ = policy_->encode(history_);
  if (!all_finite(latent_)) {
    throw std::runtime_error("nonfinite encoder output");
  }
  std::copy(latent_.begin(), latent_.end(), policy_input_.begin());
  std::copy(observation_.begin(), observation_.end(),
            policy_input_.begin() + kLatentDim);

  std::array<double, 3> effective_command = held_command_;
  if (command_age_ > contract_.command_timeout) {
    effective_command.fill(0.0);
  }
  for (std::size_t i = 0; i < effective_command.size(); ++i) {
    const double normalized = std::clamp(
        effective_command[i] / contract_.command_input_limit[i], -1.0, 1.0);
    policy_input_[kLatentDim + kObservationDim + i] = static_cast<float>(
        normalized * contract_.command_legacy_joy_scale[i] *
        contract_.command_policy_scale[i]);
  }
  if (!all_finite(policy_input_)) {
    throw std::runtime_error("nonfinite policy input");
  }
  action_ = policy_->act(policy_input_);
  if (!all_finite(action_)) {
    throw std::runtime_error("nonfinite policy action");
  }
  ++inference_count_;
}

void Controller::enter_mode(ControllerMode mode) {
  if (emergency_latched_ && mode != ControllerMode::kEmergencyStop) {
    return;
  }
  if (mode_ == mode) {
    return;
  }
  mode_ = mode;
  mode_tick_count_ = 0;
  if (mode == ControllerMode::kPolicyOn) {
    policy_tick_count_ = 0;
  }
  if (mode == ControllerMode::kEmergencyStop) {
    emergency_latched_ = true;
  }
}

ControllerOutput Controller::safe_zero(const std::string& reason, bool faulted) {
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

ControllerOutput Controller::apply_pd(const RobotState& state, const JointArray& target) {
  output_.action = action_;
  output_.target_position = target;
  output_.mode = mode_;
  output_.safe_zero = false;
  output_.faulted = false;
  output_.fault_reason.clear();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double raw_effort =
        contract_.kp[i] * (target[i] - state.joint_position[i]) +
        contract_.kd[i] * (0.0 - state.joint_velocity[i]);
    if (!std::isfinite(raw_effort)) {
      emergency_latched_ = true;
      mode_ = ControllerMode::kEmergencyStop;
      return safe_zero("nonfinite PD effort", true);
    }
    output_.effort[i] = std::clamp(
        raw_effort, -contract_.torque_limit[i], contract_.torque_limit[i]);
  }
  if (!all_finite(output_.effort)) {
    emergency_latched_ = true;
    mode_ = ControllerMode::kEmergencyStop;
    return safe_zero("nonfinite clamped effort", true);
  }
  return output_;
}

ControllerOutput Controller::update(const RobotState& state, const UserCommand& command) {
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
    command_age_ = 0.0;
    last_command_sequence_ = command.command_sequence;
  } else {
    command_age_ += contract_.physics_dt;
  }

  if (command.mode_sequence != last_mode_sequence_) {
    enter_mode(command.requested_mode);
    last_mode_sequence_ = command.mode_sequence;
  }
  if (command.emergency_stop || command.requested_mode == ControllerMode::kEmergencyStop) {
    enter_mode(ControllerMode::kEmergencyStop);
  }

  if (!state_is_finite(state) || !all_finite(held_command_)) {
    emergency_latched_ = true;
    mode_ = ControllerMode::kEmergencyStop;
    return safe_zero("nonfinite state or command", true);
  }
  if (emergency_latched_ || mode_ == ControllerMode::kEmergencyStop) {
    return safe_zero("emergency stop latched", false);
  }
  if (mode_ == ControllerMode::kTorqueOff) {
    return safe_zero("torque off", false);
  }

  JointArray target{};
  if (mode_ == ControllerMode::kWalkReady) {
    const double elapsed = static_cast<double>(mode_tick_count_ + 1) * contract_.physics_dt;
    const double progress = std::clamp(elapsed / contract_.walk_ready_duration, 0.0, 1.0);
    const double blend = 0.5 * (1.0 - std::cos(kPi * progress));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] = blend * contract_.default_pose[i];
    }
    ++mode_tick_count_;
  } else {
    if (policy_tick_count_ % contract_.inference_decimation == 0) {
      try {
        gait_phase_ += contract_.physics_dt *
                       static_cast<double>(contract_.inference_decimation) *
                       contract_.gait_frequency;
        gait_phase_ -= std::floor(gait_phase_);
        if (!build_observation(state)) {
          throw std::runtime_error("nonfinite observation");
        }
        run_inference();
        output_.inference_ran = true;
      } catch (const std::exception& error) {
        emergency_latched_ = true;
        mode_ = ControllerMode::kEmergencyStop;
        return safe_zero(error.what(), true);
      }
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] = contract_.default_pose[i] +
                  static_cast<double>(action_[i]) * contract_.action_scale[i];
    }
    ++policy_tick_count_;
  }

  ++physics_tick_count_;
  auto result = apply_pd(state, target);
  result.inference_ran = output_.inference_ran;
  output_ = result;
  return output_;
}

}  // namespace rubi_control_core

#pragma once

#include <array>
#include <string>
#include <vector>

#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

struct PolicyContract {
  double physics_dt{0.002};
  std::size_t inference_decimation{5};
  double command_timeout{0.5};
  double walk_ready_duration{0.5};
  double dof_position_scale{1.0};
  double dof_velocity_scale{0.1};
  double angular_velocity_scale{0.25};
  double gait_frequency{2.5};
  std::array<double, 4> gait_parameters{2.5, 0.5, 0.5, 0.0};
  std::array<double, 3> command_input_limit{0.5, 0.5, 1.0};
  std::array<double, 3> command_legacy_joy_scale{1.0, 0.75, 1.5};
  std::array<double, 3> command_policy_scale{2.0, 2.0, 0.25};
  JointArray default_pose{-0.0, 0.65, -1.3, 0.0, 0.65, -1.3};
  JointArray kp{40.0, 40.0, 40.0, 40.0, 40.0, 40.0};
  JointArray kd{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  JointArray action_scale{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  JointArray torque_limit{90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
};

inline const std::array<std::string, kJointCount>& canonical_joint_order() {
  static const std::array<std::string, kJointCount> names{
      "L_HR_JOINT", "L_HP_JOINT", "L_KN_JOINT",
      "R_HR_JOINT", "R_HP_JOINT", "R_KN_JOINT"};
  return names;
}

void validate_joint_order(const std::vector<std::string>& joint_names);

}  // namespace rubi_control_core

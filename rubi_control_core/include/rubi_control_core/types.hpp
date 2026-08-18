#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace rubi_control_core {

constexpr std::size_t kJointCount = 6;
constexpr std::size_t kObservationDim = 30;
constexpr std::size_t kHistoryLength = 10;
constexpr std::size_t kEncoderInputDim = kObservationDim * kHistoryLength;
constexpr std::size_t kLatentDim = 3;
constexpr std::size_t kPolicyInputDim = kLatentDim + kObservationDim + 3;
constexpr std::size_t kActionDim = kJointCount;

using JointArray = std::array<double, kJointCount>;
using Observation = std::array<float, kObservationDim>;
using History = std::array<float, kEncoderInputDim>;
using Latent = std::array<float, kLatentDim>;
using PolicyInput = std::array<float, kPolicyInputDim>;
using Action = std::array<float, kActionDim>;

enum class ControllerMode {
  kTorqueOff,
  kWalkReady,
  kPolicyOn,
  kEmergencyStop,
};

struct RobotState {
  double sim_time_sec{0.0};
  JointArray joint_position{};
  JointArray joint_velocity{};
  JointArray measured_effort{};
  std::array<double, 4> base_orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> base_angular_velocity{};
};

struct UserCommand {
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

struct ControllerOutput {
  Action action{};
  JointArray target_position{};
  JointArray effort{};
  ControllerMode mode{ControllerMode::kTorqueOff};
  bool inference_ran{false};
  bool safe_zero{true};
  bool faulted{false};
  std::string fault_reason;
};

}  // namespace rubi_control_core

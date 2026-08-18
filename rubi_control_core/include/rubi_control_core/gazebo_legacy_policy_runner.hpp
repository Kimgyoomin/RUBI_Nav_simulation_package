#pragma once

#include <array>
#include <memory>
#include <string>

namespace rubi_control_core {

constexpr std::size_t kGazeboLegacyObservationDim = 32;
constexpr std::size_t kGazeboLegacyHistoryLength = 10;
constexpr std::size_t kGazeboLegacyInputDim =
    kGazeboLegacyObservationDim * kGazeboLegacyHistoryLength;
constexpr std::size_t kGazeboLegacyOutputDim = 7;

using GazeboLegacyObservation =
    std::array<float, kGazeboLegacyObservationDim>;
using GazeboLegacyHistory = std::array<float, kGazeboLegacyInputDim>;
using GazeboLegacyNetworkOutput =
    std::array<float, kGazeboLegacyOutputDim>;

class GazeboLegacyPolicyInterface {
 public:
  virtual ~GazeboLegacyPolicyInterface() = default;
  virtual GazeboLegacyNetworkOutput infer(
      const GazeboLegacyHistory& history) = 0;
};

class GazeboLegacyPolicyRunner final : public GazeboLegacyPolicyInterface {
 public:
  explicit GazeboLegacyPolicyRunner(const std::string& policy_path);
  ~GazeboLegacyPolicyRunner() override;

  GazeboLegacyPolicyRunner(const GazeboLegacyPolicyRunner&) = delete;
  GazeboLegacyPolicyRunner& operator=(const GazeboLegacyPolicyRunner&) = delete;
  GazeboLegacyPolicyRunner(GazeboLegacyPolicyRunner&&) noexcept;
  GazeboLegacyPolicyRunner& operator=(GazeboLegacyPolicyRunner&&) noexcept;

  GazeboLegacyNetworkOutput infer(
      const GazeboLegacyHistory& history) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rubi_control_core

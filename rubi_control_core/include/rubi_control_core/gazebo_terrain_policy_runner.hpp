#pragma once

#include <array>
#include <memory>
#include <string>

#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

constexpr std::size_t kGazeboTerrainObservationDim = 30;
constexpr std::size_t kGazeboTerrainCommandDim = 3;
constexpr std::size_t kGazeboTerrainActorObservationDim =
    kGazeboTerrainObservationDim + kGazeboTerrainCommandDim;
constexpr std::size_t kGazeboTerrainHistoryLength = 10;
constexpr std::size_t kGazeboTerrainEncoderInputDim =
    kGazeboTerrainActorObservationDim * kGazeboTerrainHistoryLength;
constexpr std::size_t kGazeboTerrainLatentDim = 32;
constexpr std::size_t kGazeboTerrainPolicyInputDim =
    kGazeboTerrainLatentDim + kGazeboTerrainActorObservationDim;

using GazeboTerrainObservation =
    std::array<float, kGazeboTerrainObservationDim>;
using GazeboTerrainActorObservation =
    std::array<float, kGazeboTerrainActorObservationDim>;
using GazeboTerrainHistory =
    std::array<float, kGazeboTerrainEncoderInputDim>;
using GazeboTerrainLatent = std::array<float, kGazeboTerrainLatentDim>;
using GazeboTerrainPolicyInput =
    std::array<float, kGazeboTerrainPolicyInputDim>;

class GazeboTerrainPolicyInterface {
 public:
  virtual ~GazeboTerrainPolicyInterface() = default;
  virtual GazeboTerrainLatent encode(const GazeboTerrainHistory& history) = 0;
  virtual Action act(const GazeboTerrainPolicyInput& input) = 0;
};

class GazeboTerrainPolicyRunner final : public GazeboTerrainPolicyInterface {
 public:
  GazeboTerrainPolicyRunner(const std::string& encoder_path,
                            const std::string& policy_path);
  ~GazeboTerrainPolicyRunner() override;

  GazeboTerrainPolicyRunner(const GazeboTerrainPolicyRunner&) = delete;
  GazeboTerrainPolicyRunner& operator=(const GazeboTerrainPolicyRunner&) = delete;
  GazeboTerrainPolicyRunner(GazeboTerrainPolicyRunner&&) noexcept;
  GazeboTerrainPolicyRunner& operator=(GazeboTerrainPolicyRunner&&) noexcept;

  GazeboTerrainLatent encode(const GazeboTerrainHistory& history) override;
  Action act(const GazeboTerrainPolicyInput& input) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rubi_control_core

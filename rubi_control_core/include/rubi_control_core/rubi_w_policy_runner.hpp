#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace rubi_control_core {

constexpr std::size_t kRubiWJointCount = 8;
constexpr std::size_t kRubiWObservationDim = 34;
constexpr std::size_t kRubiWHistoryLength = 10;
constexpr std::size_t kRubiWEncoderInputDim =
    kRubiWObservationDim * kRubiWHistoryLength;
constexpr std::size_t kRubiWLatentDim = 3;
constexpr std::size_t kRubiWPolicyInputDim =
    kRubiWLatentDim + kRubiWObservationDim + 3;
constexpr std::size_t kRubiWActionDim = kRubiWJointCount;

using RubiWJointArray = std::array<double, kRubiWJointCount>;
using RubiWObservation = std::array<float, kRubiWObservationDim>;
using RubiWHistory = std::array<float, kRubiWEncoderInputDim>;
using RubiWLatent = std::array<float, kRubiWLatentDim>;
using RubiWPolicyInput = std::array<float, kRubiWPolicyInputDim>;
using RubiWAction = std::array<float, kRubiWActionDim>;

class RubiWPolicyInterface {
 public:
  virtual ~RubiWPolicyInterface() = default;
  virtual RubiWLatent encode(const RubiWHistory& history) = 0;
  virtual RubiWAction act(const RubiWPolicyInput& input) = 0;
};

class RubiWPolicyRunner final : public RubiWPolicyInterface {
 public:
  RubiWPolicyRunner(const std::string& encoder_path,
                    const std::string& policy_path);
  ~RubiWPolicyRunner() override;

  RubiWPolicyRunner(const RubiWPolicyRunner&) = delete;
  RubiWPolicyRunner& operator=(const RubiWPolicyRunner&) = delete;
  RubiWPolicyRunner(RubiWPolicyRunner&&) noexcept;
  RubiWPolicyRunner& operator=(RubiWPolicyRunner&&) noexcept;

  RubiWLatent encode(const RubiWHistory& history) override;
  RubiWAction act(const RubiWPolicyInput& input) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rubi_control_core

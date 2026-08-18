#pragma once

#include <memory>
#include <string>

#include "rubi_control_core/types.hpp"

namespace rubi_control_core {

class PolicyInterface {
 public:
  virtual ~PolicyInterface() = default;
  virtual Latent encode(const History& history) = 0;
  virtual Action act(const PolicyInput& input) = 0;
};

class PolicyRunner final : public PolicyInterface {
 public:
  PolicyRunner(const std::string& encoder_path, const std::string& policy_path);
  ~PolicyRunner() override;

  PolicyRunner(const PolicyRunner&) = delete;
  PolicyRunner& operator=(const PolicyRunner&) = delete;
  PolicyRunner(PolicyRunner&&) noexcept;
  PolicyRunner& operator=(PolicyRunner&&) noexcept;

  Latent encode(const History& history) override;
  Action act(const PolicyInput& input) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rubi_control_core

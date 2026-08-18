#include "rubi_control_core/gazebo_legacy_policy_runner.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rubi_control_core {
namespace {

void validate_session(Ort::Session& session) {
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
    throw std::runtime_error(
        "Gazebo legacy policy: expected exactly one input and output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  auto input_name = session.GetInputNameAllocated(0, allocator);
  auto output_name = session.GetOutputNameAllocated(0, allocator);
  const auto input_type = session.GetInputTypeInfo(0);
  const auto output_type = session.GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  const auto input_shape = input_info.GetShape();
  const auto output_shape = output_info.GetShape();

  if (std::string(input_name.get()) != "initial_obs" ||
      std::string(output_name.get()) != "action_output") {
    throw std::runtime_error(
        "Gazebo legacy policy: tensor name contract mismatch");
  }
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    throw std::runtime_error(
        "Gazebo legacy policy: tensor dtype must be float32");
  }
  const bool input_ok =
      input_shape.size() == 2 && input_shape[1] == kGazeboLegacyInputDim &&
      (input_shape[0] == -1 || input_shape[0] == 1);
  const bool output_ok =
      output_shape.size() == 2 && output_shape[1] == kGazeboLegacyOutputDim &&
      (output_shape[0] == -1 || output_shape[0] == 1);
  if (!input_ok || !output_ok) {
    throw std::runtime_error(
        "Gazebo legacy policy: expected normalized shapes 320->7");
  }
}

}  // namespace

class GazeboLegacyPolicyRunner::Impl {
 public:
  explicit Impl(const std::string& policy_path)
      : environment(ORT_LOGGING_LEVEL_WARNING, "rubi_gazebo_legacy_policy"),
        session(nullptr) {
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    options.SetIntraOpNumThreads(1);
    session = Ort::Session(environment, policy_path.c_str(), options);
    validate_session(session);
  }

  Ort::Env environment;
  Ort::SessionOptions options;
  Ort::Session session;
};

GazeboLegacyPolicyRunner::GazeboLegacyPolicyRunner(
    const std::string& policy_path)
    : impl_(std::make_unique<Impl>(policy_path)) {}

GazeboLegacyPolicyRunner::~GazeboLegacyPolicyRunner() = default;
GazeboLegacyPolicyRunner::GazeboLegacyPolicyRunner(
    GazeboLegacyPolicyRunner&&) noexcept = default;
GazeboLegacyPolicyRunner& GazeboLegacyPolicyRunner::operator=(
    GazeboLegacyPolicyRunner&&) noexcept = default;

GazeboLegacyNetworkOutput GazeboLegacyPolicyRunner::infer(
    const GazeboLegacyHistory& history) {
  std::array<int64_t, 2> input_shape{1, kGazeboLegacyInputDim};
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<float>(
      memory, const_cast<float*>(history.data()), history.size(),
      input_shape.data(), input_shape.size());
  const char* input_names[] = {"initial_obs"};
  const char* output_names[] = {"action_output"};
  auto outputs = impl_->session.Run(Ort::RunOptions{nullptr}, input_names,
                                    &tensor, 1, output_names, 1);
  if (outputs.size() != 1 || !outputs.front().IsTensor()) {
    throw std::runtime_error(
        "Gazebo legacy ONNX inference did not return one tensor");
  }
  const auto info = outputs.front().GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      info.GetShape() != std::vector<int64_t>{1, kGazeboLegacyOutputDim}) {
    throw std::runtime_error(
        "Gazebo legacy ONNX runtime output contract mismatch");
  }
  GazeboLegacyNetworkOutput result{};
  const float* values = outputs.front().GetTensorData<float>();
  std::copy(values, values + result.size(), result.begin());
  return result;
}

}  // namespace rubi_control_core

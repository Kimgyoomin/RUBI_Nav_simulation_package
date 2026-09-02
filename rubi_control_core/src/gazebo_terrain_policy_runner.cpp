#include "rubi_control_core/gazebo_terrain_policy_runner.hpp"

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

void validate_session(Ort::Session& session, std::size_t input_dim,
                      std::size_t output_dim, const std::string& label) {
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
    throw std::runtime_error(label + ": expected exactly one input and output");
  }

  Ort::AllocatorWithDefaultOptions allocator;
  auto input_name = session.GetInputNameAllocated(0, allocator);
  auto output_name = session.GetOutputNameAllocated(0, allocator);
  const auto input_type = session.GetInputTypeInfo(0);
  const auto output_type = session.GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  const std::vector<int64_t> expected_input{static_cast<int64_t>(input_dim)};
  const std::vector<int64_t> expected_output{static_cast<int64_t>(output_dim)};

  if (std::string(input_name.get()) != "mlp_input" ||
      std::string(output_name.get()) != "mlp_output") {
    throw std::runtime_error(label + ": tensor name contract mismatch");
  }
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    throw std::runtime_error(label + ": tensor dtype must be float32");
  }
  if (input_info.GetShape() != expected_input ||
      output_info.GetShape() != expected_output) {
    throw std::runtime_error(label + ": tensor shape contract mismatch");
  }
}

template <std::size_t InputDim, std::size_t OutputDim>
std::array<float, OutputDim> infer(Ort::Session& session,
                                   const std::array<float, InputDim>& input) {
  std::array<int64_t, 1> input_shape{static_cast<int64_t>(InputDim)};
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<float>(
      memory, const_cast<float*>(input.data()), input.size(), input_shape.data(),
      input_shape.size());
  const char* input_names[] = {"mlp_input"};
  const char* output_names[] = {"mlp_output"};
  auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1,
                             output_names, 1);
  if (outputs.size() != 1 || !outputs.front().IsTensor()) {
    throw std::runtime_error("terrain ONNX inference did not return one tensor");
  }
  const auto info = outputs.front().GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      info.GetShape() !=
          std::vector<int64_t>{static_cast<int64_t>(OutputDim)}) {
    throw std::runtime_error(
        "terrain ONNX inference output contract changed at runtime");
  }
  std::array<float, OutputDim> result{};
  const float* values = outputs.front().template GetTensorData<float>();
  std::copy(values, values + OutputDim, result.begin());
  return result;
}

}  // namespace

class GazeboTerrainPolicyRunner::Impl {
 public:
  Impl(const std::string& encoder_path, const std::string& policy_path)
      : environment(ORT_LOGGING_LEVEL_WARNING, "rubi_gazebo_terrain_policy"),
        encoder(nullptr),
        policy(nullptr) {
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    options.SetIntraOpNumThreads(1);
    encoder = Ort::Session(environment, encoder_path.c_str(), options);
    policy = Ort::Session(environment, policy_path.c_str(), options);
    validate_session(encoder, kGazeboTerrainEncoderInputDim,
                     kGazeboTerrainLatentDim, "terrain encoder");
    validate_session(policy, kGazeboTerrainPolicyInputDim, kActionDim,
                     "terrain policy");
  }

  Ort::Env environment;
  Ort::SessionOptions options;
  Ort::Session encoder;
  Ort::Session policy;
};

GazeboTerrainPolicyRunner::GazeboTerrainPolicyRunner(
    const std::string& encoder_path, const std::string& policy_path)
    : impl_(std::make_unique<Impl>(encoder_path, policy_path)) {}

GazeboTerrainPolicyRunner::~GazeboTerrainPolicyRunner() = default;
GazeboTerrainPolicyRunner::GazeboTerrainPolicyRunner(
    GazeboTerrainPolicyRunner&&) noexcept = default;
GazeboTerrainPolicyRunner& GazeboTerrainPolicyRunner::operator=(
    GazeboTerrainPolicyRunner&&) noexcept = default;

GazeboTerrainLatent GazeboTerrainPolicyRunner::encode(
    const GazeboTerrainHistory& history) {
  return infer<kGazeboTerrainEncoderInputDim, kGazeboTerrainLatentDim>(
      impl_->encoder, history);
}

Action GazeboTerrainPolicyRunner::act(const GazeboTerrainPolicyInput& input) {
  return infer<kGazeboTerrainPolicyInputDim, kActionDim>(impl_->policy, input);
}

}  // namespace rubi_control_core

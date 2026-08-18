#include <onnxruntime_cxx_api.h>

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string shape_string(const std::vector<int64_t>& shape) {
  std::ostringstream stream;
  stream << '[';
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) {
      stream << ',';
    }
    stream << shape[i];
  }
  stream << ']';
  return stream.str();
}

const char* element_type_name(ONNXTensorElementDataType type) {
  switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return "float32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return "float64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return "int32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return "int64";
    default:
      return "other";
  }
}

struct TensorContract {
  std::string name;
  std::vector<int64_t> shape;
  ONNXTensorElementDataType type;
};

TensorContract input_contract(Ort::Session& session, std::size_t index,
                              Ort::AllocatorWithDefaultOptions& allocator) {
  auto name = session.GetInputNameAllocated(index, allocator);
  auto type_info = session.GetInputTypeInfo(index);
  auto tensor = type_info.GetTensorTypeAndShapeInfo();
  return {name.get(), tensor.GetShape(), tensor.GetElementType()};
}

TensorContract output_contract(Ort::Session& session, std::size_t index,
                               Ort::AllocatorWithDefaultOptions& allocator) {
  auto name = session.GetOutputNameAllocated(index, allocator);
  auto type_info = session.GetOutputTypeInfo(index);
  auto tensor = type_info.GetTensorTypeAndShapeInfo();
  return {name.get(), tensor.GetShape(), tensor.GetElementType()};
}

bool has_last_dimension(const TensorContract& contract, int64_t expected) {
  return !contract.shape.empty() && contract.shape.back() == expected;
}

bool supported_feature_shape(const TensorContract& contract, int64_t expected) {
  if (!has_last_dimension(contract, expected)) {
    return false;
  }
  return contract.shape.size() == 1 ||
         (contract.shape.size() == 2 && contract.shape.front() <= 1);
}

bool supported_name_pair(const TensorContract& input, const TensorContract& output) {
  return (input.name == "mlp_input" && output.name == "mlp_output") ||
         (input.name == "initial_obs" && output.name == "action_output");
}

std::vector<int64_t> concrete_shape(const std::vector<int64_t>& raw_shape) {
  auto shape = raw_shape;
  for (auto& dimension : shape) {
    if (dimension <= 0) {
      dimension = 1;
    }
  }
  return shape;
}

std::size_t element_count(const std::vector<int64_t>& shape) {
  return std::accumulate(shape.begin(), shape.end(), std::size_t{1},
                         [](std::size_t product, int64_t dimension) {
                           return product * static_cast<std::size_t>(dimension);
                         });
}

std::vector<float> run_inference(Ort::Session& session,
                                 const TensorContract& input_contract,
                                 const TensorContract& output_contract,
                                 std::vector<float> input) {
  const auto input_shape = concrete_shape(input_contract.shape);
  const auto expected_output_shape = concrete_shape(output_contract.shape);
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  auto input_tensor = Ort::Value::CreateTensor<float>(
      memory, input.data(), input.size(), input_shape.data(), input_shape.size());
  const char* input_names[] = {input_contract.name.c_str()};
  const char* output_names[] = {output_contract.name.c_str()};
  auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1,
                             output_names, 1);
  if (outputs.size() != 1 || !outputs.front().IsTensor()) {
    throw std::runtime_error("inference did not return one tensor");
  }
  const auto actual_output_shape =
      outputs.front().GetTensorTypeAndShapeInfo().GetShape();
  if (concrete_shape(actual_output_shape) != expected_output_shape) {
    throw std::runtime_error("inference output shape changed at runtime");
  }
  const auto count =
      outputs.front().GetTensorTypeAndShapeInfo().GetElementCount();
  const float* values = outputs.front().GetTensorData<float>();
  return {values, values + count};
}

bool all_finite(const std::vector<float>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

void print_values(const char* label, const std::vector<float>& values) {
  const auto [minimum, maximum] = std::minmax_element(values.begin(), values.end());
  std::cout << label << "_values=[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      std::cout << ',';
    }
    std::cout << values[i];
  }
  std::cout << "]\n" << label << "_min=" << *minimum << '\n'
            << label << "_max=" << *maximum << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 4 && argc != 5) {
    std::cerr << "usage: rubi_onnx_inspect MODEL [EXPECTED_INPUT_DIM "
                 "EXPECTED_OUTPUT_DIM [--probe]]\n";
    return 64;
  }
  const bool probe = argc == 5 && std::string(argv[4]) == "--probe";
  if (argc == 5 && !probe) {
    std::cerr << "unknown option: " << argv[4] << '\n';
    return 64;
  }

  try {
    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "rubi_onnx_inspect");
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    Ort::Session session(environment, argv[1], options);
    Ort::AllocatorWithDefaultOptions allocator;

    std::cout << "model=" << argv[1] << '\n';
    std::cout << "onnxruntime_api_version=" << ORT_API_VERSION << '\n';
    std::cout << "input_count=" << session.GetInputCount() << '\n';
    std::cout << "output_count=" << session.GetOutputCount() << '\n';

    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
      std::cerr << "contract_status=FAIL reason=expected_one_input_and_one_output\n";
      return 2;
    }

    const auto input = input_contract(session, 0, allocator);
    const auto output = output_contract(session, 0, allocator);
    std::cout << "input_name=" << input.name << '\n';
    std::cout << "input_shape=" << shape_string(input.shape) << '\n';
    std::cout << "input_dtype=" << element_type_name(input.type) << '\n';
    std::cout << "output_name=" << output.name << '\n';
    std::cout << "output_shape=" << shape_string(output.shape) << '\n';
    std::cout << "output_dtype=" << element_type_name(output.type) << '\n';
    std::cout << "normalized_input_features="
              << (input.shape.empty() ? -1 : input.shape.back()) << '\n';
    std::cout << "normalized_output_features="
              << (output.shape.empty() ? -1 : output.shape.back()) << '\n';
    std::cout << "input_dynamic_batch="
              << (input.shape.size() == 2 && input.shape.front() <= 0 ? "true" : "false")
              << '\n';
    std::cout << "output_dynamic_batch="
              << (output.shape.size() == 2 && output.shape.front() <= 0 ? "true" : "false")
              << '\n';

    bool valid = supported_name_pair(input, output);
    valid = valid && input.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
    valid = valid && output.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;

    if (argc == 4) {
      const auto expected_input = std::strtoll(argv[2], nullptr, 10);
      const auto expected_output = std::strtoll(argv[3], nullptr, 10);
      valid = valid && supported_feature_shape(input, expected_input);
      valid = valid && supported_feature_shape(output, expected_output);
      std::cout << "expected_input_last_dim=" << expected_input << '\n';
      std::cout << "expected_output_last_dim=" << expected_output << '\n';
    }

    if (argc == 5) {
      const auto expected_input = std::strtoll(argv[2], nullptr, 10);
      const auto expected_output = std::strtoll(argv[3], nullptr, 10);
      valid = valid && supported_feature_shape(input, expected_input);
      valid = valid && supported_feature_shape(output, expected_output);
      std::cout << "expected_input_last_dim=" << expected_input << '\n';
      std::cout << "expected_output_last_dim=" << expected_output << '\n';

      const auto input_count = element_count(concrete_shape(input.shape));
      std::vector<float> zero_input(input_count, 0.0F);
      const auto zero_output = run_inference(session, input, output, zero_input);
      valid = valid && zero_output.size() == static_cast<std::size_t>(expected_output);
      valid = valid && all_finite(zero_output);
      print_values("zero_output", zero_output);

      std::vector<float> synthetic_input(input_count);
      for (std::size_t i = 0; i < synthetic_input.size(); ++i) {
        synthetic_input[i] = static_cast<float>((static_cast<int>(i % 17) - 8) * 0.03125);
      }
      const auto synthetic_output =
          run_inference(session, input, output, synthetic_input);
      valid = valid && synthetic_output.size() == static_cast<std::size_t>(expected_output);
      valid = valid && all_finite(synthetic_output);
      print_values("synthetic_output", synthetic_output);

      bool deterministic = true;
      for (int repetition = 1; repetition < 10; ++repetition) {
        deterministic = deterministic &&
            run_inference(session, input, output, synthetic_input) == synthetic_output;
      }
      valid = valid && deterministic;
      std::cout << "repeat_count=10\n";
      std::cout << "deterministic=" << (deterministic ? "true" : "false") << '\n';
      std::cout << "finite="
                << (all_finite(zero_output) && all_finite(synthetic_output) ? "true" : "false")
                << '\n';
    }

    std::cout << "contract_status=" << (valid ? "PASS" : "MISMATCH") << '\n';
    return valid ? 0 : 2;
  } catch (const Ort::Exception& error) {
    std::cerr << "contract_status=FAIL onnxruntime_error=" << error.what() << '\n';
    return 3;
  } catch (const std::exception& error) {
    std::cerr << "contract_status=FAIL error=" << error.what() << '\n';
    return 4;
  }
}

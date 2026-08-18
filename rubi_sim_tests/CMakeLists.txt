cmake_minimum_required(VERSION 3.8)
project(rubi_sim_tests)

find_package(ament_cmake REQUIRED)

set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/onnxruntime")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "Vendored ONNX Runtime headers are missing")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR "Vendored ONNX Runtime library is missing")
endif()

add_library(onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_ROOT}/include"
)

add_executable(rubi_onnx_inspect src/onnx_inspect.cpp)
target_compile_features(rubi_onnx_inspect PRIVATE cxx_std_17)
target_link_libraries(rubi_onnx_inspect PRIVATE onnxruntime)
set_target_properties(rubi_onnx_inspect PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS rubi_onnx_inspect
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES models/encoder.onnx models/policy.onnx
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY vendor/onnxruntime/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime.so*"
)
install(DIRECTORY vendor/onnxruntime/include/
  DESTINATION include/${PROJECT_NAME}/onnxruntime
)
install(FILES vendor/onnxruntime/LICENSE
  DESTINATION share/${PROJECT_NAME}/vendor/onnxruntime
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../config/rubi_policy_manifest.yaml"
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_test(
    NAME canonical_encoder_contract_and_inference
    COMMAND rubi_onnx_inspect
      "${CMAKE_CURRENT_SOURCE_DIR}/models/encoder.onnx" 300 3 --probe
  )
  add_test(
    NAME canonical_policy_contract_and_inference
    COMMAND rubi_onnx_inspect
      "${CMAKE_CURRENT_SOURCE_DIR}/models/policy.onnx" 36 6 --probe
  )
endif()

ament_package()

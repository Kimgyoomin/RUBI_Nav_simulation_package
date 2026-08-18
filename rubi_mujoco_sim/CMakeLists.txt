cmake_minimum_required(VERSION 3.8)
project(rubi_mujoco_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glfw3 QUIET)
find_package(PNG QUIET)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rubi_control_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}" CACHE PATH "MuJoCo installation root")
set(MUJOCO_INCLUDE_DIR "" CACHE PATH "Directory containing mujoco/mujoco.h")
set(MUJOCO_LIBRARY "" CACHE FILEPATH "MuJoCo shared library")

if(MUJOCO_ROOT AND (NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY))
  find_path(_mujoco_root_include mujoco/mujoco.h
    PATHS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH)
  find_library(_mujoco_root_library NAMES mujoco
    PATHS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin" NO_DEFAULT_PATH)
  if(_mujoco_root_include AND _mujoco_root_library)
    set(MUJOCO_INCLUDE_DIR "${_mujoco_root_include}" CACHE PATH "" FORCE)
    set(MUJOCO_LIBRARY "${_mujoco_root_library}" CACHE FILEPATH "" FORCE)
  endif()
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  find_path(_mujoco_system_include mujoco/mujoco.h)
  find_library(_mujoco_system_library NAMES mujoco)
  if(_mujoco_system_include AND _mujoco_system_library)
    set(MUJOCO_INCLUDE_DIR "${_mujoco_system_include}" CACHE PATH "" FORCE)
    set(MUJOCO_LIBRARY "${_mujoco_system_library}" CACHE FILEPATH "" FORCE)
  endif()
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import mujoco,pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)"
    RESULT_VARIABLE _mujoco_python_result
    OUTPUT_VARIABLE _mujoco_python_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(_mujoco_python_result EQUAL 0)
    set(_mujoco_wheel_include "${_mujoco_python_dir}/include")
    file(GLOB _mujoco_wheel_libraries "${_mujoco_python_dir}/libmujoco.so*")
    list(SORT _mujoco_wheel_libraries)
    if(EXISTS "${_mujoco_wheel_include}/mujoco/mujoco.h" AND _mujoco_wheel_libraries)
      list(GET _mujoco_wheel_libraries 0 _mujoco_wheel_library)
      set(MUJOCO_INCLUDE_DIR "${_mujoco_wheel_include}" CACHE PATH "" FORCE)
      set(MUJOCO_LIBRARY "${_mujoco_wheel_library}" CACHE FILEPATH "" FORCE)
    endif()
  endif()
endif()

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" OR NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR "MuJoCo C++ headers/library not found; set MUJOCO_ROOT")
endif()

get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
message(STATUS "RUBI MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "RUBI MuJoCo library: ${MUJOCO_LIBRARY}")

add_library(MuJoCo::MuJoCo SHARED IMPORTED)
set_target_properties(MuJoCo::MuJoCo PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

add_executable(rubi_mujoco_model_smoke src/model_smoke.cpp)
target_compile_features(rubi_mujoco_model_smoke PRIVATE cxx_std_17)
target_link_libraries(rubi_mujoco_model_smoke PRIVATE MuJoCo::MuJoCo)
set_target_properties(rubi_mujoco_model_smoke PROPERTIES
  BUILD_RPATH "${MUJOCO_LIBRARY_DIR}"
  INSTALL_RPATH "${MUJOCO_LIBRARY_DIR}"
)

add_executable(rubi_mujoco_node src/controller_node.cpp)
target_compile_features(rubi_mujoco_node PRIVATE cxx_std_17)
ament_target_dependencies(rubi_mujoco_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  rosgraph_msgs
  rubi_control_core
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(rubi_mujoco_node MuJoCo::MuJoCo)
if(TARGET glfw)
  target_link_libraries(rubi_mujoco_node glfw)
  target_compile_definitions(rubi_mujoco_node PRIVATE RUBI_MUJOCO_HAS_GLFW=1)
  message(STATUS "RUBI MuJoCo GUI: GLFW renderer enabled")
else()
  message(WARNING "RUBI MuJoCo GUI: GLFW not found; headless remains available")
endif()
if(TARGET PNG::PNG)
  target_link_libraries(rubi_mujoco_node PNG::PNG)
  target_compile_definitions(rubi_mujoco_node PRIVATE RUBI_MUJOCO_HAS_PNG=1)
endif()
set_target_properties(rubi_mujoco_node PROPERTIES
  BUILD_RPATH "${MUJOCO_LIBRARY_DIR}"
  INSTALL_RPATH "${MUJOCO_LIBRARY_DIR}"
)

install(TARGETS rubi_mujoco_model_smoke rubi_mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_test(
    NAME rubi_mujoco_model_smoke_test
    COMMAND rubi_mujoco_model_smoke
      "${CMAKE_CURRENT_SOURCE_DIR}/models/rubi.xml" 100
  )
  set_tests_properties(rubi_mujoco_model_smoke_test PROPERTIES TIMEOUT 30)
  add_test(
    NAME rubi_mujoco_controller_self_test
    COMMAND rubi_mujoco_node --ros-args
      -p self_test:=true
      -p run_as_fast_as_possible:=true
  )
  set_tests_properties(rubi_mujoco_controller_self_test PROPERTIES TIMEOUT 60)
endif()

ament_package()

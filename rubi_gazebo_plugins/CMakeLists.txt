cmake_minimum_required(VERSION 3.8)
project(rubi_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rubi_control_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(rubi_unified_policy_plugin SHARED
  src/rubi_unified_policy_plugin.cpp
)
add_library(rubi_gazebo_legacy_policy_plugin SHARED
  src/rubi_gazebo_legacy_policy_plugin.cpp
)
target_compile_features(rubi_unified_policy_plugin PRIVATE cxx_std_17)
target_include_directories(rubi_unified_policy_plugin PRIVATE
  include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(rubi_unified_policy_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(rubi_unified_policy_plugin
  ament_index_cpp
  gazebo_ros
  geometry_msgs
  rclcpp
  rubi_control_core
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(rubi_unified_policy_plugin ${GAZEBO_LIBRARIES})
set_target_properties(rubi_unified_policy_plugin PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

target_compile_features(rubi_gazebo_legacy_policy_plugin PRIVATE cxx_std_17)
target_include_directories(rubi_gazebo_legacy_policy_plugin PRIVATE
  include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(rubi_gazebo_legacy_policy_plugin PRIVATE
  ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(rubi_gazebo_legacy_policy_plugin
  ament_index_cpp
  gazebo_ros
  geometry_msgs
  rclcpp
  rubi_control_core
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(rubi_gazebo_legacy_policy_plugin ${GAZEBO_LIBRARIES})
set_target_properties(rubi_gazebo_legacy_policy_plugin PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(velocity_input_arbitration_test
    test/velocity_input_arbitration_test.cpp
  )
  target_compile_features(velocity_input_arbitration_test PRIVATE cxx_std_17)
  target_include_directories(velocity_input_arbitration_test PRIVATE include)
endif()

install(TARGETS
  rubi_unified_policy_plugin
  rubi_gazebo_legacy_policy_plugin
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rubi_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(mid360_frame_transform
  src/mid360_frame_transform.cpp
)
target_compile_features(mid360_frame_transform PUBLIC cxx_std_17)
target_include_directories(mid360_frame_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mid360_frame_transform
  geometry_msgs
  livox_ros_driver2
  sensor_msgs
)

add_executable(mid360_frame_adapter_node
  src/mid360_frame_adapter_node.cpp
)
target_compile_features(mid360_frame_adapter_node PUBLIC cxx_std_17)
target_link_libraries(mid360_frame_adapter_node mid360_frame_transform)
ament_target_dependencies(mid360_frame_adapter_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/legacy_torque_off_guard.py
  scripts/mid360_imu_analyzer.py
  scripts/mid360_imu_axis_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mid360_frame_transform
  mid360_frame_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(mid360_frame_transform_test
    test/test_mid360_frame_transform.cpp
  )
  target_link_libraries(mid360_frame_transform_test mid360_frame_transform)
  ament_target_dependencies(mid360_frame_transform_test
    livox_ros_driver2
    sensor_msgs
  )
  ament_add_pytest_test(mid360_imu_static test/test_mid360_imu_static.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rubi_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

#ifndef RUBI_GAZEBO_SIM__MID360_FRAME_TRANSFORM_HPP_
#define RUBI_GAZEBO_SIM__MID360_FRAME_TRANSFORM_HPP_

#include <array>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace rubi_gazebo_sim
{

void rotate_vector_roll_180(geometry_msgs::msg::Vector3 & vector);

std::array<double, 9> rotate_covariance_roll_180(
  const std::array<double, 9> & covariance);

bool compose_orientation_roll_180(
  geometry_msgs::msg::Quaternion & orientation);

void transform_custom_msg(
  livox_ros_driver2::msg::CustomMsg & message,
  bool apply_roll_180,
  const std::string & output_frame_id);

bool transform_pointcloud2(
  sensor_msgs::msg::PointCloud2 & message,
  bool apply_roll_180,
  const std::string & output_frame_id,
  std::string & error);

bool transform_imu(
  sensor_msgs::msg::Imu & message,
  bool apply_roll_180,
  const std::string & output_frame_id);

}  // namespace rubi_gazebo_sim

#endif  // RUBI_GAZEBO_SIM__MID360_FRAME_TRANSFORM_HPP_

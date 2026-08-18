#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "gtest/gtest.h"
#include "rubi_gazebo_sim/mid360_frame_transform.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

constexpr double kTolerance = 1.0e-9;

sensor_msgs::msg::PointCloud2 make_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 34;
  cloud.header.frame_id = "livox_frame_raw";
  cloud.height = 1;
  cloud.width = 2;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
  *x = 1.0F;
  *y = 2.0F;
  *z = 3.0F;
  *r = 17U;
  ++x;
  ++y;
  ++z;
  ++r;
  *x = -4.0F;
  *y = -5.0F;
  *z = -6.0F;
  *r = 23U;
  return cloud;
}

TEST(Mid360FrameTransform, VectorIdentityWhenDisabled)
{
  geometry_msgs::msg::Vector3 vector;
  vector.x = 1.0;
  vector.y = 2.0;
  vector.z = 3.0;
  sensor_msgs::msg::Imu message;
  message.angular_velocity = vector;

  EXPECT_TRUE(
    rubi_gazebo_sim::transform_imu(
      message, false, "livox_frame_raw"));
  EXPECT_DOUBLE_EQ(message.angular_velocity.x, 1.0);
  EXPECT_DOUBLE_EQ(message.angular_velocity.y, 2.0);
  EXPECT_DOUBLE_EQ(message.angular_velocity.z, 3.0);
}

TEST(Mid360FrameTransform, Roll180VectorDoubleTransformAndNorm)
{
  geometry_msgs::msg::Vector3 vector;
  vector.x = 1.0;
  vector.y = 2.0;
  vector.z = 3.0;
  const double norm = std::sqrt(14.0);

  rubi_gazebo_sim::rotate_vector_roll_180(vector);
  EXPECT_DOUBLE_EQ(vector.x, 1.0);
  EXPECT_DOUBLE_EQ(vector.y, -2.0);
  EXPECT_DOUBLE_EQ(vector.z, -3.0);
  EXPECT_NEAR(
    std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z),
    norm, kTolerance);

  rubi_gazebo_sim::rotate_vector_roll_180(vector);
  EXPECT_DOUBLE_EQ(vector.x, 1.0);
  EXPECT_DOUBLE_EQ(vector.y, 2.0);
  EXPECT_DOUBLE_EQ(vector.z, 3.0);
}

TEST(Mid360FrameTransform, CustomMsgCoordinatesAndMetadata)
{
  livox_ros_driver2::msg::CustomMsg message;
  message.header.stamp.sec = 42;
  message.header.stamp.nanosec = 99;
  message.header.frame_id = "livox_frame_raw";
  message.timebase = 123456U;
  message.point_num = 1U;
  message.lidar_id = 7U;
  message.rsvd = {{1U, 2U, 3U}};
  message.points.resize(1);
  auto & point = message.points.front();
  point.offset_time = 888U;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.reflectivity = 44U;
  point.tag = 0x10U;
  point.line = 3U;

  rubi_gazebo_sim::transform_custom_msg(
    message, true, "livox_frame");

  EXPECT_EQ(message.header.stamp.sec, 42);
  EXPECT_EQ(message.header.stamp.nanosec, 99U);
  EXPECT_EQ(message.header.frame_id, "livox_frame");
  EXPECT_EQ(message.timebase, 123456U);
  EXPECT_EQ(message.point_num, 1U);
  EXPECT_EQ(message.lidar_id, 7U);
  EXPECT_EQ(message.rsvd, (std::array<std::uint8_t, 3>{{1U, 2U, 3U}}));
  EXPECT_FLOAT_EQ(point.x, 1.0F);
  EXPECT_FLOAT_EQ(point.y, -2.0F);
  EXPECT_FLOAT_EQ(point.z, -3.0F);
  EXPECT_EQ(point.offset_time, 888U);
  EXPECT_EQ(point.reflectivity, 44U);
  EXPECT_EQ(point.tag, 0x10U);
  EXPECT_EQ(point.line, 3U);
}

TEST(Mid360FrameTransform, PointCloud2CoordinatesFieldsAndCount)
{
  auto cloud = make_cloud();
  const auto stamp = cloud.header.stamp;
  const auto point_step = cloud.point_step;
  const auto row_step = cloud.row_step;
  std::string error;

  ASSERT_TRUE(
    rubi_gazebo_sim::transform_pointcloud2(
      cloud, true, "livox_frame", error)) << error;
  EXPECT_EQ(cloud.header.stamp, stamp);
  EXPECT_EQ(cloud.header.frame_id, "livox_frame");
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.point_step, point_step);
  EXPECT_EQ(cloud.row_step, row_step);

  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> r(cloud, "r");
  EXPECT_FLOAT_EQ(*x, 1.0F);
  EXPECT_FLOAT_EQ(*y, -2.0F);
  EXPECT_FLOAT_EQ(*z, -3.0F);
  EXPECT_EQ(*r, 17U);
  ++x;
  ++y;
  ++z;
  ++r;
  EXPECT_FLOAT_EQ(*x, -4.0F);
  EXPECT_FLOAT_EQ(*y, 5.0F);
  EXPECT_FLOAT_EQ(*z, 6.0F);
  EXPECT_EQ(*r, 23U);
}

TEST(Mid360FrameTransform, CovarianceUsesGeneralMatrixRule)
{
  const std::array<double, 9> input{{
    1.0, 2.0, 3.0,
    4.0, 5.0, 6.0,
    7.0, 8.0, 9.0}};
  const std::array<double, 9> expected{{
    1.0, -2.0, -3.0,
    -4.0, 5.0, 6.0,
    -7.0, 8.0, 9.0}};
  EXPECT_EQ(rubi_gazebo_sim::rotate_covariance_roll_180(input), expected);
}

TEST(Mid360FrameTransform, QuaternionCompositionAndNorm)
{
  geometry_msgs::msg::Quaternion raw_stationary;
  raw_stationary.x = 1.0;
  raw_stationary.w = 0.0;
  ASSERT_TRUE(rubi_gazebo_sim::compose_orientation_roll_180(raw_stationary));
  EXPECT_NEAR(raw_stationary.x, 0.0, kTolerance);
  EXPECT_NEAR(raw_stationary.y, 0.0, kTolerance);
  EXPECT_NEAR(raw_stationary.z, 0.0, kTolerance);
  EXPECT_NEAR(std::abs(raw_stationary.w), 1.0, kTolerance);

  const double half_yaw = 0.25;
  geometry_msgs::msg::Quaternion raw_yaw;
  raw_yaw.x = std::cos(half_yaw);
  raw_yaw.y = std::sin(half_yaw);
  raw_yaw.w = 0.0;
  ASSERT_TRUE(rubi_gazebo_sim::compose_orientation_roll_180(raw_yaw));
  const double norm = std::sqrt(
    raw_yaw.x * raw_yaw.x + raw_yaw.y * raw_yaw.y +
    raw_yaw.z * raw_yaw.z + raw_yaw.w * raw_yaw.w);
  EXPECT_NEAR(norm, 1.0, kTolerance);
  EXPECT_NEAR(std::abs(raw_yaw.z), std::sin(half_yaw), kTolerance);
  EXPECT_NEAR(std::abs(raw_yaw.w), std::cos(half_yaw), kTolerance);
  EXPECT_GT(raw_yaw.z * raw_yaw.w, 0.0);
}

TEST(Mid360FrameTransform, ImuTransformsAllCovariances)
{
  sensor_msgs::msg::Imu message;
  message.header.frame_id = "livox_frame_raw";
  message.orientation.x = 1.0;
  message.orientation.w = 0.0;
  message.angular_velocity.x = 1.0;
  message.angular_velocity.y = 2.0;
  message.angular_velocity.z = 3.0;
  message.linear_acceleration.x = 4.0;
  message.linear_acceleration.y = 5.0;
  message.linear_acceleration.z = 6.0;
  const std::array<double, 9> covariance{{
    1.0, 2.0, 3.0,
    4.0, 5.0, 6.0,
    7.0, 8.0, 9.0}};
  message.orientation_covariance = covariance;
  message.angular_velocity_covariance = covariance;
  message.linear_acceleration_covariance = covariance;

  ASSERT_TRUE(
    rubi_gazebo_sim::transform_imu(
      message, true, "livox_frame"));
  const auto expected = rubi_gazebo_sim::rotate_covariance_roll_180(covariance);
  EXPECT_EQ(message.orientation_covariance, expected);
  EXPECT_EQ(message.angular_velocity_covariance, expected);
  EXPECT_EQ(message.linear_acceleration_covariance, expected);
  EXPECT_DOUBLE_EQ(message.angular_velocity.y, -2.0);
  EXPECT_DOUBLE_EQ(message.angular_velocity.z, -3.0);
  EXPECT_DOUBLE_EQ(message.linear_acceleration.y, -5.0);
  EXPECT_DOUBLE_EQ(message.linear_acceleration.z, -6.0);
}

TEST(Mid360FrameTransform, UnknownOrientationContractIsPreserved)
{
  sensor_msgs::msg::Imu message;
  message.orientation.x = 0.1;
  message.orientation.y = 0.2;
  message.orientation.z = 0.3;
  message.orientation.w = 0.4;
  message.orientation_covariance[0] = -1.0;
  const auto orientation = message.orientation;

  EXPECT_TRUE(
    rubi_gazebo_sim::transform_imu(
      message, true, "livox_frame"));
  EXPECT_EQ(message.orientation, orientation);
  EXPECT_DOUBLE_EQ(message.orientation_covariance[0], -1.0);
}

}  // namespace

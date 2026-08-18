#include "rubi_gazebo_sim/mid360_frame_transform.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "sensor_msgs/msg/point_field.hpp"

namespace rubi_gazebo_sim
{
namespace
{

constexpr std::array<double, 3> kRoll180Signs{{1.0, -1.0, -1.0}};

bool host_is_big_endian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

template<typename Scalar>
Scalar read_scalar(const std::uint8_t * data, bool data_is_big_endian)
{
  std::array<std::uint8_t, sizeof(Scalar)> bytes{};
  std::copy(data, data + sizeof(Scalar), bytes.begin());
  if (data_is_big_endian != host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  Scalar value{};
  std::memcpy(&value, bytes.data(), sizeof(Scalar));
  return value;
}

template<typename Scalar>
void write_scalar(std::uint8_t * data, Scalar value, bool data_is_big_endian)
{
  std::array<std::uint8_t, sizeof(Scalar)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(Scalar));
  if (data_is_big_endian != host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::copy(bytes.begin(), bytes.end(), data);
}

std::size_t field_size(const sensor_msgs::msg::PointField & field)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return sizeof(float);
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return sizeof(double);
  }
  return 0;
}

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  const auto iterator = std::find_if(
    message.fields.begin(), message.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  return iterator == message.fields.end() ? nullptr : &(*iterator);
}

void negate_field(
  std::uint8_t * data,
  const sensor_msgs::msg::PointField & field,
  bool data_is_big_endian)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    const float value = read_scalar<float>(data, data_is_big_endian);
    write_scalar<float>(data, -value, data_is_big_endian);
  } else {
    const double value = read_scalar<double>(data, data_is_big_endian);
    write_scalar<double>(data, -value, data_is_big_endian);
  }
}

}  // namespace

void rotate_vector_roll_180(geometry_msgs::msg::Vector3 & vector)
{
  vector.y = -vector.y;
  vector.z = -vector.z;
}

std::array<double, 9> rotate_covariance_roll_180(
  const std::array<double, 9> & covariance)
{
  std::array<double, 9> result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row * 3 + column] =
        kRoll180Signs[row] * covariance[row * 3 + column] *
        kRoll180Signs[column];
    }
  }
  return result;
}

bool compose_orientation_roll_180(
  geometry_msgs::msg::Quaternion & orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;

  // q_out = q_world_raw * inverse(q_corrected_raw).
  // For Rx(pi), inverse(q) is rotation-equivalent to q=(1,0,0,0).
  orientation.x = w;
  orientation.y = z;
  orientation.z = -y;
  orientation.w = -x;
  return true;
}

void transform_custom_msg(
  livox_ros_driver2::msg::CustomMsg & message,
  bool apply_roll_180,
  const std::string & output_frame_id)
{
  if (apply_roll_180) {
    for (auto & point : message.points) {
      point.y = -point.y;
      point.z = -point.z;
    }
  }
  if (!output_frame_id.empty()) {
    message.header.frame_id = output_frame_id;
  }
}

bool transform_pointcloud2(
  sensor_msgs::msg::PointCloud2 & message,
  bool apply_roll_180,
  const std::string & output_frame_id,
  std::string & error)
{
  const auto * x_field = find_field(message, "x");
  const auto * y_field = find_field(message, "y");
  const auto * z_field = find_field(message, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    error = "PointCloud2 requires x, y, and z fields";
    return false;
  }

  for (const auto * field : {x_field, y_field, z_field}) {
    const std::size_t size = field_size(*field);
    if (size == 0) {
      error = "PointCloud2 x/y/z fields must be FLOAT32 or FLOAT64";
      return false;
    }
    if (field->count != 1 || field->offset + size > message.point_step) {
      error = "PointCloud2 x/y/z field layout is invalid";
      return false;
    }
  }

  const std::size_t required_size =
    static_cast<std::size_t>(message.row_step) * message.height;
  if (required_size > message.data.size()) {
    error = "PointCloud2 data is smaller than row_step * height";
    return false;
  }

  if (apply_roll_180) {
    for (std::uint32_t row = 0; row < message.height; ++row) {
      for (std::uint32_t column = 0; column < message.width; ++column) {
        std::uint8_t * point = message.data.data() +
          static_cast<std::size_t>(row) * message.row_step +
          static_cast<std::size_t>(column) * message.point_step;
        negate_field(point + y_field->offset, *y_field, message.is_bigendian);
        negate_field(point + z_field->offset, *z_field, message.is_bigendian);
      }
    }
  }

  if (!output_frame_id.empty()) {
    message.header.frame_id = output_frame_id;
  }
  error.clear();
  return true;
}

bool transform_imu(
  sensor_msgs::msg::Imu & message,
  bool apply_roll_180,
  const std::string & output_frame_id)
{
  bool orientation_valid = true;
  if (apply_roll_180) {
    rotate_vector_roll_180(message.angular_velocity);
    rotate_vector_roll_180(message.linear_acceleration);
    message.angular_velocity_covariance = rotate_covariance_roll_180(
      message.angular_velocity_covariance);
    message.linear_acceleration_covariance = rotate_covariance_roll_180(
      message.linear_acceleration_covariance);

    if (message.orientation_covariance[0] != -1.0) {
      orientation_valid = compose_orientation_roll_180(message.orientation);
      message.orientation_covariance = rotate_covariance_roll_180(
        message.orientation_covariance);
    }
  }
  if (!output_frame_id.empty()) {
    message.header.frame_id = output_frame_id;
  }
  return orientation_valid;
}

}  // namespace rubi_gazebo_sim

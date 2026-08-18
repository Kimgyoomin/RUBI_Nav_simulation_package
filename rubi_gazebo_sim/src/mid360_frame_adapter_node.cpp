#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rubi_gazebo_sim/mid360_frame_transform.hpp"

namespace rubi_gazebo_sim
{

class Mid360FrameAdapter : public rclcpp::Node
{
public:
  Mid360FrameAdapter()
  : Node("mid360_frame_adapter")
  {
    apply_roll_180_ = declare_parameter<bool>("apply_roll_180", true);
    output_frame_id_ = declare_parameter<std::string>(
      "output_frame_id", "livox_frame");
    const auto input_lidar_topic = declare_parameter<std::string>(
      "input_lidar_topic", "/livox/lidar_raw");
    const auto output_lidar_topic = declare_parameter<std::string>(
      "output_lidar_topic", "/livox/lidar");
    const auto input_pointcloud2_topic = declare_parameter<std::string>(
      "input_pointcloud2_topic", "/livox/lidar_raw_PointCloud2");
    const auto output_pointcloud2_topic = declare_parameter<std::string>(
      "output_pointcloud2_topic", "/livox/lidar_PointCloud2");
    const auto input_imu_topic = declare_parameter<std::string>(
      "input_imu_topic", "/livox/imu_raw");
    const auto output_imu_topic = declare_parameter<std::string>(
      "output_imu_topic", "/livox/imu");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    custom_publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_lidar_topic, qos);
    pointcloud2_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_pointcloud2_topic, qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_imu_topic, qos);

    custom_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_lidar_topic, qos,
      std::bind(&Mid360FrameAdapter::on_custom, this, std::placeholders::_1));
    pointcloud2_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_pointcloud2_topic, qos,
      std::bind(&Mid360FrameAdapter::on_pointcloud2, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic, qos,
      std::bind(&Mid360FrameAdapter::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MID360_FRAME_ADAPTER=READY apply_roll_180=%s raw=(%s,%s,%s) "
      "corrected=(%s,%s,%s) frame=%s",
      apply_roll_180_ ? "true" : "false",
      input_lidar_topic.c_str(), input_pointcloud2_topic.c_str(),
      input_imu_topic.c_str(), output_lidar_topic.c_str(),
      output_pointcloud2_topic.c_str(), output_imu_topic.c_str(),
      output_frame_id_.c_str());
  }

private:
  void on_custom(const livox_ros_driver2::msg::CustomMsg::SharedPtr input)
  {
    auto output = *input;
    transform_custom_msg(output, apply_roll_180_, output_frame_id_);
    custom_publisher_->publish(output);
  }

  void on_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    auto output = *input;
    std::string error;
    if (!transform_pointcloud2(
        output, apply_roll_180_, output_frame_id_, error))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 correction rejected: %s", error.c_str());
      return;
    }
    pointcloud2_publisher_->publish(output);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr input)
  {
    auto output = *input;
    if (!transform_imu(output, apply_roll_180_, output_frame_id_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU orientation is invalid; vectors/covariances were corrected "
        "without manufacturing a valid orientation");
    }
    imu_publisher_->publish(output);
  }

  bool apply_roll_180_{true};
  std::string output_frame_id_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace rubi_gazebo_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rubi_gazebo_sim::Mid360FrameAdapter>());
  rclcpp::shutdown();
  return 0;
}

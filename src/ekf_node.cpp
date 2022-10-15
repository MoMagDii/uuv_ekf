
#include "ekf_node.hpp"
#include <iostream>
#include <memory>

using std::placeholders::_1;
// ================================================================
EkfNode::EkfNode()
: Node("ekf_node")
{
  uuv_ = std::make_shared<UUV>();
  uuv_->init_clock(this->get_clock());
  sensor_processor_ = std::make_shared<SensorProcessor>();
  sensor_processor_->init_clock(this->get_clock());

  // Subscribers initialization
  wrench_sub_ =
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/swift/wrench", rclcpp::QoS(10),
    std::bind(&UUV::wrenchCallback, uuv_, _1));
  dvl_sub_ =
    this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/swift/dvl_twist", rclcpp::SensorDataQoS(),
    std::bind(&SensorProcessor::dvlCallback, sensor_processor_, _1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/swift/imu", rclcpp::SensorDataQoS(),
    std::bind(&SensorProcessor::imuCallback, sensor_processor_, _1));
  pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "/swift/pressure", rclcpp::SensorDataQoS(),
    std::bind(&SensorProcessor::pressureCallback, sensor_processor_, _1));
  // Publishers initialization
  filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/swift/odom",
      rclcpp::QoS(10));
}
// ================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

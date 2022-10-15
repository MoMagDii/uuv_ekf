#ifndef EKF__NODE_HPP_
#define EKF__NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_processor.hpp"
#include "uuv_model.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class EkfNode : public rclcpp::Node
{
public:
  EkfNode();
private:
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  std::shared_ptr<SensorProcessor> sensor_processor_;
  std::shared_ptr<UUV> uuv_;

};
#endif  // EKF__NODE_HPP_

// Copyright 2022 MoMagDii
#include "sensor_processor.hpp"
#include <iostream>
#include <memory>
using std::placeholders::_1;

SensorProcessor::SensorProcessor(){}
SensorProcessor::~SensorProcessor(){}
// ===================================================================
void SensorProcessor::dvlCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Velocity vector in the dvl frame
  Eigen::Vector3d v_dvl;
  tf2::fromMsg(msg->twist.twist.linear, v_dvl);
  // Rotation of the velocity vector v_dvl from dvl frame to body frame
  body_linear_velocity_ = dvl_rotation_.toRotationMatrix() * v_dvl;
  // DVL translation compensation
  body_linear_velocity_ += body_angular_velocity_.cross(dvl_translation_);
 // msg->twist.covariance;
}
// ===================================================================
void SensorProcessor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Angular velocity in imu frame
  Eigen::Vector3d w_imu;
  tf2::fromMsg(msg->angular_velocity, w_imu);
  // rotating the vector w_imu by imu_rotation_ quaternion
  body_angular_velocity_ = imu_rotation_.toRotationMatrix() * w_imu;
}
// ===================================================================
void SensorProcessor::pressureCallback(
  const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  // compute the gauge pressure in Pa, atmospheric pressure is 101.325 kPa
  float Pgauge = (msg->fluid_pressure) - (101.325 * 1e+3);
  // depth in meter
  float h = Pgauge / (1025 * 9.806);
  // update odom z position, ENU
  double z = -1 * h - pressure_translation_(2);
  double z_cov = msg->variance * msg->variance;
}
// ====================================================================
void SensorProcessor::init_clock(const rclcpp::Clock::SharedPtr rclcpp_clock)
{
    rclcpp_clock_ = rclcpp_clock;
    lookup_tf2Buffer();
}
// =====================================================================
void SensorProcessor::lookup_tf2Buffer()
{
    // TF2 buffer stores known frames and offers services for the relationships
    // between frames
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(rclcpp_clock_);
    // Transform listener to request and receive coordinate frame transform
    // information over a tf2 buffer
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // *********************************************
    try {
      // Look up for the transformation between a source frame and target frame
      // tf2::TimePointZero means the latest available transform in the buffer
      // 15 seconds timeout parameter, waiting for 15s until a transform becomes
      // available
      geometry_msgs::msg::TransformStamped dvl_transform;
      dvl_transform = tf_buffer_->lookupTransform(
        "swift/base_link", "swift/dvl_link", tf2::TimePointZero,
        tf2::durationFromSec(15));
      // Extracting  sensor cordinate translation and rotation in quaternion
      tf2::fromMsg(dvl_transform.transform.rotation, dvl_rotation_);
      tf2::fromMsg(dvl_transform.transform.translation, dvl_translation_);
    } catch (tf2::TransformException & ex) {
      std::cout<<  "Could not transform dvl_link static frame: %s" << std::endl;
      std::cout<< ex.what() << std::endl;
    }
    // *********************************************
    try {
      // Look up for the transformation between base_link and imu_link
      geometry_msgs::msg::TransformStamped imu_transform;
      imu_transform = tf_buffer_->lookupTransform(
        "swift/base_link", "swift/imu_link", tf2::TimePointZero,
        tf2::durationFromSec(15));
      tf2::fromMsg(imu_transform.transform.rotation, imu_rotation_);
      tf2::fromMsg(imu_transform.transform.translation, imu_translation_);
    } catch (tf2::TransformException & ex) {
      std::cout<<  "Could not transform imu_link static frame: %s" << std::endl;
      std::cout<< ex.what() << std::endl;
    }
    /// *********************************************
    try {
      // Look up for the transformation between base_link and pressure_link
      geometry_msgs::msg::TransformStamped pressure_transform;
      pressure_transform = tf_buffer_->lookupTransform(
        "swift/base_link", "swift/pressure_link", tf2::TimePointZero,
        tf2::durationFromSec(15));
      tf2::fromMsg(pressure_transform.transform.translation,
        pressure_translation_);
    } catch (tf2::TransformException & ex) {
      std::cout<<  "Could not transform pressure_link static frame: %s" << std::endl;
      std::cout<< ex.what() << std::endl;
    }
}
// ===================================================================

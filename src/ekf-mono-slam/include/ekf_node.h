#pragma once

#include "ekf_mono_slam/msg/image_feature_measurements.hpp"
#include "filter/ekf.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class EKFNode : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() = default;

 private:
  rclcpp::Subscription<ekf_mono_slam::msg::ImageFeatureMeasurements>::SharedPtr measurements_subscriber_;
  EKF ekf_;

  void step_callback(const ekf_mono_slam::msg::ImageFeatureMeasurements::ConstSharedPtr& msg);
};

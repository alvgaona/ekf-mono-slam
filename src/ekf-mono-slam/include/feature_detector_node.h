#pragma once

#include "ekf_mono_slam/srv/feature_detector.hpp"
#include "filter/ekf.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class FeatureDetectorNode : public rclcpp::Node {
 public:
  FeatureDetectorNode();
  ~FeatureDetectorNode() = default;

 private:
  rclcpp::Service<ekf_mono_slam::srv::FeatureDetector>::SharedPtr service_;

  void detect_callback(const std::shared_ptr<ekf_mono_slam::srv::FeatureDetector::Request> request,
                       std::shared_ptr<ekf_mono_slam::srv::FeatureDetector::Response> response);
};

#pragma once

#include "ekf_mono_slam/msg/image_feature_measurement_array.hpp"
#include "ekf_mono_slam/srv/feature_detect.hpp"
#include "rclcpp/rclcpp.hpp"

class FeatureDetectorNode : public rclcpp::Node {
 public:
  FeatureDetectorNode();
  ~FeatureDetectorNode() override = default;

 private:
  rclcpp::Service<ekf_mono_slam::srv::FeatureDetect>::SharedPtr detect_service_;

  rclcpp::Publisher<ekf_mono_slam::msg::ImageFeatureMeasurementArray>::SharedPtr
    image_measurements_publisher_;

  void detect_features(
    const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Request>& request,
    const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Response>& response
  );
};

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include "ekf_mono_slam/msg/image_feature_measurement_array.hpp"
#include "ekf_mono_slam/srv/feature_detect.hpp"

class FeatureDetectorNode : public rclcpp::Node {
 public:
  FeatureDetectorNode();
  ~FeatureDetectorNode() override = default;

 private:
  rclcpp::Service<ekf_mono_slam::srv::FeatureDetect>::SharedPtr detect_service_;

  rclcpp::Publisher<ekf_mono_slam::msg::ImageFeatureMeasurementArray>::SharedPtr
    image_measurements_publisher_;

  std::shared_ptr<rerun::RecordingStream> rec_;

  void detect_features(
    const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Request>& request,
    const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Response>& response
  );
};

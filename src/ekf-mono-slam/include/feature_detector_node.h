#pragma once

#include "ekf_mono_slam/msg/image_feature_measurements.hpp"
#include "filter/ekf.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class FeatureDetectorNode : public rclcpp::Node {
 public:
  FeatureDetectorNode();
  ~FeatureDetectorNode() = default;

 private:
  std::shared_ptr<image_transport::Subscriber> image_subscriber_;
  rclcpp::Publisher<ekf_mono_slam::msg::ImageFeatureMeasurements>::SharedPtr image_measurements_publisher_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
};

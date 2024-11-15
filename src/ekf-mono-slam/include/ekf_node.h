#pragma once

#include "ekf_mono_slam/msg/image_feature_measurement_array.hpp"
#include "ekf_mono_slam/msg/state.hpp"
#include "ekf_mono_slam/srv/feature_detect.hpp"
#include "filter/ekf.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class EKFNode final : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() override = default;

 private:
  EKF ekf_;
  rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedPtr
      feature_detect_client_;
  std::shared_ptr<image_transport::Subscriber> image_subscriber_;
  rclcpp::Publisher<ekf_mono_slam::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<ekf_mono_slam::msg::CovarianceMatrix>::SharedPtr
      covariance_publisher_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void init_callback(
      rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedFuture future
  );
};

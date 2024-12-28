#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

#include "ekf_mono_slam/msg/covariance_matrix.hpp"
#include "ekf_mono_slam/msg/image_feature_measurement_array.hpp"
#include "ekf_mono_slam/msg/state.hpp"
#include "filter/ekf.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

class EKFNode final : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() override = default;

 private:
  EKF filter_;
  std::shared_ptr<image_transport::Subscriber> image_subscriber_;
  rclcpp::Publisher<ekf_mono_slam::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<ekf_mono_slam::msg::CovarianceMatrix>::SharedPtr
    covariance_publisher_;

  std::shared_ptr<rerun::RecordingStream> rec_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
};

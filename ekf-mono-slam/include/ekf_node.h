#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ekf_mono_slam/msg/covariance_matrix.hpp"
#include "ekf_mono_slam/msg/state.hpp"
#include "filter/ekf.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

class EKFNode final : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() override = default;

 private:
  std::unique_ptr<EKF> ekf_;
  std::shared_ptr<image_transport::Subscriber> image_subscriber_;
  rclcpp::Publisher<ekf_mono_slam::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<ekf_mono_slam::msg::CovarianceMatrix>::SharedPtr
    covariance_publisher_;

  SlamConfig load_slam_config();
  void declare_parameters();
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void publish_state();
};

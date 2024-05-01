#pragma once

#include "filter/ekf.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class EKFNode : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() = default;

 private:
  std::shared_ptr<image_transport::Subscriber> image_subscriber_;
  EKF ekf_;

  void step_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
};

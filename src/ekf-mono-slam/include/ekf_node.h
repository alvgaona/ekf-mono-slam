#ifndef EKF_NODE_MONO_SLAM_EKF_H
#define EKF_NODE_MONO_SLAM_EKF_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class EKFNode : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() = default;

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  void step_callback(sensor_msgs::msg::Image::SharedPtr msg);
};

#endif /* EKF_NODE_MONO_SLAM_EKF_H */

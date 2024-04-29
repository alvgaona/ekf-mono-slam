#ifndef EKF_NODE_MONO_SLAM_EKF_H
#define EKF_NODE_MONO_SLAM_EKF_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EKFNode : public rclcpp::Node {
 public:
  EKFNode();
  ~EKFNode() = default;

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void step_callback(std_msgs::msg::String::UniquePtr msg);
};

#endif /* EKF_NODE_MONO_SLAM_EKF_H */

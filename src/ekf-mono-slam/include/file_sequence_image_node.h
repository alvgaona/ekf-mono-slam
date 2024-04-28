#ifndef FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H
#define FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class FileSequenceImageNode : public rclcpp::Node {
 public:
  FileSequenceImageNode();
  ~FileSequenceImageNode() = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  void timer_callback();
};

#endif /* FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H */

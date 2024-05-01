#ifndef FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H
#define FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H

#include "image/file_sequence_image_provider.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class FileSequenceImageNode : public rclcpp::Node {
 public:
  FileSequenceImageNode();
  ~FileSequenceImageNode() = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::unique_ptr<FileSequenceImageProvider> image_provider_;

  void timer_callback();
};

#endif /* FILE_SEQUENCE_IMAGE_NODE_MONO_SLAM_EKF_H */

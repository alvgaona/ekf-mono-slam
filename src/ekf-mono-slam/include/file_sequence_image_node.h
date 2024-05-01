#pragma once

#include "image/file_sequence_image_provider.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class FileSequenceImageNode : public rclcpp::Node {
 public:
  FileSequenceImageNode();
  ~FileSequenceImageNode() = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<image_transport::Publisher> image_publisher_;
  std::unique_ptr<FileSequenceImageProvider> image_provider_;

  void timer_callback();
};

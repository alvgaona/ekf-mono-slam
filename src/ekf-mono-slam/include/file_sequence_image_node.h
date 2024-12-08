#pragma once

#include <rclcpp/rclcpp.hpp>

#include "image/file_sequence_image_provider.h"
#include "image_transport/image_transport.hpp"

class FileSequenceImageNode : public rclcpp::Node {
 public:
  FileSequenceImageNode();
  ~FileSequenceImageNode() override = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<image_transport::Publisher> image_publisher_;
  std::unique_ptr<FileSequenceImageProvider> image_provider_;

  void timer_callback() const;
};

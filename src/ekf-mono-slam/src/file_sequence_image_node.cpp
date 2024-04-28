#include "file_sequence_image_node.h"

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

FileSequenceImageNode::FileSequenceImageNode() : Node("file_sequence_image"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("slam/ekf/step, 10");
  timer_ = this->create_wall_timer(500ms, std::bind(&FileSequenceImageNode::timer_callback, this));
}

void FileSequenceImageNode::timer_callback() {
  Eigen::Vector2i v(1, 2);
  Eigen::Vector2i w(1, 2);
  auto message = std_msgs::msg::String();
  message.data = "The result is " + std::to_string(v.dot(w));
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FileSequenceImageNode>());
  rclcpp::shutdown();
  return 0;
}

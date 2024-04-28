#include "ekf_node.h"

#include <memory>

EKFNode::EKFNode() : Node("ekf_node") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "slam/ekf/step", 10, std::bind(&EKFNode::step_callback, this, std::placeholders::_1));
}

void EKFNode::step_callback(std_msgs::msg::String::UniquePtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

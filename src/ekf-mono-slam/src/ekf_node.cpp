#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"

EKFNode::EKFNode() : Node("ekf_node") {
  measurements_subscriber_ = this->create_subscription<ekf_mono_slam::msg::ImageFeatureMeasurements>(
      "features/image/measurements", 10, std::bind(&EKFNode::step_callback, this, std::placeholders::_1));
}

void EKFNode::step_callback(const ekf_mono_slam::msg::ImageFeatureMeasurements::ConstSharedPtr& msg) {}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

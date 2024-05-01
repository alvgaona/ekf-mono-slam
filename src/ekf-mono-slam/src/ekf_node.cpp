#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"

EKFNode::EKFNode() : Node("ekf_node") {
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image", 10, std::bind(&EKFNode::step_callback, this, std::placeholders::_1));
}

void EKFNode::step_callback(sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;
  RCLCPP_INFO(this->get_logger(), "Image size '%d'", image.size().width);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

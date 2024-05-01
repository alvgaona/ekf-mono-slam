#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"

EKFNode::EKFNode() : Node("ekf_node") {
  image_subscriber_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "camera/image", std::bind(&EKFNode::step_callback, this, std::placeholders::_1), "raw"));
}

void EKFNode::step_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  // if (ekf_.GetFeatureDetector() == nullptr) {
  //   ekf_.Init(image);
  // }

  // ekf_.Step(image);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

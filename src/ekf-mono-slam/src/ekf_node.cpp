#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"

EKFNode::EKFNode() : Node("ekf_node") {
  image_subscriber_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "camera/image", std::bind(&EKFNode::step_callback, this, std::placeholders::_1), "raw"));
  detect_client_ = this->create_client<ekf_mono_slam::srv::FeatureDetector>("feature_detect");
}

void EKFNode::step_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  auto request = std::make_shared<ekf_mono_slam::srv::FeatureDetector::Request>();
  request->image = *msg;

  auto future_response = detect_client_->async_send_request(request);

  auto status = future_response.wait_for(std::chrono::seconds(5));

  if (status == std::future_status::ready) {
    auto response = future_response.get();
    RCLCPP_INFO(this->get_logger(), "Total number of features: '%d'", response->features.size());
  }

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

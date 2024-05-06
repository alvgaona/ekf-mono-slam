#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "feature/image_feature_measurement.h"

EKFNode::EKFNode() : Node("ekf_node") {
  image_subscriber_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "camera/image", std::bind(&EKFNode::image_callback, this, std::placeholders::_1), "raw"));
  feature_detect_client_ = this->create_client<ekf_mono_slam::srv::FeatureDetect>("features/detect");
  state_publisher_ = this->create_publisher<ekf_mono_slam::msg::State>("filter/state", 10);
  covariance_publisher_ = this->create_publisher<ekf_mono_slam::msg::CovarianceMatrix>("filter/covariance", 10);
}

void EKFNode::init_callback(rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedFuture future) {
  auto status = future.wait_for(std::chrono::milliseconds(1000));

  if (status == std::future_status::ready) {
    auto response = future.get();

    std::vector<std::shared_ptr<ImageFeatureMeasurement>> features;

    for (auto im_feat : response->features) {
      const auto descriptor_size = im_feat.descriptor.size();
      auto descriptor = cv::Mat(1, descriptor_size, CV_8UC1, im_feat.descriptor.data());

      // TODO: do I need the feature index at this point or later?
      features.push_back(
          std::make_shared<ImageFeatureMeasurement>(cv::Point2f(im_feat.point.x, im_feat.point.y), descriptor));
    }

    std::cout << features.at(0) << std::endl;

    ekf_.AddFeatures(features);
  }

  ekf_mono_slam::msg::State state_msg;
  state_msg.header.frame_id = "base_link";
  state_msg.header.stamp = this->get_clock()->now();
  state_msg.dimension = ekf_.GetState()->GetDimension();

  state_publisher_->publish(state_msg);

  // TODO: publish covariance matrix
}

void EKFNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!ekf_.isInitilized()) {
    auto request = std::make_shared<ekf_mono_slam::srv::FeatureDetect::Request>();
    request->image = *msg;

    feature_detect_client_->async_send_request(request,
                                               std::bind(&EKFNode::init_callback, this, std::placeholders::_1));
  } else {
    ekf_.Predict();

    ekf_mono_slam::msg::State state_msg;
    state_msg.header.frame_id = "base_link";
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.pose.position.x = ekf_.GetState()->GetPosition()[0];

    state_publisher_->publish(state_msg);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

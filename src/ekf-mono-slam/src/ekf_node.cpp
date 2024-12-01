#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "feature/image_feature_measurement.h"

EKFNode::EKFNode() : Node("ekf_node") {
  image_subscriber_ = std::make_shared<image_transport::Subscriber>(
    image_transport::create_subscription(
      this,
      "camera/image",
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        this->image_callback(msg);
      },
      "raw"
    )
  );
  feature_detect_client_ =
    this->create_client<ekf_mono_slam::srv::FeatureDetect>("features/detect");
  state_publisher_ =
    this->create_publisher<ekf_mono_slam::msg::State>("filter/state", 10);
  covariance_publisher_ =
    this->create_publisher<ekf_mono_slam::msg::CovarianceMatrix>(
      "filter/covariance", 10
    );
}

void EKFNode::init_callback(
  const rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedFuture& future
) {
  if (auto status = future.wait_for(std::chrono::milliseconds(1000));
      status == std::future_status::ready) {
    const auto& response = future.get();

    std::vector<std::shared_ptr<ImageFeatureMeasurement>> features;

    for (auto im_feat : response->features) {
      const auto descriptor_size = static_cast<int>(im_feat.descriptor.size());
      auto descriptor =
        cv::Mat(1, descriptor_size, CV_8UC1, im_feat.descriptor.data());

      features.push_back(std::make_shared<ImageFeatureMeasurement>(
        cv::Point2f(im_feat.point.x, im_feat.point.y), descriptor
      ));
    }

    ekf_.add_features(features);
  }

  ekf_mono_slam::msg::State state_msg;
  state_msg.header.frame_id = "base_link";
  state_msg.header.stamp = this->get_clock()->now();
  state_msg.dimension = ekf_.state()->dimension();

  state_publisher_->publish(state_msg);

  // TODO: publish covariance matrix
}

void EKFNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg
) {
  if (!ekf_.is_initilized()) {
    auto request =
      std::make_shared<ekf_mono_slam::srv::FeatureDetect::Request>();
    request->image = *msg;

    feature_detect_client_->async_send_request(
      request,
      [this](rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedFuture
               future  // NOLINT
      ) { this->init_callback(future); }
    );
  } else {
    ekf_.predict();

    ekf_mono_slam::msg::State state_msg;
    state_msg.header.frame_id = "base_link";
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.pose.position.x = ekf_.state()->position()[0];

    state_publisher_->publish(state_msg);
  }
}

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

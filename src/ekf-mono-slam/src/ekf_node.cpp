#include "ekf_node.h"

#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <memory>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>

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

  rec_ = std::make_shared<rerun::RecordingStream>("my_app", "default");
  rec_->spawn().exit_on_failure();

  rec_->log_file_from_path(std::filesystem::path("rerun/my_app.rbl"));
}

void EKFNode::init_callback(
  const rclcpp::Client<ekf_mono_slam::srv::FeatureDetect>::SharedFuture& future
) {
  if (auto status = future.wait_for(std::chrono::milliseconds(1000));
      status == std::future_status::ready) {
    const auto& response = future.get();

    std::vector<std::shared_ptr<ImageFeatureMeasurement>> features;

    for (size_t i = 0; i < response->features.size(); i++) {
      const auto& im_feat = response->features[i];
      const auto descriptor_size = static_cast<int>(im_feat.descriptor.size());
      cv::Mat descriptor = cv::Mat(1, descriptor_size, CV_8UC1);
      std::memcpy(descriptor.data, im_feat.descriptor.data(), descriptor_size);

      features.push_back(std::make_shared<ImageFeatureMeasurement>(
        cv::Point2f(im_feat.point.x, im_feat.point.y), descriptor, i
      ));
    }

    ekf_.add_features(features);
  }
}

void EKFNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg
) {
  if (!ekf_.is_initilized()) {
    rec_->log(
      "/",
      rerun::TextLog("EKF is not initialized. Initializing...")
        .with_level(rerun::TextLogLevel::Info)
    );

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
    const cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    const cv::Mat image = cv_ptr->image;

    rec_->log("camera/state/x", rerun::Scalar(ekf_.state()->position()[0]));
    rec_->log("camera/state/y", rerun::Scalar(ekf_.state()->position()[1]));
    rec_->log("camera/state/z", rerun::Scalar(ekf_.state()->position()[2]));

    ekf_.predict();
    ekf_.match_predicted_features(image);

    // TODO: continue implementing
  }
}

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

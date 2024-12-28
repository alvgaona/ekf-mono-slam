#include "ekf_node.h"

#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <memory>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>

#include "feature/feature_detector.h"
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

void EKFNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg
) {
  const cv_bridge::CvImagePtr cv_ptr =
    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  const cv::Mat image = cv_ptr->image;

  FeatureDetector detector(
    FeatureDetector::Type::BRISK, cv::Size(image.rows, image.cols)
  );

  if (!filter_.is_initilized()) {
    detector.detect_features(image);
    filter_.add_features(detector.image_features());
  } else {
    rec_->log("camera/state/x", rerun::Scalar(filter_.state()->position()[0]));
    rec_->log("camera/state/y", rerun::Scalar(filter_.state()->position()[1]));
    rec_->log("camera/state/z", rerun::Scalar(filter_.state()->position()[2]));

    filter_.predict();

    // TODO: continue implementing
  }
}

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

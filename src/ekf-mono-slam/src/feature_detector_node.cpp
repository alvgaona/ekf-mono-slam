#include "feature_detector_node.h"

#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <rerun.hpp>
#include <rerun/archetypes/points2d.hpp>
#include <rerun/recording_stream.hpp>

#include "collection_adapters.h"
#include "feature/feature_detector.h"

FeatureDetectorNode::FeatureDetectorNode() : Node("feature_detector") {
  detect_service_ = this->create_service<ekf_mono_slam::srv::FeatureDetect>(
    "features/detect",
    [this](
      const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Request>&
        request,
      const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Response>&
        response
    ) { detect_features(request, response); }
  );
  image_measurements_publisher_ =
    this->create_publisher<ekf_mono_slam::msg::ImageFeatureMeasurementArray>(
      "features/image/measurements", 10
    );

  rec_ = std::make_shared<rerun::RecordingStream>("my_app", "default");
  rec_->spawn().exit_on_failure();

  rec_->log_file_from_path(std::filesystem::path("rerun/my_app.rbl"));
}

void FeatureDetectorNode::detect_features(
  const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Request>& request,
  const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Response>& response
) {
  const cv_bridge::CvImagePtr cv_ptr =
    cv_bridge::toCvCopy(request->image, sensor_msgs::image_encodings::BGR8);
  const cv::Mat image = cv_ptr->image;

  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions;

  // TODO(alvgaona): make sure this loop is completed when passing predictions
  // to the feature detector
  for (size_t i = 0; i < request->predictions.size(); i++) {
    // cv::Mat(1, descriptor_size, CV_8UC1,
    // request->predictions[i].covariance_matrix.data());
    // TODO(alvgaona): create the image feature prediction from image with all
    // its values
    predictions.push_back(std::make_shared<ImageFeaturePrediction>(
      cv::Point2f(
        request->predictions[i].point.x, request->predictions[i].point.y
      ),
      i
    ));
  }

  FeatureDetector feature_detector(
    FeatureDetector::build_detector(DetectorType::AKAZE),
    FeatureDetector::build_descriptor_extractor(DescriptorExtractorType::AKAZE),
    cv::Size(image.rows, image.cols)
  );

  feature_detector.detect_features(image, predictions);

  auto detected_image_features = feature_detector.image_features();

  rec_->log(
    "/",
    rerun::TextLog(
      "Detected " + std::to_string(detected_image_features.size()) + " features"
    )
      .with_level(rerun::TextLogLevel::Debug)
  );

  std::vector<rerun::Position2D> points;
  for (auto& feature : detected_image_features) {
    auto coordinates = feature->coordinates();
    points.emplace_back(coordinates.x, coordinates.y);
  }

  const uint32_t width = image.cols;
  const uint32_t height = image.rows;

  rec_->log("image", rerun::Image::from_rgb24(image, {width, height}));
  rec_->log("image/keypoints", rerun::Points2D(points));

  ekf_mono_slam::msg::ImageFeatureMeasurementArray image_feature_measurements;
  std::vector<ekf_mono_slam::msg::ImageFeatureMeasurement> response_features;

  const auto image_features = feature_detector.image_features();
  for (const auto& m : image_features) {
    ekf_mono_slam::msg::ImageFeatureMeasurement feature;
    auto coordinates = m->coordinates();
    auto descriptor = m->descriptor_data();

    feature.point.x = coordinates.x;
    feature.point.y = coordinates.y;

    descriptor.copyTo(feature.descriptor);
    response_features.push_back(feature);
    image_feature_measurements.features.push_back(feature);
  }

  image_feature_measurements.header.frame_id = "image_link";
  image_feature_measurements.header.stamp = this->get_clock()->now();

  response->features = response_features;
  image_measurements_publisher_->publish(image_feature_measurements);
}

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

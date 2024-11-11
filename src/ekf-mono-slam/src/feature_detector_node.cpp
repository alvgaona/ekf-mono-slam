#include "feature_detector_node.h"

#include "cv_bridge/cv_bridge.h"
#include "feature/feature_detector.h"
#include "spdlog/spdlog.h"

FeatureDetectorNode::FeatureDetectorNode() : Node("feature_detector") {
  detect_service_ = this->create_service<ekf_mono_slam::srv::FeatureDetect>(
      "features/detect",
      std::bind(
          &FeatureDetectorNode::detect_features,
          this,
          std::placeholders::_1,
          std::placeholders::_2
      )
  );
  image_measurements_publisher_ =
      this->create_publisher<ekf_mono_slam::msg::ImageFeatureMeasurementArray>(
          "features/image/measurements", 10
      );
}

void FeatureDetectorNode::detect_features(
    const std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Request> request,
    std::shared_ptr<ekf_mono_slam::srv::FeatureDetect::Response> response
) {
  const cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(request->image, sensor_msgs::image_encodings::BGR8);
  const cv::Mat image = cv_ptr->image;

  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions;

  for (auto im_pred : request->predictions) {
    // cv::Mat(1, descriptor_size, CV_8UC1, im_pred.covariance_matrix.data());
    // FIXME: create the image feature prediction from image with all its values
    predictions.push_back(std::make_shared<ImageFeaturePrediction>(
        cv::Point2f(im_pred.point.x, im_pred.point.y)
    ));
  }

  FeatureDetector feature_detector(
      FeatureDetector::BuildDetector(DetectorType::AKAZE),
      FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE),
      cv::Size(image.rows, image.cols)
  );

  feature_detector.DetectFeatures(image, predictions);

  auto detected_image_features = feature_detector.GetImageFeatures();

  ekf_mono_slam::msg::ImageFeatureMeasurementArray image_feature_measurements;
  std::vector<ekf_mono_slam::msg::ImageFeatureMeasurement> response_features;

  const auto image_features = feature_detector.GetImageFeatures();
  for (auto i = 0u; i < image_features.size(); i++) {
    const auto m = image_features[i];
    ekf_mono_slam::msg::ImageFeatureMeasurement feature;
    auto coordinates = m->GetCoordinates();
    auto descriptor = m->GetDescriptorData();

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

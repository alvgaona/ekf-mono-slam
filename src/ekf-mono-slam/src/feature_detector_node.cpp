#include "feature_detector_node.h"

#include "cv_bridge/cv_bridge.h"
#include "feature/feature_detector.h"
#include "spdlog/spdlog.h"

FeatureDetectorNode::FeatureDetectorNode() : Node("feature_detector") {
  image_subscriber_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "camera/image", std::bind(&FeatureDetectorNode::image_callback, this, std::placeholders::_1), "raw"));
  image_measurements_publisher_ =
      this->create_publisher<ekf_mono_slam::msg::ImageFeatureMeasurements>("features/image/measurements", 10);
}

void FeatureDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  FeatureDetector feature_detector(FeatureDetector::BuildDetector(DetectorType::AKAZE),
                                   FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE),
                                   cv::Size(image.rows, image.cols));

  feature_detector.DetectFeatures(image);

  ekf_mono_slam::msg::ImageFeatureMeasurements image_feature_measurements;

  for (auto m : feature_detector.GetImageFeatures()) {
    ekf_mono_slam::msg::ImageFeatureMeasurement feature;
    auto coordinates = m->GetCoordinates();
    auto descriptor = m->GetDescriptorData();

    feature.feature_index = m->GetFeatureIndex();
    feature.x = coordinates.x;
    feature.y = coordinates.y;

    descriptor.copyTo(feature.descriptor);

    image_feature_measurements.features.push_back(feature);
  }

  image_measurements_publisher_->publish(image_feature_measurements);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

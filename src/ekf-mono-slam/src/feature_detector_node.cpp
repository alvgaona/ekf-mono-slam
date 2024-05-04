#include "feature_detector_node.h"

#include "cv_bridge/cv_bridge.h"
#include "feature/feature_detector.h"

FeatureDetectorNode::FeatureDetectorNode() : Node("feature_detector") {
  service_ = this->create_service<ekf_mono_slam::srv::FeatureDetector>(
      "feature_detect",
      std::bind(&FeatureDetectorNode::detect_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void FeatureDetectorNode::detect_callback(const std::shared_ptr<ekf_mono_slam::srv::FeatureDetector::Request> request,
                                          std::shared_ptr<ekf_mono_slam::srv::FeatureDetector::Response> response) {
  auto image_msg = request->image;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  FeatureDetector feature_detector(FeatureDetector::BuildDetector(DetectorType::AKAZE),
                                   FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE),
                                   cv::Size(image.rows, image.cols));

  feature_detector.DetectFeatures(image);

  // RCLCPP_INFO(this->get_logger(), "Image size '%d'", image.size().width);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

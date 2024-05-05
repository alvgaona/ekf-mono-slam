#include "ekf_node.h"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "feature/image_feature_measurement.h"

EKFNode::EKFNode() : Node("ekf_node") {
  measurements_subscriber_ = this->create_subscription<ekf_mono_slam::msg::ImageFeatureMeasurements>(
      "features/image/measurements", 10, std::bind(&EKFNode::step_callback, this, std::placeholders::_1));
}

void EKFNode::step_callback(const ekf_mono_slam::msg::ImageFeatureMeasurements::ConstSharedPtr& msg) {
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> features;

  for (auto im_feature_mes : msg->features) {
    const auto descriptor_size = im_feature_mes.descriptor.size();
    auto descriptor = cv::Mat(1, descriptor_size, CV_8UC1, im_feature_mes.descriptor.data());

    // TODO: do I need the feature index at this point or later?
    features.push_back(
        std::make_shared<ImageFeatureMeasurement>(cv::Point2f(im_feature_mes.x, im_feature_mes.y), descriptor));
  }

  if (!ekf_.isInitilized()) {
    ekf_.AddFeatures(features);

    std::cout << ekf_.GetState()->GetInverseDepthFeatures().size() << std::endl;
    return;
  }

  // TODO: do step
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

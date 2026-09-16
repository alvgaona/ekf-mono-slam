#include "ekf_node.h"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>

#include "configuration/slam_config.h"
#include "feature/descriptor_extractor_type.h"
#include "feature/detector_type.h"

namespace {

  std::string to_upper(std::string value) {
    std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); }
    );
    return value;
  }

  DetectorType parse_detector_type(const std::string& name) {
    const auto upper = to_upper(name);
    if (upper == "AKAZE") {
      return DetectorType::AKAZE;
    }
    if (upper == "BRISK") {
      return DetectorType::BRISK;
    }
    if (upper == "ORB") {
      return DetectorType::ORB;
    }
    if (upper == "FAST") {
      return DetectorType::FAST;
    }
    throw std::invalid_argument("Unsupported detector_type: " + name);
  }

  DescriptorExtractorType parse_descriptor_type(const std::string& name) {
    const auto upper = to_upper(name);
    if (upper == "AKAZE") {
      return DescriptorExtractorType::AKAZE;
    }
    if (upper == "BRISK") {
      return DescriptorExtractorType::BRISK;
    }
    if (upper == "ORB") {
      return DescriptorExtractorType::ORB;
    }
    throw std::invalid_argument("Unsupported descriptor_type: " + name);
  }

}  // namespace

void EKFNode::declare_parameters() {
  declare_parameter("camera.px", 640);
  declare_parameter("camera.py", 480);
  declare_parameter("camera.fx", 525.060143149240389);
  declare_parameter("camera.fy", 524.245488213640215);
  declare_parameter("camera.k1", -7.613e-003);
  declare_parameter("camera.k2", 9.388e-004);
  declare_parameter("camera.cx", 308.649343121753361);
  declare_parameter("camera.cy", 236.536005491807288);
  declare_parameter("camera.dx", 0.007021618750000);
  declare_parameter("camera.dy", 0.007027222916667);
  declare_parameter("camera.pixel_error_x", 1.0);
  declare_parameter("camera.pixel_error_y", 1.0);
  declare_parameter("camera.angular_vision_x", 62.720770890650357);
  declare_parameter("camera.angular_vision_y", 49.163954709609868);

  declare_parameter("kinematics.linear_accel_sd", 0.0005);
  declare_parameter("kinematics.angular_accel_sd", 0.00005);
  declare_parameter("kinematics.std_v0", 0.025);
  declare_parameter("kinematics.std_w0", 0.025);
  declare_parameter("kinematics.inv_depth_sd", 1.0);
  declare_parameter("kinematics.epsilon", 2.22e-16);

  declare_parameter("image_feature.image_area_divide_times", 2);
  declare_parameter("image_feature.image_mask_ellipse_size", 5.0);
  declare_parameter("image_feature.features_per_image", 20);
  declare_parameter("image_feature.init_inv_depth", 1.0);
  declare_parameter("image_feature.detector_type", std::string("AKAZE"));
  declare_parameter("image_feature.descriptor_type", std::string("AKAZE"));

  declare_parameter("delta_t", 0.04);
}

SlamConfig EKFNode::load_slam_config() {
  SlamConfig config;

  config.camera.px = get_parameter("camera.px").as_int();
  config.camera.py = get_parameter("camera.py").as_int();
  config.camera.fx = get_parameter("camera.fx").as_double();
  config.camera.fy = get_parameter("camera.fy").as_double();
  config.camera.k1 = get_parameter("camera.k1").as_double();
  config.camera.k2 = get_parameter("camera.k2").as_double();
  config.camera.cx = get_parameter("camera.cx").as_double();
  config.camera.cy = get_parameter("camera.cy").as_double();
  config.camera.dx = get_parameter("camera.dx").as_double();
  config.camera.dy = get_parameter("camera.dy").as_double();
  config.camera.pixel_error_x =
    get_parameter("camera.pixel_error_x").as_double();
  config.camera.pixel_error_y =
    get_parameter("camera.pixel_error_y").as_double();
  config.camera.angular_vision_x =
    get_parameter("camera.angular_vision_x").as_double();
  config.camera.angular_vision_y =
    get_parameter("camera.angular_vision_y").as_double();

  config.kinematics.linear_accel_sd =
    get_parameter("kinematics.linear_accel_sd").as_double();
  config.kinematics.angular_accel_sd =
    get_parameter("kinematics.angular_accel_sd").as_double();
  config.kinematics.std_v0 = get_parameter("kinematics.std_v0").as_double();
  config.kinematics.std_w0 = get_parameter("kinematics.std_w0").as_double();
  config.kinematics.inv_depth_sd =
    get_parameter("kinematics.inv_depth_sd").as_double();
  config.kinematics.epsilon = get_parameter("kinematics.epsilon").as_double();

  config.image_feature.image_area_divide_times =
    get_parameter("image_feature.image_area_divide_times").as_int();
  config.image_feature.image_mask_ellipse_size =
    get_parameter("image_feature.image_mask_ellipse_size").as_double();
  config.image_feature.features_per_image =
    get_parameter("image_feature.features_per_image").as_int();
  config.image_feature.init_inv_depth =
    get_parameter("image_feature.init_inv_depth").as_double();
  config.image_feature.detector_type =
    parse_detector_type(get_parameter("image_feature.detector_type").as_string()
    );
  config.image_feature.descriptor_type = parse_descriptor_type(
    get_parameter("image_feature.descriptor_type").as_string()
  );

  config.delta_t = get_parameter("delta_t").as_double();
  return config;
}

EKFNode::EKFNode() : Node("ekf_node") {
  declare_parameters();
  ekf_ = std::make_unique<EKF>(load_slam_config());

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

  RCLCPP_INFO(get_logger(), "EKF node ready (in-process feature detector)");
}

void EKFNode::publish_state() {
  const auto state = ekf_->state();
  ekf_mono_slam::msg::State msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.dimension = state->dimension();

  msg.pose.position.x = state->position().x();
  msg.pose.position.y = state->position().y();
  msg.pose.position.z = state->position().z();
  msg.pose.orientation.w = state->orientation().w();
  msg.pose.orientation.x = state->orientation().x();
  msg.pose.orientation.y = state->orientation().y();
  msg.pose.orientation.z = state->orientation().z();

  msg.velocity.linear.x = state->velocity().x();
  msg.velocity.linear.y = state->velocity().y();
  msg.velocity.linear.z = state->velocity().z();
  msg.velocity.angular.x = state->angular_velocity().x();
  msg.velocity.angular.y = state->angular_velocity().y();
  msg.velocity.angular.z = state->angular_velocity().z();

  state_publisher_->publish(msg);

  const auto& cov = ekf_->covariance_matrix()->matrix();
  ekf_mono_slam::msg::CovarianceMatrix cov_msg;
  cov_msg.header = msg.header;
  cov_msg.rows = static_cast<int32_t>(cov.rows());
  cov_msg.cols = static_cast<int32_t>(cov.cols());
  const auto bytes =
    static_cast<size_t>(cov.rows() * cov.cols()) * sizeof(double);
  cov_msg.data.resize(bytes);
  std::memcpy(cov_msg.data.data(), cov.data(), bytes);
  covariance_publisher_->publish(cov_msg);
}

void EKFNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg
) {
  const cv_bridge::CvImagePtr cv_ptr =
    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  const cv::Mat image = cv_ptr->image;

  if (!ekf_->is_initialized()) {
    RCLCPP_INFO(get_logger(), "EKF is not initialized. Initializing...");
  }

  ekf_->process_frame(image);
  publish_state();
}

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

#include "file_sequence_image_node.h"

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

FileSequenceImageNode::FileSequenceImageNode() : Node("file_sequence_image") {
  this->declare_parameter("image_dir", "");
  auto image_dir = this->get_parameter("image_dir").get_value<std::string>();

  image_provider_ = std::make_unique<FileSequenceImageProvider>(image_dir, 1, 359);
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("slam/ekf/step", 10);
  timer_ = this->create_wall_timer(40ms, std::bind(&FileSequenceImageNode::timer_callback, this));
}

void FileSequenceImageNode::timer_callback() {
  cv::Mat image = image_provider_->GetNextImage();

  cv_bridge::CvImage cv_bridge_image;
  cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_bridge_image.image = image;

  sensor_msgs::msg::Image::SharedPtr image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

  publisher_->publish(*image_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FileSequenceImageNode>());
  rclcpp::shutdown();
  return 0;
}

#include "file_sequence_image_node.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

FileSequenceImageNode::FileSequenceImageNode() : Node("file_sequence_image") {
  this->declare_parameter("image_dir", "");
  this->declare_parameter("start_image_index", 1);
  this->declare_parameter("end_image_index", 350);

  auto image_dir = this->get_parameter("image_dir").get_value<std::string>();
  auto start_image_index =
    this->get_parameter("start_image_index").get_value<uint16_t>();
  auto end_image_index =
    this->get_parameter("end_image_index").get_value<uint16_t>();

  image_provider_ = std::make_unique<FileSequenceImageProvider>(
    image_dir, start_image_index, end_image_index
  );
  image_publisher_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(this, "camera/image")
  );
  timer_ = this->create_wall_timer(40ms, [this]() { timer_callback(); });
}

void FileSequenceImageNode::timer_callback() const {
  const cv::Mat image = image_provider_->next();

  if (!image.empty()) {
    cv_bridge::CvImage cv_bridge_image;
    cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_bridge_image.image = image;

    sensor_msgs::msg::Image::SharedPtr image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    image_publisher_->publish(*image_msg);
  }
}

int main(const int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FileSequenceImageNode>());
  rclcpp::shutdown();
  return 0;
}

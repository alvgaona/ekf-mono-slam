#include "image/file_sequence_image_provider.h"

FileSequenceImageProvider::FileSequenceImageProvider(std::string directory) {
  this->directory = std::filesystem::path(directory);
  this->image_counter_ = 0;
  this->start_index_ = 1;
  this->end_index_ = 10;  // TODO: Change and get length from number of images in directory.
  this->image_type_ = ImageFileFormat::JPG;
}

cv::Mat FileSequenceImageProvider::GetNextImage() {
  if (image_counter_ == end_index_) {
    return cv::imread(std::filesystem::path(""));
  }

  std::stringstream image_number;
  image_number << std::setw(5) << std::setfill('0') << start_index_ + image_counter_;
  std::stringstream image_full_filename;
  image_full_filename << directory.c_str() << image_number.str() << "." << ImageFileUtils::ToString(image_type_);

  image_counter_++;

  spdlog::debug("Loading image: {}", image_full_filename.str());
  cv::Mat image = cv::imread(image_full_filename.str());
  spdlog::debug("Image size is {}x{}", image.rows, image.cols);
  return image;
}

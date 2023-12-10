#include "image/file_sequence_image_provider.h"

#include <spdlog/spdlog.h>

#include "image/image_file_utils.h"

FileSequenceImageProvider::FileSequenceImageProvider(const std::string& directory, const int start_index,
                                                     const int end_index) {
  this->directory = std::filesystem::path(directory);
  this->image_counter_ = 0;
  this->start_index_ = start_index;
  this->end_index_ = end_index;
  this->image_type_ = ImageFileFormat::JPG;
}

cv::Mat FileSequenceImageProvider::GetNextImage() {
  if (image_counter_ > end_index_ - start_index_) {
    spdlog::warn("No more images in directory");
  }

  std::stringstream image_number;
  image_number << std::setw(5) << std::setfill('0') << start_index_ + image_counter_;
  std::stringstream image_full_filename;
  image_full_filename << directory.c_str() << image_number.str() << "." << ImageFileUtils::ToString(image_type_);

  spdlog::debug("Loading image: {}", image_full_filename.str());
  cv::Mat image = cv::imread(image_full_filename.str());
  spdlog::debug("Image size is {}x{}", image.rows, image.cols);

  image_counter_++;

  return image;
}

#include "file_sequence_image_provider.h"

FileSequenceImageProvider::FileSequenceImageProvider(std::string directory) {
  this->directory = std::filesystem::path(directory);
  this->image_counter_ = 0;
  this->start_index_ = 1;
  this->end_index_ = 359; // TODO: Change and get length from number of images in directory.
  this->image_type_ = ImageFileType::kJpg;
}

cv::Mat FileSequenceImageProvider::GetNextImage() {
  if(image_counter_ == end_index_) {
    return cv::imread(std::filesystem::path(""));
  }

  std::stringstream image_number;
  image_number << std::setw(5) << std::setfill('0') << start_index_ + image_counter_;
  std::stringstream image_full_filename;
  image_full_filename << directory.c_str() << image_number.str() << "." << ImageFileUtils::ToString(image_type_);

  image_counter_++;

  std::cout << "Loading image: " << image_full_filename.str() << std::endl;

  return cv::imread(image_full_filename.str());
}

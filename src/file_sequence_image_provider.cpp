#include "file_sequence_image_provider.h"

FileSequenceImageProvider::FileSequenceImageProvider(std::string directory) {
  this->it = std::filesystem::directory_iterator(directory);
  this->image_counter_ = 0;
}

FileSequenceImageProvider::~FileSequenceImageProvider() {}

cv::Mat& FileSequenceImageProvider::GetNextImage() {

  // TODO: Refactor logic since directory_iterator cannot guarantee an order.

  std::filesystem::directory_iterator end_it = std::filesystem::end(it);

  if(it == end_it) {
    std::filesystem::path no_path("");
    image_ = cv::imread(no_path);
    return image_;
  }

  auto filename = it->path();
  std::cout << filename << std::endl;

  image_ = cv::imread(filename.string());

  it++;
  image_counter_++;

  return image_;
}

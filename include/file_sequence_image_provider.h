#ifndef FILE_SEQUENCE_IMAGE_PROVIDER_H
#define FILE_SEQUENCE_IMAGE_PROVIDER_H

#include "image_provider.h"

#include <exception>
#include <string>
#include <filesystem>
#include <iostream>

class FileSequenceImageProvider : public ImageProvider {
 public:
  FileSequenceImageProvider(std::string directory);
  ~FileSequenceImageProvider();
  virtual cv::Mat& GetNextImage() override;

 private:
  std::filesystem::directory_iterator it;
  int image_counter_;
};

#endif

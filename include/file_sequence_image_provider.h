#ifndef EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_
#define EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_

#include <spdlog/spdlog.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "image_file_format.h"
#include "image_file_utils.h"
#include "image_provider.h"

class FileSequenceImageProvider : public ImageProvider {
 public:
  FileSequenceImageProvider(std::string directory);
  virtual ~FileSequenceImageProvider() = default;

  virtual cv::Mat GetNextImage() override;

 private:
  std::filesystem::path directory;
  int start_index_;
  int end_index_;
  int image_counter_;
  ImageFileFormat image_type_;
};

#endif /* EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_ */

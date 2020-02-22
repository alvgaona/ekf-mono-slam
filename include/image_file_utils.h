#ifndef EKF_MONO_SLAM_IMAGE_FILE_UTILS_H
#define EKF_MONO_SLAM_IMAGE_FILE_UTILS_H

#include <exception>
#include <string>

#include "image_file_type.h"

class ImageFileUtils {
 public:
  ImageFileUtils() = delete;
  virtual ~ImageFileUtils() = delete;

  static std::string ToString(ImageFileType type) {
    switch (type) {
      case ImageFileType::kJpg:
        return "jpg";
      case ImageFileType ::kPng:
        return "png";
      default:
        throw std::runtime_error("Image type is not supported.");
    }
  }
};

#endif  // EKF_MONO_SLAM_IMAGE_FILE_UTILS_H

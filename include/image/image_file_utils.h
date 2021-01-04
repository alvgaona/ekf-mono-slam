#ifndef EKF_MONO_SLAM_IMAGE_FILE_UTILS_H
#define EKF_MONO_SLAM_IMAGE_FILE_UTILS_H

#include <exception>
#include <string>

#include "image_file_format.h"

class ImageFileUtils {
 public:
  ImageFileUtils() = delete;
  virtual ~ImageFileUtils() = delete;

  static std::string ToString(ImageFileFormat format) {
    switch (format) {
      case ImageFileFormat::JPG:
        return "jpg";
      case ImageFileFormat::PNG:
        return "png";
      default:
        throw std::runtime_error("Format is not supported.");
    }
  }
};

#endif  // EKF_MONO_SLAM_IMAGE_FILE_UTILS_H

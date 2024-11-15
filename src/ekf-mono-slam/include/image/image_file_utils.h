#pragma once

#include <stdexcept>
#include <string>

#include "image_file_format.h"

class ImageFileUtils final {
 public:
  ImageFileUtils() = delete;
  ~ImageFileUtils() = delete;

  static std::string ToString(const ImageFileFormat format) {
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

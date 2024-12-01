#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "image_file_format.h"
#include "image_provider.h"

class FileSequenceImageProvider final : public ImageProvider {
 public:
  explicit FileSequenceImageProvider(
    const std::string& directory, int start_index = 1, int end_index = 10
  );
  ~FileSequenceImageProvider() override = default;

  [[nodiscard]] int image_counter() const { return this->image_counter_; }

  cv::Mat next() override;

 private:
  std::filesystem::path directory;
  int start_index_;
  int end_index_;
  int image_counter_;
  ImageFileFormat image_type_;
};

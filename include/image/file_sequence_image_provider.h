#ifndef EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_
#define EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_


#include <filesystem>
#include <string>

#include "image_file_format.h"
#include "image_provider.h"

class FileSequenceImageProvider final : public ImageProvider {
 public:
  explicit FileSequenceImageProvider(const std::string& directory, int start_index = 1, int end_index = 10);
  ~FileSequenceImageProvider() override = default;

  [[nodiscard]] int GetImageCounter() const {
    return this->image_counter_;
  }

  cv::Mat GetNextImage() override;

 private:
  std::filesystem::path directory;
  int start_index_;
  int end_index_;
  int image_counter_;
  ImageFileFormat image_type_;
};

#endif /* EKF_MONO_SLAM_FILE_SEQUENCE_IMAGE_PROVIDER_H_ */

#include "image/file_sequence_image_provider.h"

#include <spdlog/spdlog.h>

#include "image/image_file_utils.h"

/**
 * @brief Constructs a FileSequenceImageProvider object for reading images from a sequence.
 *
 * This constructor initializes a new FileSequenceImageProvider object with the specified directory path, indices for
 * image sequence boundaries, and the default image format.
 *
 * @param directory The path to the directory containing the image sequence.
 * @param start_index The index of the first image in the sequence (inclusive).
 * @param end_index The index of the last image in the sequence (inclusive).
 *
 * This constructor allows setting up an object to efficiently iterate through a sequence of images stored in a
 * directory with a consistent naming convention based on their indices.
 */
FileSequenceImageProvider::FileSequenceImageProvider(const std::string& directory, const int start_index,
                                                     const int end_index) {
  this->directory = std::filesystem::path(directory);
  this->image_counter_ = 0;
  this->start_index_ = start_index;
  this->end_index_ = end_index;
  this->image_type_ = ImageFileFormat::JPG;
}

/**
 * @brief Retrieves the next image from the sequence.
 *
 * This method reads the next image from the specified directory based on a predefined filename format.
 *
 * @return A `cv::Mat` object containing the next image in the sequence, or an empty `cv::Mat` if no more images are
 * available.
 *
 * This method facilitates iterating through a sequence of images stored in a specific directory with consistent naming.
 */
cv::Mat FileSequenceImageProvider::GetNextImage() {
  if (image_counter_ > end_index_ - start_index_) {
    spdlog::warn("No more images in directory");
    return cv::Mat();
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

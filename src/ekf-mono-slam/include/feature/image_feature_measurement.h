#pragma once

#include <opencv2/core.hpp>

#include "image_feature.h"
#include "undistorted_image_feature.h"

class ImageFeatureMeasurement final : public ImageFeature {
 public:
  ImageFeatureMeasurement(
    cv::Point2f coordinates, const cv::Mat& descriptor_data, int index = -1
  );

  ImageFeatureMeasurement(const ImageFeatureMeasurement& source) = delete;
  ImageFeatureMeasurement(ImageFeatureMeasurement&& source) noexcept = delete;

  ImageFeatureMeasurement& operator=(const ImageFeatureMeasurement& source
  ) = delete;
  ImageFeatureMeasurement& operator=(ImageFeatureMeasurement&& source) = delete;

  ~ImageFeatureMeasurement() override = default;

  [[nodiscard]] cv::Mat descriptor_data() const { return descriptor_data_; }

  [[nodiscard]] UndistortedImageFeature undistort() const;

 private:
  cv::Mat descriptor_data_;
};

#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_H_

#include "opencv2/opencv.hpp"

class ImageFeature {
 public:
  explicit ImageFeature(cv::Point2f coordinates);
  explicit ImageFeature(cv::Point2f coordinates, int feature_index);

  virtual ~ImageFeature() = default;

  [[nodiscard]] virtual cv::Point2f GetCoordinates() const { return coordinates_; }

  [[nodiscard]] virtual int GetFeatureIndex() const { return feature_index_; }

  [[nodiscard]] virtual int ComputeZone(int zone_width, int zone_height, int image_width) const;

 protected:
  cv::Point2f coordinates_;
  int feature_index_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_H_ */

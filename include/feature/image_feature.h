#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_H_

#include <opencv4/opencv2/core/types.hpp>

class ImageFeature {
 public:
  ImageFeature() = default;
  virtual ~ImageFeature() = default;

  [[nodiscard]] cv::Point2f GetCoordinates() const { return coordinates_; }

  virtual int ComputeZone(int zone_width, int zone_height, int image_width, int image_height);

 protected:
  cv::Point2f coordinates_;
  int feature_index_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_H_ */

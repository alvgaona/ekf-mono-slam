#ifndef EKF_MONO_SLAM_ELLIPSE_H_
#define EKF_MONO_SLAM_ELLIPSE_H_

#include <opencv2/opencv.hpp>

#include "ekf_math.h"

class Ellipse {
 public:
  Ellipse(cv::Point2f center, cv::Mat matrix);
  virtual ~Ellipse() = default;

  cv::Point2f GetCenter() { return center_; }

  cv::Size2f Axes();
  double Angle();

 private:
  cv::Point2f center_;
  cv::Mat matrix_;
  cv::Mat eigen_values_;
  cv::Mat eigen_vectors_;
};

#endif /* EKF_MONO_SLAM_ELLIPSE_H_ */

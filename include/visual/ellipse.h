#ifndef EKF_MONO_SLAM_ELLIPSE_H_
#define EKF_MONO_SLAM_ELLIPSE_H_

#include <opencv2/opencv.hpp>

class Ellipse final {
 public:
  Ellipse(cv::Point2f center, const cv::Mat& matrix);
  virtual ~Ellipse() = default;

  [[nodiscard]] cv::Point2f GetCenter() const { return center_; }

  cv::Size2f Axes();
  double Angle();

  bool Contains(cv::Point2f point);

 private:
  cv::Point2f center_;
  cv::Mat matrix_;
  cv::Mat eigen_values_;
  cv::Mat eigen_vectors_;
};

#endif /* EKF_MONO_SLAM_ELLIPSE_H_ */

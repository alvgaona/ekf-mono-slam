#ifndef EKF_MONO_SLAM_EKF_MATH_H_
#define EKF_MONO_SLAM_EKF_MATH_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace EkfMath {
static constexpr double CHISQ_95_2 = 5.9915L;
static constexpr double PI = 3.14159265L;

inline double Rad2Deg(double rads) { return rads * 180.0L / PI; }

template <typename T>
inline Eigen::Matrix<T, 4, 4> toMatrix(Eigen::Quaternion<T> q) {
  Eigen::Matrix4f Q;

  Q << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(), q.x(),
      q.w();

  return Q;
}
}  // namespace EkfMath

#endif /* EKF_MONO_SLAM_EKF_MATH_H_ */

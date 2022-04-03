#ifndef EKF_MONO_SLAM_EKF_MATH_H_
#define EKF_MONO_SLAM_EKF_MATH_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace EkfMath {
static constexpr double CHISQ_95_2 = 5.9915L;
static constexpr double PI = 3.14159265L;

inline double Rad2Deg(double rads) { return rads * 180.0L / PI; }

const Eigen::MatrixXd computeJacobianDirectionalVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& directionalVector);
}  // namespace EkfMath

#endif /* EKF_MONO_SLAM_EKF_MATH_H_ */

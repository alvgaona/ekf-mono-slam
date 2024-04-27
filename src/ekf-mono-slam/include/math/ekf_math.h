#ifndef EKF_MONO_SLAM_EKF_MATH_H_
#define EKF_MONO_SLAM_EKF_MATH_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace EkfMath {
static constexpr double CHISQ_95_2 = 5.9915L;
static constexpr double PI = 3.14159265L;

inline double Rad2Deg(const double rads) { return rads * 180.0 / PI; }

Eigen::Matrix3d rotationMatrixDerivativesByq0(const Eigen::Quaterniond& q);

Eigen::Matrix3d rotationMatrixDerivativesByq1(const Eigen::Quaterniond& q);

Eigen::Matrix3d rotationMatrixDerivativesByq2(const Eigen::Quaterniond& q);

Eigen::Matrix3d rotationMatrixDerivativesByq3(const Eigen::Quaterniond& q);

Eigen::Vector3d partialDerivativeq0byOmegai(const Eigen::Vector3d& omega, double dt);

Eigen::Vector3d partialDerivativeqibyOmegai(const Eigen::Vector3d& omega, double dt);

Eigen::Matrix3d partialDerivativeqibyOmegaj(const Eigen::Vector3d& omega, double dt);

Eigen::MatrixXd jacobianDirectionalVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& directionalVector);

Eigen::Matrix2d jacobianUndistortion(const cv::Point& coordinates);
}  // namespace EkfMath

#endif /* EKF_MONO_SLAM_EKF_MATH_H_ */

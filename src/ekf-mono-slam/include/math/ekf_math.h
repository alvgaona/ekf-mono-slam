#pragma once

#include "opencv2/opencv.hpp"
#include "feature/undistorted_image_feature.h"

namespace EkfMath {
static constexpr double CHISQ_95_2 = 5.9915L;
static constexpr double PI = 3.14159265L;

inline double Rad2Deg(const double rads) { return rads * 180.0 / PI; }

bool isFeatureInFrontOfCamera(const Eigen::Vector3d& directionalVector);

bool isFeatureVisibleInFrame(const cv::Point2d& coordinates);

cv::Point2d distortImageFeature(const UndistortedImageFeature& image_feature);

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

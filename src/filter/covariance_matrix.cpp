#include "filter/covariance_matrix.h"

CovarianceMatrix::CovarianceMatrix() {
  matrix_ = Eigen::MatrixXd(13, 13);
  matrix_ << KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD;
}

// TODO: Implement
void CovarianceMatrix::Add(std::vector<std::shared_ptr<ImageFeatureMeasurement>>& image_features) {}

// TODO: Implement
void CovarianceMatrix::Add(const ImageFeature* image_feature) {}

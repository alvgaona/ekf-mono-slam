#include "covariance_matrix.h"

using namespace ConfigurationManager;

CovarianceMatrix::CovarianceMatrix() {
  matrix_ = Eigen::MatrixXd(13,13);
  matrix_ << EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, EPSILON, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, EPSILON, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, EPSILON, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, LINEAR_ACCEL_SD * LINEAR_ACCEL_SD, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, LINEAR_ACCEL_SD * LINEAR_ACCEL_SD, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, LINEAR_ACCEL_SD * LINEAR_ACCEL_SD, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ANGULAR_ACCEL_SD * ANGULAR_ACCEL_SD, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ANGULAR_ACCEL_SD * ANGULAR_ACCEL_SD, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ANGULAR_ACCEL_SD * ANGULAR_ACCEL_SD;
}

// TODO: Implement
void CovarianceMatrix::AddImageFeaturesToMatrix(std::vector<std::unique_ptr<ImageFeature>>& image_features) {}

// TODO: Implement
void CovarianceMatrix::AddImageFeatureToMatrix(const ImageFeature* image_feature) {}


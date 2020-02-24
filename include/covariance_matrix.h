#ifndef EKF_MONO_SLAM_COVARIANCE_MATRIX_H_
#define EKF_MONO_SLAM_COVARIANCE_MATRIX_H_

#include <Eigen/Dense>
#include <iostream>
#include <ostream>

#include "configuration_manager.h"
#include "image_feature.h"
#include "state.h"

class CovarianceMatrix {
 public:
  CovarianceMatrix();
  virtual ~CovarianceMatrix() = default;
  CovarianceMatrix(const CovarianceMatrix& source) = delete;
  CovarianceMatrix(CovarianceMatrix&& source) noexcept = delete;

  CovarianceMatrix& operator=(const CovarianceMatrix& source) = delete;
  CovarianceMatrix& operator=(CovarianceMatrix&& source) noexcept = delete;

  void AddImageFeaturesToMatrix(std::vector<std::unique_ptr<ImageFeature>>& image_features);
  void AddImageFeatureToMatrix(const ImageFeature* image_feature);

  friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& covariance_matrix) {
    os << covariance_matrix.matrix_;
    return os;
  }

 private:
  Eigen::MatrixXd matrix_;
};

#endif /* EKF_MONO_SLAM_COVARIANCE_MATRIX_H_ */

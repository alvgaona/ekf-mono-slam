#ifndef EKF_MONO_SLAM_COVARIANCE_MATRIX_H_
#define EKF_MONO_SLAM_COVARIANCE_MATRIX_H_

#include <Eigen/Dense>
#include <iostream>
#include <ostream>

#include "configuration/kinematics_parameters.h"
#include "feature/image_feature.h"
#include "feature/image_feature_measurement.h"
#include "math/ekf_math.h"
#include "state.h"

class CovarianceMatrix {
 public:
  CovarianceMatrix();
  CovarianceMatrix(const CovarianceMatrix& source) = delete;
  CovarianceMatrix(CovarianceMatrix&& source) noexcept = delete;

  CovarianceMatrix& operator=(const CovarianceMatrix& source) = delete;
  CovarianceMatrix& operator=(CovarianceMatrix&& source) noexcept = delete;

  virtual ~CovarianceMatrix() = default;

  friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& covariance_matrix) {
    os << covariance_matrix.matrix_;
    return os;
  }

  void Add(const ImageFeatureMeasurement* const image_feature, State* const state);

 private:
  Eigen::MatrixXd matrix_;
};

#endif /* EKF_MONO_SLAM_COVARIANCE_MATRIX_H_ */

#ifndef EKF_MONO_SLAM_COVARIANCE_MATRIX_H_
#define EKF_MONO_SLAM_COVARIANCE_MATRIX_H_

#include "feature/image_feature_measurement.h"
#include "math/ekf_math.h"
#include "state.h"

class CovarianceMatrix final {
 public:
  CovarianceMatrix();
  CovarianceMatrix(const CovarianceMatrix& source) = delete;
  CovarianceMatrix(CovarianceMatrix&& source) noexcept = delete;

  CovarianceMatrix& operator=(const CovarianceMatrix& source) = delete;
  CovarianceMatrix& operator=(CovarianceMatrix&& source) noexcept = delete;

  ~CovarianceMatrix() = default;

  friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& covariance_matrix) {
    os << covariance_matrix.matrix_;
    return os;
  }

  void Predict(const std::shared_ptr<State>& state, double dt);

  void Add(const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement,
           const std::shared_ptr<State>& state);

  [[nodiscard]] const Eigen::MatrixXd& GetMatrix() const { return matrix_; }

 private:
  Eigen::MatrixXd matrix_;
};

#endif /* EKF_MONO_SLAM_COVARIANCE_MATRIX_H_ */

#pragma once

#include <eigen3/Eigen/Core>

#include "feature/image_feature_measurement.h"
#include "feature/map_feature.h"
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

  friend std::ostream& operator<<(
    std::ostream& os, const CovarianceMatrix& covariance_matrix
  ) {
    os << covariance_matrix.matrix_;
    return os;
  }

  [[nodiscard]] Eigen::MatrixXd feature_covariance_block(
    const MapFeature& feature
  ) const;

  [[nodiscard]] Eigen::MatrixXd camera_covariance_block() const;

  void predict(const std::shared_ptr<State>& state, double dt);

  void add(
    const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement,
    const std::shared_ptr<State>& state
  );

  [[nodiscard]] const Eigen::MatrixXd& matrix() const { return matrix_; }

 private:
  Eigen::MatrixXd matrix_;
};

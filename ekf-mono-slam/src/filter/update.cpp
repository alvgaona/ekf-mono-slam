#include "filter/ekf.h"

#include <utility>

#include "math/ekf_math.h"

namespace {

  struct StackedMeasurement {
    Eigen::VectorXd z;
    Eigen::VectorXd h;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
  };

  bool stack_measurements(
    const State& state,
    const std::vector<FeatureAssociation>& associations,
    StackedMeasurement& stacked
  ) {
    const int m = static_cast<int>(associations.size());
    if (m == 0) {
      return false;
    }

    const int n = state.dimension();
    stacked.z = Eigen::VectorXd::Zero(2 * m);
    stacked.h = Eigen::VectorXd::Zero(2 * m);
    stacked.H = Eigen::MatrixXd::Zero(2 * m, n);
    stacked.R = Eigen::MatrixXd::Zero(2 * m, 2 * m);

    const auto& camera = state.config().camera;
    const double rx = camera.pixel_error_x * camera.pixel_error_x;
    const double ry = camera.pixel_error_y * camera.pixel_error_y;

    for (int i = 0; i < m; ++i) {
      const auto& association = associations[i];
      if (!association.feature() || !association.feature()->has_prediction()) {
        return false;
      }
      const auto& prediction = association.feature()->prediction();
      const Eigen::MatrixXd& Hi = prediction.measurement_jacobian();
      if (Hi.rows() != 2 || Hi.cols() != n) {
        return false;
      }

      stacked.z.segment<2>(2 * i) = association.z();
      stacked.h(2 * i) = prediction.coordinates().x;
      stacked.h(2 * i + 1) = prediction.coordinates().y;
      stacked.H.block(2 * i, 0, 2, n) = Hi;
      stacked.R(2 * i, 2 * i) = rx;
      stacked.R(2 * i + 1, 2 * i + 1) = ry;
    }
    return true;
  }

  Eigen::VectorXd kalman_delta(
    const Eigen::MatrixXd& P,
    const StackedMeasurement& stacked,
    Eigen::MatrixXd& K,
    Eigen::MatrixXd& S
  ) {
    S = stacked.H * P * stacked.H.transpose() + stacked.R;
    K = P * stacked.H.transpose() * S.inverse();
    return K * (stacked.z - stacked.h);
  }

  void apply_norm_jac(State& state, Eigen::MatrixXd& P) {
    const Eigen::Matrix4d J =
      EkfMath::quaternion_normalization_jacobian(state.orientation());
    const int n = static_cast<int>(P.rows());
    const Eigen::MatrixXd top = P.block(0, 3, 3, 4) * J.transpose();
    const Eigen::MatrixXd left = J * P.block(3, 0, 4, 3);
    const Eigen::MatrixXd quat = J * P.block(3, 3, 4, 4) * J.transpose();
    P.block(0, 3, 3, 4) = top;
    P.block(3, 0, 4, 3) = left;
    P.block(3, 3, 4, 4) = quat;
    if (n > 7) {
      const Eigen::MatrixXd right = J * P.block(3, 7, 4, n - 7);
      const Eigen::MatrixXd bottom = P.block(7, 3, n - 7, 4) * J.transpose();
      P.block(3, 7, 4, n - 7) = right;
      P.block(7, 3, n - 7, 4) = bottom;
    }
    state.normalize_orientation();
  }

}  // namespace

void EKF::update_state_only(
  State& trial, const std::vector<FeatureAssociation>& associations
) const {
  StackedMeasurement stacked;
  if (!stack_measurements(trial, associations, stacked)) {
    return;
  }

  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  const Eigen::VectorXd dx =
    kalman_delta(covariance_matrix_->matrix(), stacked, K, S);
  if (!dx.allFinite() || dx.size() != trial.dimension()) {
    return;
  }
  trial.apply_delta(dx, true);
}

void EKF::update(
  const std::vector<FeatureAssociation>& associations,
  const bool count_matches
) {
  StackedMeasurement stacked;
  if (!stack_measurements(*state_, associations, stacked)) {
    return;
  }

  const Eigen::MatrixXd P = covariance_matrix_->matrix();
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  const Eigen::VectorXd dx = kalman_delta(P, stacked, K, S);
  if (!dx.allFinite() || dx.size() != state_->dimension()) {
    return;
  }

  state_->apply_delta(dx, false);
  Eigen::MatrixXd updated = P - K * S * K.transpose();
  updated = 0.5 * (updated + updated.transpose()).eval();
  apply_norm_jac(*state_, updated);
  covariance_matrix_->matrix() = std::move(updated);

  if (count_matches) {
    for (const auto& association : associations) {
      association.feature()->increment_times_matched();
    }
  }
}

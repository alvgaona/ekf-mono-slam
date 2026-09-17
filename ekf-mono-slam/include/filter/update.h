#pragma once

#include <vector>

#include "filter/covariance_matrix.h"
#include "filter/matching.h"
#include "filter/state.h"

class KalmanUpdate {
 public:
  void update(
    State& state,
    CovarianceMatrix& covariance,
    const std::vector<FeatureAssociation>& associations,
    bool count_matches = true
  ) const;

  void update_state_only(
    State& state,
    const CovarianceMatrix& covariance,
    const std::vector<FeatureAssociation>& associations
  ) const;
};

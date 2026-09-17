#include "filter/ransac.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <utility>

#include "filter/ekf.h"

namespace {

  std::shared_ptr<MapFeature> feature_with_index(
    const State& state, const int index
  ) {
    for (const auto& feature : state.features()) {
      if (feature->index() == index) {
        return feature;
      }
    }
    return nullptr;
  }

  int adapted_hypotheses(
    const int support, const int ic_count, const RansacConfig& config
  ) {
    if (support <= 0 || ic_count <= 0) {
      return config.max_hypotheses;
    }
    if (support >= ic_count) {
      return 0;
    }
    const double epsilon =
      1.0 - static_cast<double>(support) / static_cast<double>(ic_count);
    const double inlier_prob = 1.0 - epsilon;
    if (inlier_prob <= 0.0 || inlier_prob >= 1.0) {
      return 0;
    }
    const int n_hyp = static_cast<int>(std::ceil(
      std::log(1.0 - config.success_probability) / std::log(1.0 - inlier_prob)
    ));
    return std::clamp(n_hyp, 0, config.max_hypotheses);
  }

}  // namespace

RansacSplit::RansacSplit(
  std::vector<FeatureAssociation> low_innovation,
  std::vector<FeatureAssociation> outliers
)
  : low_innovation_(std::move(low_innovation)), outliers_(std::move(outliers)) {
}

OnePointRansac::OnePointRansac(const RansacConfig& config) : config_(config) {}

RansacSplit OnePointRansac::select_low_innovation(
  const EKF& ekf, const std::vector<FeatureAssociation>& ic
) const {
  if (ic.empty()) {
    return {};
  }

  const State& state = *ekf.state();
  std::mt19937 rng(config_.rng_seed);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(ic.size()) - 1);

  int n_hyp = config_.max_hypotheses;
  std::vector<int> best_support;

  for (int i = 0; i < n_hyp; ++i) {
    const int sample = pick(rng);
    State trial(state);
    ekf.update_state_only(trial, {ic[sample]});

    std::vector<int> support_indices;
    for (int j = 0; j < static_cast<int>(ic.size()); ++j) {
      auto trial_feature =
        feature_with_index(trial, ic[j].feature()->index());
      if (!trial_feature) {
        continue;
      }
      const auto h = matcher_.project(trial, *trial_feature);
      if (!h.has_value()) {
        continue;
      }
      if ((ic[j].z() - *h).norm() < config_.innovation_threshold) {
        support_indices.push_back(j);
      }
    }

    if (static_cast<int>(support_indices.size()) >
        static_cast<int>(best_support.size())) {
      best_support = support_indices;
      n_hyp = adapted_hypotheses(
        static_cast<int>(best_support.size()),
        static_cast<int>(ic.size()),
        config_
      );
    }
  }

  std::vector<char> is_inlier(ic.size(), 0);
  for (const int index : best_support) {
    is_inlier[index] = 1;
  }
  std::vector<FeatureAssociation> low_innovation;
  std::vector<FeatureAssociation> outliers;
  for (int i = 0; i < static_cast<int>(ic.size()); ++i) {
    if (is_inlier[i]) {
      low_innovation.push_back(ic[i]);
    } else {
      outliers.push_back(ic[i]);
    }
  }
  return {std::move(low_innovation), std::move(outliers)};
}

std::vector<FeatureAssociation> OnePointRansac::rescue_high_innovation(
  State& state,
  const CovarianceMatrix& covariance,
  const std::vector<FeatureAssociation>& outliers
) const {
  std::vector<FeatureAssociation> high_innovation;
  high_innovation.reserve(outliers.size());

  for (const auto& association : outliers) {
    if (!association.feature()) {
      continue;
    }
    if (!state.compute_feature_prediction(
          association.feature(), covariance, false
        )) {
      continue;
    }
    if (matcher_.innovation_within_chi2(
          association.z(), association.feature()->prediction()
        )) {
      high_innovation.push_back(association);
    }
  }
  return high_innovation;
}

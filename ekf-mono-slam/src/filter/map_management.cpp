#include "filter/map_management.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "feature/image_feature_prediction.h"
#include "feature/inverse_depth_map_feature.h"

MapManager::MapManager(const SlamConfig& config) : config_(config) {}

void MapManager::manage(
  State& state,
  CovarianceMatrix& covariance,
  FeatureDetector& detector,
  const cv::Mat& image,
  const int inlier_count
) const {
  delete_unstable(state, covariance);
  convert_one(state, covariance);
  add_needed(state, covariance, detector, image, inlier_count);
}

void MapManager::delete_unstable(State& state, CovarianceMatrix& covariance)
  const {
  const auto& map_config = config_.map_management;
  std::vector<std::shared_ptr<MapFeature>> to_delete;
  to_delete.reserve(state.features().size());

  for (const auto& feature : state.features()) {
    if (feature->times_predicted() > map_config.min_times_predicted &&
        feature->times_matched() <
          map_config.match_rate *
            static_cast<double>(feature->times_predicted())) {
      to_delete.push_back(feature);
    }
  }

  std::ranges::sort(
    to_delete,
    [](
      const std::shared_ptr<MapFeature>& a, const std::shared_ptr<MapFeature>& b
    ) { return a->position() > b->position(); }
  );

  for (const auto& feature : to_delete) {
    covariance.remove(*feature);
    state.remove(feature);
  }
}

void MapManager::convert_one(State& state, CovarianceMatrix& covariance) const {
  const double threshold = config_.map_management.linearity_index_threshold;
  std::shared_ptr<InverseDepthMapFeature> to_convert;
  for (const auto& feature : state.inverse_depth_features()) {
    if (feature->linearity_index(state.position(), covariance.matrix()) <
        threshold) {
      to_convert = feature;
      break;
    }
  }
  if (!to_convert) {
    return;
  }
  covariance.convert_inverse_depth(
    *to_convert, to_convert->cartesian_jacobian()
  );
  state.convert_to_cartesian(to_convert);
}

void MapManager::add_needed(
  State& state,
  CovarianceMatrix& covariance,
  FeatureDetector& detector,
  const cv::Mat& image,
  const int inlier_count
) const {
  const int deficit = config_.image_feature.features_per_image - inlier_count;
  if (deficit <= 0) {
    return;
  }

  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions;
  predictions.reserve(state.features().size());
  for (const auto& feature : state.features()) {
    if (feature->has_prediction()) {
      predictions.push_back(
        std::make_shared<ImageFeaturePrediction>(feature->prediction())
      );
    }
  }

  detector.detect_features(image, predictions, deficit);
  for (const auto& measurement : detector.image_features()) {
    covariance.add(measurement, state);
    state.add(measurement);
  }
}

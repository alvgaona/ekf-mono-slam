#pragma once

#include "feature/image_feature_measurement.h"
#include "feature/map_feature.h"

void match_predictions(
  const cv::Mat& image,
  const std::vector<std::shared_ptr<MapFeature>>& map_features
);

double compute_distance(
  const cv::Mat& descriptor,
  const cv::Mat& candidate_descriptors,
  int candidate_descriptor_index
);

std::list<cv::DMatch> find_best_n_matches(
  const std::shared_ptr<MapFeature>& map_feature,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
  const cv::Mat& mask,
  int number
);

std::vector<cv::DMatch> match_features(
  const std::shared_ptr<MapFeature>& map_feature,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
  const cv::Mat& mask
);

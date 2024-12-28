#include "matching/matching.h"

#include <opencv2/core/types.hpp>

#include "feature/feature_detector.h"

// Lookup table for counting bits - typically static and computed once
static constexpr std::array<uint8_t, 256> POPCOUNT_TABLE = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3,
  3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4,
  3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4,
  4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5,
  3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2,
  2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
  4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5,
  5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5,
  5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

/**
 * @brief Matches predicted map features against detected image features
 *
 * Takes an input image and a set of predicted map feature locations, detects
 * features in the image, and attempts to match the map features with detected
 * image features. The matching is performed by:
 * 1. Collecting predictions for each map feature
 * 2. Detecting features in masked image regions around predictions
 * 3. Creating a binary mask for features within each prediction's uncertainty
 * ellipse
 * 4. Matching features using descriptor matching and ratio test
 *
 * @param image Input image to detect and match features in
 * @param map_features Vector of map features to match against
 * @return Nothing. Successful matches are currently ignored due to TODO
 */
void search_ic_matches(
  const cv::Mat& image,
  const std::vector<std::shared_ptr<MapFeature>>& map_features
) {
  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions;

  for (const auto& map_feature : map_features) {
    if (const auto prediction = map_feature->prediction(); !prediction) {
      predictions.push_back(prediction);
    }
  }

  FeatureDetector detector(
    FeatureDetector::Type::AKAZE, cv::Size(image.rows, image.cols)
  );

  const cv::Mat image_mask(cv::Mat::zeros(image.rows, image.cols, CV_8UC1));
  FeatureDetector::build_image_mask(image_mask, predictions);

  const auto image_feature_measurements =
    detector.detect_and_compute(image, image_mask);

  for (const auto& prediction : predictions) {
    cv::Mat mask(cv::Mat::zeros(
      1, static_cast<int>(image_feature_measurements.size()), CV_8UC1
    ));

    const auto ellipse = prediction->ellipse();

    for (auto i = 0; i < static_cast<int>(image_feature_measurements.size());
         i++) {
      if (ellipse.contains(image_feature_measurements[i]->coordinates())) {
        mask.at<uchar>(0, i) = 1;
      }
    }

    const auto& curr_map_feature = map_features[prediction->index()];

    const auto matches =
      match_features(curr_map_feature, image_feature_measurements, mask);

    if (matches.size() > 0) {
      // TODO: collect and return
    }
  }
}

/**
 * @brief Matches a map feature against image feature measurements using
 * descriptor matching
 *
 * @param map_feature The map feature to match against
 * @param features Vector of image feature measurements to match with
 * @param mask Binary mask indicating which features to consider for matching
 *
 * @return Vector of DMatch objects containing the matched features that pass
 * ratio test
 */
std::vector<cv::DMatch> match_features(
  const std::shared_ptr<MapFeature>& map_feature,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
  const cv::Mat& mask
) {
  std::vector<cv::DMatch> matches;
  constexpr float ratio_test_threshold = 0.8f;

  const auto best_matches = find_best_n_matches(map_feature, features, mask, 2);

  if (best_matches.size() == 1) {
    matches.push_back(best_matches.front());
  } else if (best_matches.size() >= 2) {
    // Apply Lowe's ratio test: https://stackoverflow.com/a/60343973
    const float ratio =
      best_matches.front().distance / best_matches.back().distance;
    if (ratio < ratio_test_threshold) {
      matches.push_back(best_matches.front());
    }
  }

  return matches;
}

/**
 * @brief Computes the distance between two feature descriptors
 *
 * Supports two descriptor types:
 * - CV_32F: Uses L2 (Euclidean) distance
 * - CV_8U: Uses Hamming distance with popcount lookup table
 *
 * @param query_descriptor Query descriptor matrix (1 x N)
 * @param train_descriptor Train descriptor matrix (1 x N)
 *
 * @return Distance between the descriptors (L2 or Hamming depending on type)
 */
double compute_distance(
  const cv::Mat& query_descriptor, const cv::Mat& train_descriptor
) {
  int type = query_descriptor.type();

  assert(
    (type == train_descriptor.type()) && (type == CV_32F || type == CV_8U)
  );
  assert(query_descriptor.cols == train_descriptor.cols);

  double distance = 0.f;

  if (type == CV_32F) {
    for (auto i = 0; i < train_descriptor.cols; i++) {
      const auto subs =
        query_descriptor.at<float>(i) - train_descriptor.at<float>(i);
      distance += subs * subs;
    }

    distance = std::sqrt(distance);
  }

  if (type == CV_8U) {
    auto hamming_distance = 0;

    for (auto i = 0; i < query_descriptor.cols; i++) {
      auto x = query_descriptor.at<uint8_t>(i);
      auto y = train_descriptor.at<uint8_t>(i);

      hamming_distance += POPCOUNT_TABLE.at(x ^ y);
    }

    distance = hamming_distance;
  }

  return distance;
}

/**
 * @brief Finds the N best matches for a descriptor among candidate descriptors
 *
 * @param number Number of best matches to find
 * @param descriptor Query descriptor matrix (1 x N)
 * @param candidate_descriptors Matrix of candidate descriptors (M x N)
 * @param mask Optional mask specifying which candidates to consider
 *
 * @return std::list<cv::DMatch> List of best N matches sorted by distance (best
 * first)
 */
std::list<cv::DMatch> find_best_n_matches(
  const std::shared_ptr<MapFeature>& map_feature,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
  const cv::Mat& mask,
  int number
) {
  double min_distance = -1.0L;

  std::list<cv::DMatch> best_matches;

  for (auto i = 0; i < static_cast<int>(features.size()); i++) {
    if (!mask.empty() && mask.at<bool>(i)) {
      const auto query_descriptor = map_feature->descriptor_data();
      const auto train_descriptor = features.at(i)->descriptor_data();

      auto distance = compute_distance(query_descriptor, train_descriptor);

      if (distance < min_distance || best_matches.size() < 2) {
        min_distance =
          min_distance < 0 ? distance : std::min(min_distance, distance);

        best_matches.emplace_front(0, i, static_cast<float>(distance));

        if (static_cast<int>(best_matches.size()) > number) {
          best_matches.pop_back();
        }
      }
    }
  }

  return best_matches;
}

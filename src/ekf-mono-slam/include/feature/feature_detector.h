#pragma once

#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

#include "image_feature_measurement.h"
#include "image_feature_prediction.h"
#include "zone.h"

class FeatureDetector final {
 public:
  enum class Type {
    BRISK,
    AKAZE,
    ORB,
  };

  FeatureDetector(Type feature_type, cv::Size img_size);
  ~FeatureDetector() = default;
  FeatureDetector(const FeatureDetector& source) = delete;
  FeatureDetector(FeatureDetector&& source) noexcept = delete;
  FeatureDetector& operator=(const FeatureDetector& source) = delete;
  FeatureDetector& operator=(FeatureDetector&& source) noexcept = delete;

  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& image_features(
  ) {
    return image_features_;
  }

  [[nodiscard]] int zones_in_row() const { return zones_in_row_; }

  [[nodiscard]] cv::Size zone_size() const { return zone_size_; }

  [[nodiscard]] cv::Size image_size() const { return img_size_; }

  void detect_features(const cv::Mat& image);

  void detect_features(
    const cv::Mat& image,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

  std::vector<std::shared_ptr<ImageFeatureMeasurement>> detect_and_compute(
    const cv::Mat& image, const cv::Mat& image_mask
  );

  static void build_image_mask(
    const cv::Mat& image_mask,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> image_features_;
  cv::Ptr<cv::FeatureDetector> detector_;

  cv::Size img_size_;
  cv::Size zone_size_;
  int zones_in_row_;

  static cv::Ptr<cv::FeatureDetector> create(Type type);

  void search_features_by_zone(
    const cv::Mat& image_mask,
    const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& image_features,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

  [[nodiscard]] std::vector<std::shared_ptr<Zone>> create_zones() const;

  void group_features_and_prediction_by_zone(
    std::vector<std::shared_ptr<Zone>>& zones,
    const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  ) const;

  void compute_image_feature_measurements(
    const cv::Mat& image_mask,
    const cv::Mat& descriptors,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& image_keypoints
  );

  void select_image_measurements_from_zones(
    std::list<std::shared_ptr<Zone>>& zones, const cv::Mat& image_mask
  );
};

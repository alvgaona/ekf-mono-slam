#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <math.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

#include "configuration_manager.h"
#include "draw.h"
#include "image_feature_measurement.h"
#include "image_feature_prediction.h"
#include "zone.h"

class FeatureDetector {
 public:
  FeatureDetector(cv::FeatureDetector& detector);
  ~FeatureDetector();
  FeatureDetector(const FeatureDetector& source) = delete;
  FeatureDetector(FeatureDetector&& source) noexcept = delete;

  FeatureDetector& operator=(const FeatureDetector& source) = delete;
  FeatureDetector& operator=(FeatureDetector&& source) noexcept = default;

  std::vector<std::unique_ptr<ImageFeatureMeasurement>>& GetImageFeatures();

  void DetectFeatures(cv::Mat& image, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions);

 private:
  std::vector<std::unique_ptr<ImageFeatureMeasurement>> image_features_;
  std::unique_ptr<cv::FeatureDetector> detector_;
  std::unique_ptr<cv::DescriptorExtractor> extractor_;

  void BuildImageMask(cv::Mat& image, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions);

  void SearchFeaturesByZone(std::vector<Zone*>& zones,
                            std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                            std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

  std::vector<Zone*> CreateZones(int zones_in_a_row, int zone_height, int zone_width);

  void GroupFeaturesAndPredictionsByZone(std::vector<Zone*>& zones, std::vector<ImageFeaturePrediction*> predictions,
                                         std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int image_height,
                                         int image_width);
};

#endif

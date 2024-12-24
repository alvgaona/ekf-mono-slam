#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "feature/zone.h"
#include "filter/ekf.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;

TEST(SLAMIntegration, FindFeatureInStateAndCovariance) {
  FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/"
  );
  const cv::Mat image = image_provider.next();

  FeatureDetector detector(
    FeatureDetector::build_detector(DetectorType::BRISK),
    FeatureDetector::build_descriptor_extractor(DescriptorExtractorType::BRISK),
    cv::Size(640, 480)
  );

  detector.detect_features(image);

  const auto image_features = detector.image_features();

  for (auto i = 0u; i < image_features.size(); i++) {
    image_features[i]->index(i);
  }

  ASSERT_EQ(image_features.size(), 20);

  EKF ekf;

  ekf.add_features(image_features);

  ASSERT_EQ(ekf.state()->dimension(), 13 + 20 * 6);
  ASSERT_EQ(ekf.state()->inverse_depth_features().size(), 20);
  ASSERT_EQ(ekf.state()->cartesian_features().size(), 0);

  for (size_t i = 0; i < image_features.size(); i++) {
    ASSERT_EQ(
      ekf.state()->inverse_depth_features()[i]->index(),
      image_features[i]->index()
    );
  }

  ASSERT_EQ(ekf.covariance_matrix()->matrix().cols(), 13 + 20 * 6);
  ASSERT_EQ(ekf.covariance_matrix()->matrix().rows(), 13 + 20 * 6);
}

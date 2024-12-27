#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "feature/zone.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;

TEST(FeatureDetectors, DetectFeatures) {
  FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/"
  );
  const cv::Mat image = image_provider.next();

  FeatureDetector detector(FeatureDetector::Type::BRISK, cv::Size(640, 480));

  detector.detect_features(image);

  ASSERT_EQ(detector.image_features().size(), 20);
  ASSERT_EQ(detector.zone_size(), cv::Size(160, 120));
  ASSERT_EQ(detector.image_size(), cv::Size(640, 480));
  ASSERT_EQ(detector.zones_in_row(), 4);
}

TEST(ImageFeatureMeasurement, UndistortImageFeatureMeasurement) {
  const ImageFeatureMeasurement image_feature_measurement(
    cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1), 0
  );
  const UndistortedImageFeature undistorted_image_feature =
    image_feature_measurement.undistort();

  ASSERT_EQ(
    undistorted_image_feature.coordinates(),
    Eigen::Vector2d(1.4040732757828778, 1.0760232978706483)
  );
}

TEST(Zones, CreateZone) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.id(), 0);
  ASSERT_EQ(zone.dimensions(), cv::Size(100, 100));
}

TEST(Zones, AddFeature) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.id(), 0);
  ASSERT_EQ(zone.dimensions(), cv::Size(100, 100));
}

TEST(Zones, ComputeZone) {
  const cv::Mat descriptor = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);
  const ImageFeatureMeasurement feature1(cv::Point2f(0, 0), descriptor, 0);
  ASSERT_EQ(feature1.compute_zone(480, 270, 1920), 0);

  const ImageFeatureMeasurement feature2(
    cv::Point2f(1900, 1000), descriptor, 1
  );
  ASSERT_EQ(feature2.compute_zone(480, 270, 1920), 15);

  const ImageFeatureMeasurement feature3(cv::Point2f(959, 271), descriptor, 2);
  ASSERT_EQ(feature3.compute_zone(480, 270, 1920), 5);
}

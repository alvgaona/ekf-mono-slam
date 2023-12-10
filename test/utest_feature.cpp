#include <feature/feature_detector.h>
#include <gmock/gmock-matchers.h>
#include <spdlog/spdlog.h>

#include "feature/image_feature_measurement.h"
#include "feature/zone.h"
#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

//-------------------------------------//
//   Beginning Feature Tests.          //
//-------------------------------------//

using namespace ::testing;

using ::testing::NotNull;

TEST(FeatureDetectors, CreateFeatureDetector) {
  const cv::Ptr<cv::FeatureDetector> akaze_detector = FeatureDetector::BuildDetector(DetectorType::AKAZE);
  const cv::Ptr<cv::FeatureDetector> orb_detector = FeatureDetector::BuildDetector(DetectorType::ORB);
  const cv::Ptr<cv::FeatureDetector> brisk_detector = FeatureDetector::BuildDetector(DetectorType::BRISK);

  EXPECT_NE(akaze_detector, nullptr);
  EXPECT_NE(orb_detector, nullptr);
  EXPECT_NE(brisk_detector, nullptr);
}

TEST(FeatureDetectors, NotSupportedDetector) {
  EXPECT_THROW(
    { FeatureDetector::BuildDetector(DetectorType::FAST); },
    std::runtime_error);
}

TEST(FeatureDetectors, DetectFeatures) {
  spdlog::set_level(spdlog::level::debug);
  FileSequenceImageProvider image_provider("./test/resources/AGZ_subset/MAV Images/");
  const cv::Mat image = image_provider.GetNextImage();

  FeatureDetector detector(
    FeatureDetector::BuildDetector(DetectorType::BRISK),
    FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::BRISK),
    cv::Size(1920, 1080)
    );

  detector.DetectFeatures(image);
}

TEST(ImageFeature, UndistortImageFeatureMeasurement) {
  const ImageFeatureMeasurement image_feature_measurement(cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));
  const UndistortedImageFeature undistorted_image_feature = image_feature_measurement.Undistort();

  EXPECT_EQ(undistorted_image_feature.GetCoordinates(), Eigen::Vector2d(1.4040732757828778, 1.0760232978706483));
}

TEST(Zones, CreateZone) {
  const Zone zone(0, cv::Size(100, 100));
  EXPECT_EQ(zone.GetId(), 0);
  EXPECT_EQ(zone.GetDimensions(), cv::Size(100, 100));
}

TEST(Zones, AddFeature) {
  const Zone zone(0, cv::Size(100, 100));
  EXPECT_EQ(zone.GetId(), 0);
  EXPECT_EQ(zone.GetDimensions(), cv::Size(100, 100));
}




#include <spdlog/spdlog.h>

#include <filesystem>
#include <iostream>

#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

//-------------------------------------//
//   Beginning Image Tests.          //
//-------------------------------------//

using namespace ::testing;

TEST(FileSequenceProvider, GetNextImage) {
  FileSequenceImageProvider image_provider("./test/resources/AGZ_subset/MAV Images/");
  const cv::Mat image = image_provider.GetNextImage();
  const auto size = image.size();
  EXPECT_EQ(size.height, 1080);
  EXPECT_EQ(size.width, 1920);
}

TEST(FileSequenceProvider, IncreaseCounter) {
  FileSequenceImageProvider image_provider("./test/resources/AGZ_subset/MAV Images/");
  image_provider.GetNextImage();
  image_provider.GetNextImage();

  EXPECT_EQ(image_provider.GetImageCounter(), 2);
}

TEST(FileSequenceProvider, NoMoreImagesInDirectory) {
  spdlog::set_level(spdlog::level::warn);
  FileSequenceImageProvider image_provider("./test/resources/AGZ_subset/MAV Images/", 350, 350);
  cv::Mat image = image_provider.GetNextImage();
  auto size = image.size();

  EXPECT_EQ(size.height, 1080);
  EXPECT_EQ(size.width, 1920);

  image = image_provider.GetNextImage();
  size = image.size();

  EXPECT_EQ(size.height, 0);
  EXPECT_EQ(size.width, 0);
}

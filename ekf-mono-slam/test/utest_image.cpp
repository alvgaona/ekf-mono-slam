#include <gtest/gtest.h>

#include "image/file_sequence_image_provider.h"

TEST(FileSequenceImageProvider, InitFileSequenceImageProvider) {
  const FileSequenceImageProvider image_provider(
    "./test/resources/desk_translation/"
  );

  ASSERT_EQ(image_provider.image_counter(), 0);
}

TEST(FileSequenceImageProvider, GetFirstImage) {
  FileSequenceImageProvider image_provider(
    "./test/resources/desk_translation/"
  );

  const cv::Mat image = image_provider.next();

  ASSERT_EQ(image_provider.image_counter(), 1);
  ASSERT_EQ(image.size().width, 640);
  ASSERT_EQ(image.size().height, 480);
}

TEST(FileSequenceProvider, NoMoreImagesInDirectory) {
  FileSequenceImageProvider image_provider(
    "./test/resources/desk_translation/", 2, 2
  );
  cv::Mat image = image_provider.next();
  auto size = image.size();

  ASSERT_EQ(size.height, 480);
  ASSERT_EQ(size.width, 640);

  image = image_provider.next();
  size = image.size();

  ASSERT_EQ(size.height, 0);
  ASSERT_EQ(size.width, 0);
}

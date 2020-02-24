#include "visualizer.h"

void Visualizer::VisualizeKeyPoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat image_out = image.clone();
  cv::drawKeypoints(image, keypoints, image_out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std::string windowName = "Keypoints detected";
  cv::namedWindow(windowName, 6);
  imshow(windowName, image_out);
  cv::waitKey(0);
}

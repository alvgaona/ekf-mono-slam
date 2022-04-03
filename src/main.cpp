#include "configuration/image_feature_parameters.h"
#include "slam/mono_slam.h"
#include "feature/map_feature.h"
#include "feature/map_feature_type.h"

using namespace Eigen;

int main(int argc, const char* argv[]) {
  // Setting up logger level (default should be INFO)
  spdlog::set_level(spdlog::level::debug);

  spdlog::info("Running Mono SLAM");
  //  MonoSlam mono_slam;
  //  mono_slam.run()

  MapFeature* map_feature = new MapFeature(Eigen::Vector3d::Ones(), 6, cv::Mat::zeros(cv::Size(3,3), CV_64FC1), MapFeatureType::DEPTH);

  std::vector<std::unique_ptr<MapFeature>> vec;
  vec.push_back(std::unique_ptr<MapFeature>(map_feature));
  vec.pop_back();

  return 0;
}

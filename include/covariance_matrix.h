#ifndef EKF_MONO_SLAM_COVARIANCE_MATRIX_H_
#define EKF_MONO_SLAM_COVARIANCE_MATRIX_H_

#include "configuration_manager.h"
#include "image_feature.h"
#include "state.h"

#include <opencv2/opencv.hpp>

class CovarianceMatrix {
 public:
  CovarianceMatrix();
  ~CovarianceMatrix();
  CovarianceMatrix(const CovarianceMatrix& source) = delete;
  CovarianceMatrix(CovarianceMatrix&& source) noexcept = delete;

  CovarianceMatrix& operator=(const CovarianceMatrix& source) = delete;
  CovarianceMatrix& operator=(CovarianceMatrix&& source) noexcept = delete;

  void AddImageFeaturesToMatrix(std::vector<std::unique_ptr<ImageFeature>>& image_features);
  void AddImageFeatureToMatrix(const ImageFeature* image_feature);

 private:

  cv::Mat_<double> matrix_;
};

#endif /* EKF_MONO_SLAM_COVARIANCE_MATRIX_H_ */

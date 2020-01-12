#ifndef IMAGE_FEATURE_PREDICTION_H
#define IMAGE_FEATURE_PREDICTION_H

#include "image_feature.h"
#include "opencv2/opencv.hpp"

class ImageFeaturePrediction : public ImageFeature {
 public:
    ImageFeaturePrediction();
    ~ImageFeaturePrediction();

    cv::Mat& GetCovarianceMatrix() { return covariance_matrix_; }

private:
    cv::Mat covariance_matrix_;
};

#endif

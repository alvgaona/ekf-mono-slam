#ifndef DRAW_H
#define DRAW_H

#include <opencv2/opencv.hpp>
#include "image_feature_prediction.h"
#include "ekf_math.h"
#include "ellipse.h"

namespace Draw {
    void UncertaintyEllipse2D(cv::Mat& image, ImageFeaturePrediction* prediction, int axes_max_size, cv::Scalar color, bool fill);
}

#endif

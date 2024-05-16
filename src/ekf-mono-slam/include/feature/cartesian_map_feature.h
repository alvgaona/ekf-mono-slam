#pragma once

#include "map_feature.h"

class CartesianMapFeature final : public MapFeature {
    public:
    CartesianMapFeature(const Eigen::VectorXd& state, int position, const cv::Mat& descriptor_data);
    Eigen::Vector3d ComputeDirectionalVector(const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& camera_position);
};
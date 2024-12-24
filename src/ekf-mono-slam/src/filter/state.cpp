#include "filter/state.h"

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math/ekf_math.h>

#include <typeinfo>

#include "configuration/camera_parameters.h"
#include "configuration/image_feature_parameters.h"
#include "feature/image_feature_prediction.h"
#include "feature/map_feature.h"

/**
 * @brief Constructs a State object with default initial values.
 *
 * This constructor initializes a new State object with default values for the
 * state variables representing the pose and motion of a tracked object.
 *
 * The default values represent a static object at the origin (position) with no
 * velocity or angular velocity. The initial orientation is the identity
 * quaternion and the corresponding rotation matrix is also set to the identity
 * matrix. The dimension of the state vector is specified as 13, reflecting the
 * combined size of the position, velocity, angular velocity, and quaternion
 * elements.
 *
 * This constructor provides a convenient starting point for tracking tasks
 * where you can further update the state variables based on sensor measurements
 * and dynamic models.
 */
State::State() {
  position_ = Eigen::Vector3d(0, 0, 0);
  velocity_ = Eigen::Vector3d(0, 0, 0);
  angular_velocity_ = Eigen::Vector3d(0, 0, 0);
  orientation_ = Eigen::Quaterniond(1, 0, 0, 0);
  rotation_matrix_ = orientation_.toRotationMatrix();
  dimension_ = 13;
}

State::State(
  const Eigen::Vector3d& position,
  const Eigen::Vector3d& velocity,
  const Eigen::Quaterniond& orientation,
  const Eigen::Vector3d& angular_velocity
) {
  position_ = position;
  velocity_ = velocity;
  angular_velocity_ = angular_velocity;

  // It has to be normalized in order to compute the correct rotation matrix.
  // The formula (A.45) - in J. I. Civera, A. J. Davison, and J. M. Montiel,
  // Structure from Motion using the Extended Kalman Filter. 2012.
  // doi: 10.1007/978-3-642-24834-4 - is exactly the same as the one used by
  // Eigen.
  orientation_ = orientation.normalized();
  rotation_matrix_ = orientation_.toRotationMatrix();
  dimension_ = 13;
}

/**
 * @brief Predicts the future state of the object based on its current state and
 * time interval.
 *
 * This method updates the state of the object by integrating its velocity and
 * angular velocity over the specified time interval.
 *
 * @param delta_t The time interval (in seconds) for which the prediction is
 * made.
 *
 * The prediction involves the following steps:
 * 1. Advance the object's position by its current velocity multiplied by the
 * time interval.
 * 2. Calculate the total angular displacement due to the angular velocity
 * applied over the time interval.
 * 3. Compute the rotation axis and angle from the accumulated angular
 * displacement.
 * 4. Create a quaternion representing the incremental rotation.
 * 5. Update the current orientation by multiplying the existing quaternion with
 * the incremental rotation quaternion.
 * 6. Update the rotation matrix from the updated orientation.
 *
 * This method assumes a constant or linear motion model for the object. The
 * predicted state provides an estimated pose and motion based on the latest
 * available information.
 *
 * Note that this is a simplified prediction and might not be accurate for more
 * complex motion models or external influences.
 */
void State::predict(const double delta_t) {
  // This prediction assumes constant velocity
  position_ += velocity_ * delta_t;
  const Eigen::Vector3d angles = angular_velocity_ * delta_t;

  // Compute the orientation and its rotation matrix from angles
  const double angle = angles.norm();
  const Eigen::Vector3d axis = angles.normalized();
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(angle, axis);

  orientation_ *= q;
  rotation_matrix_ = q.toRotationMatrix();
}

/**
 * @brief Removes a specified MapFeature object from the State's associated
 * feature lists.
 *
 * This method searches for the provided MapFeature object within the internal
 * lists of depth and inverse depth features and removes it if found.
 *
 * @param feature The MapFeature object to be removed.
 *
 * This method facilitates managing the associated MapFeature objects within the
 * State and keeping the lists consistent with the current state of feature
 * tracking.
 */
void State::remove(const std::shared_ptr<MapFeature>& feature) {
  if (const auto& depth_feature =
        std::dynamic_pointer_cast<DepthMapFeature>(feature)) {
    std::erase_if(
      depth_features_,
      [&depth_feature](const std::shared_ptr<MapFeature>& f) {
        return f == depth_feature;
      }
    );
  } else if (const auto& inverse_depth_feature =
               std::dynamic_pointer_cast<InverseDepthMapFeature>(feature)) {
    std::erase_if(
      inverse_depth_features_,
      [&inverse_depth_feature](const std::shared_ptr<MapFeature>& f) {
        return f == inverse_depth_feature;
      }
    );
  }
  std::erase_if(features_, [&feature](const std::shared_ptr<MapFeature>& f) {
    return f == feature;
  });
}

/**
 * @brief Adds a new image feature measurement to the State's associated
 * MapFeature list.
 *
 * This method converts the image feature measurement into a map feature and
 * adds it to the internal list of inverse depth features.
 *
 * @param image_feature_measurement The `ImageFeatureMeasurement` object
 * containing the measurement data.
 *
 * Adding image feature measurements allows the State to build and maintain a
 * map of features observed in the world, contributing to tasks like
 * localization and mapping.
 *
 */
void State::add(
  const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement
) {
  Eigen::VectorXd feature_state(6);

  const UndistortedImageFeature undistorted_feature =
    image_feature_measurement->undistort();
  Eigen::Vector3d back_projected_point = undistorted_feature.backproject();

  // Orientation of the camera respect to the world axis. Eq (A. 59)
  back_projected_point = orientation_.toRotationMatrix() * back_projected_point;

  feature_state.segment(0, 3) = position_;

  const double hx = back_projected_point.x();
  const double hy = back_projected_point.y();
  const double hz = back_projected_point.z();

  feature_state(3) = atan2(hx, hz);                        // Eq. (A. 60)
  feature_state(4) = atan2(-hy, sqrt(hx * hx + hz * hz));  // Eq. (A. 61)
  feature_state(5) = ImageFeatureParameters::init_inv_depth;

  const auto map_feature = std::make_shared<InverseDepthMapFeature>(
    feature_state,
    this->dimension_,
    image_feature_measurement->descriptor_data(),
    image_feature_measurement->index()
  );

  dimension_ += 6;

  add(map_feature);
}

/**
 * @brief Adds a provided MapFeature object to the State's internal list based
 * on its type.
 *
 * This method adds the specified MapFeature object to the appropriate internal
 * list, differentiating between depth and inverse depth features.
 *
 * @param feature The MapFeature object to be added.
 */
void State::add(const std::shared_ptr<MapFeature>& feature) {
  if (const auto& depth_feature =
        std::dynamic_pointer_cast<DepthMapFeature>(feature)) {
    depth_features_.push_back(depth_feature);
    features_.push_back(depth_feature);
  } else if (const auto& inverse_depth_feature =
               std::dynamic_pointer_cast<InverseDepthMapFeature>(feature)) {
    inverse_depth_features_.push_back(inverse_depth_feature);
    features_.push_back(inverse_depth_feature);
  }
}

void State::predict_measurement(const CovarianceMatrix& covariance_matrix) {
  predict_measurement_state();
  predict_measurement_covariance(covariance_matrix);
}

/**
 * @brief Predicts the measurement state for each map feature in the system.
 *
 * This method processes each map feature to:
 * 1. Calculate its directional vector in camera coordinates
 * 2. Check if it's in front of the camera
 * 3. Create and add predictions for features that are visible in the frame
 *
 * For each feature, the method:
 * - Computes the directional vector using the rotation matrix and position
 * - Checks if the feature is in front of the camera using the directional
 * vector
 * - If visible, creates an ImageFeaturePrediction and adds it to the map
 * feature
 *
 * Features that are either behind the camera or outside the visible frame are
 * skipped.
 */
void State::predict_measurement_state() {
  for (const auto& map_feature : features_) {
    Eigen::Vector3d directional_vector =
      map_feature->directional_vector(rotation_matrix_.transpose(), position_);
    // directionalVector = Rcw * (yi - rwc);
    if (!map_feature->is_in_front_of_camera(directional_vector)) {
      continue;
    }

    if (const auto image_feature_prediction = ImageFeaturePrediction::from(
          directional_vector, map_feature->index()
        );
        image_feature_prediction.is_visible_in_frame()) {
      map_feature->add(image_feature_prediction);
      continue;
    }
  }
}

/**
 * @brief Predicts measurement covariance for all map features in the state.
 *
 * This method computes the measurement Jacobian matrix for each map feature
 * in the state's feature list. The measurement Jacobian represents how small
 * changes in the state affect the predicted measurements.
 *
 * @param covariance_matrix The current state covariance matrix used in
 * calculating measurement Jacobians.
 *
 * For each feature, this method:
 * - Calls the feature's measurement_jacobian() method to compute and store
 *   the measurement Jacobian matrix
 * - Uses the current state and covariance matrix to perform the calculations
 */
void State::predict_measurement_covariance(
  const CovarianceMatrix& covariance_matrix
) {
  for (const auto& map_feature : features_) {
    map_feature->measurement_jacobian(*this, covariance_matrix);
  }
}

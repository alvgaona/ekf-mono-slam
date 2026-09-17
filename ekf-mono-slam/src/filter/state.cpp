#include "filter/state.h"

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math/ekf_math.h>

#include <algorithm>
#include <memory>
#include <typeinfo>

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
State::State(const SlamConfig& config) : config_(config) {
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
  const Eigen::Vector3d& angular_velocity,
  const SlamConfig& config
)
  : config_(config) {
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
Eigen::VectorXd State::packed() const {
  Eigen::VectorXd x(dimension_);
  x.segment<3>(0) = position_;
  x(3) = orientation_.w();
  x(4) = orientation_.x();
  x(5) = orientation_.y();
  x(6) = orientation_.z();
  x.segment<3>(7) = velocity_;
  x.segment<3>(10) = angular_velocity_;
  for (const auto& feature : features_) {
    x.segment(feature->position(), static_cast<int>(feature->dimension())) =
      feature->state();
  }
  return x;
}

void State::apply_delta(
  const Eigen::VectorXd& dx, const bool normalize_quaternion
) {
  position_ += dx.segment<3>(0);
  orientation_.w() += dx(3);
  orientation_.x() += dx(4);
  orientation_.y() += dx(5);
  orientation_.z() += dx(6);
  velocity_ += dx.segment<3>(7);
  angular_velocity_ += dx.segment<3>(10);
  for (const auto& feature : features_) {
    feature->apply_delta(
      dx.segment(feature->position(), static_cast<int>(feature->dimension()))
    );
  }
  if (normalize_quaternion) {
    normalize_orientation();
  }
}

void State::normalize_orientation() {
  orientation_.normalize();
  rotation_matrix_ = orientation_.toRotationMatrix();
}

State::State(const State& source)
  : position_(source.position_),
    velocity_(source.velocity_),
    angular_velocity_(source.angular_velocity_),
    orientation_(source.orientation_),
    rotation_matrix_(source.rotation_matrix_),
    dimension_(13),
    next_feature_id_(source.next_feature_id_),
    config_(source.config_) {
  for (const auto& feature : source.features_) {
    if (const auto inverse =
          std::dynamic_pointer_cast<InverseDepthMapFeature>(feature)) {
      add(std::make_shared<InverseDepthMapFeature>(*inverse));
    } else if (const auto cartesian =
                 std::dynamic_pointer_cast<CartesianMapFeature>(feature)) {
      add(std::make_shared<CartesianMapFeature>(*cartesian));
    }
  }
}

void State::predict(const double delta_t) {
  position_ += velocity_ * delta_t;
  const Eigen::Vector3d angles = angular_velocity_ * delta_t;
  const double angle = angles.norm();

  if (angle > 1e-15) {
    const Eigen::Quaterniond q{Eigen::AngleAxisd(angle, angles.normalized())};
    orientation_ *= q;
    orientation_.normalize();
  }

  rotation_matrix_ = orientation_.toRotationMatrix();
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
  const int removed_position = feature->position();
  const int removed_dimension = static_cast<int>(feature->dimension());

  if (const auto& cartesian_feature =
        std::dynamic_pointer_cast<CartesianMapFeature>(feature)) {
    std::erase_if(
      cartesian_features_,
      [&cartesian_feature](const std::shared_ptr<MapFeature>& f) {
        return f == cartesian_feature;
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

  dimension_ -= removed_dimension;
  for (const auto& remaining : features_) {
    if (remaining->position() > removed_position) {
      remaining->set_position(remaining->position() - removed_dimension);
    }
  }
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
    image_feature_measurement->undistort(config_.camera);
  Eigen::Vector3d back_projected_point =
    undistorted_feature.backproject(config_.camera);

  // Orientation of the camera respect to the world axis. Eq (A. 59)
  back_projected_point = orientation_.toRotationMatrix() * back_projected_point;

  feature_state.segment(0, 3) = position_;

  const double hx = back_projected_point.x();
  const double hy = back_projected_point.y();
  const double hz = back_projected_point.z();

  feature_state(3) = atan2(hx, hz);                        // Eq. (A. 60)
  feature_state(4) = atan2(-hy, sqrt(hx * hx + hz * hz));  // Eq. (A. 61)
  feature_state(5) = config_.image_feature.init_inv_depth;

  const auto map_feature = std::make_shared<InverseDepthMapFeature>(
    feature_state,
    this->dimension_,
    image_feature_measurement->descriptor_data(),
    next_feature_id_++
  );

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
  if (const auto& cartesian_feature =
        std::dynamic_pointer_cast<CartesianMapFeature>(feature)) {
    cartesian_features_.push_back(cartesian_feature);
    features_.push_back(cartesian_feature);
    dimension_ += static_cast<int>(cartesian_feature->dimension());
  } else if (const auto& inverse_depth_feature =
               std::dynamic_pointer_cast<InverseDepthMapFeature>(feature)) {
    inverse_depth_features_.push_back(inverse_depth_feature);
    features_.push_back(inverse_depth_feature);
    dimension_ += static_cast<int>(inverse_depth_feature->dimension());
  }
  next_feature_id_ = std::max(next_feature_id_, feature->index() + 1);
}

void State::convert_to_cartesian(
  const std::shared_ptr<InverseDepthMapFeature>& feature
) {
  if (!feature) {
    return;
  }

  const int converted_position = feature->position();
  auto cartesian = std::make_shared<CartesianMapFeature>(
    *feature, feature->cartesian_position()
  );

  for (auto& existing : features_) {
    if (existing == feature) {
      existing = cartesian;
      break;
    }
  }
  std::erase_if(
    inverse_depth_features_,
    [&feature](const std::shared_ptr<InverseDepthMapFeature>& f) {
      return f == feature;
    }
  );
  cartesian_features_.push_back(cartesian);

  dimension_ -= 3;
  for (const auto& remaining : features_) {
    if (remaining->position() > converted_position) {
      remaining->set_position(remaining->position() - 3);
    }
  }
}

void State::predict_measurement(const CovarianceMatrix& covariance_matrix) {
  for (const auto& map_feature : features_) {
    compute_feature_prediction(map_feature, covariance_matrix, true);
  }
}

bool State::compute_feature_prediction(
  const std::shared_ptr<MapFeature>& map_feature,
  const CovarianceMatrix& covariance_matrix,
  const bool increment_predicted
) {
  Eigen::Vector3d directional_vector =
    map_feature->directional_vector(rotation_matrix_.transpose(), position_);
  if (!MapFeature::is_in_front_of_camera(directional_vector, config_.camera)) {
    return false;
  }

  auto image_feature_prediction = ImageFeaturePrediction::from(
    directional_vector, map_feature->index(), config_.camera
  );
  if (!image_feature_prediction.is_visible_in_frame(config_.camera)) {
    return false;
  }

  map_feature->add(image_feature_prediction);
  if (increment_predicted) {
    map_feature->increment_times_predicted();
  }
  map_feature->measurement_jacobian(*this, covariance_matrix);
  return true;
}

#include "../include/ekf.h"

Ekf::Ekf() : dt_(0), step_(0) {
  this->covariance_matrix_ = std::make_unique<CovarianceMatrix>();
  this->state_ = std::make_unique<State>();
  this->dt_ = 1;
  this->step_ = 0;
}

Ekf::~Ekf() {}

void Ekf::Init(cv::Mat& image) {
  std::cout << "Initializing Extender Kalman Filter" << std::endl;

  // TODO: Implement initialization logic.
}

// TODO: Implement
void Ekf::Step(cv::Mat& image) {}

void Ekf::PredictState() {
  state_->PredictState(dt_);
}

// TODO: Implement
void Ekf::PredictCovarianceMatrix() {}

// TODO: Implement
void Ekf::PredictMeasurementState() {}

// TODO: Implement
void Ekf::PredictMeasurementCovariance() {}

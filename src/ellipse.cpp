#include "ellipse.h"

Ellipse::Ellipse(cv::Size2f axes, double angle) {
  this->axes_ = axes;
  this->angle_ = angle;
}

Ellipse::~Ellipse() {}

#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <opencv2/opencv.hpp>

class Ellipse {
 public:
    Ellipse(cv::Size2f axes, double angle);
    ~Ellipse();

    cv::Size2f GetAxes() { return axes_; }
    double GetAngle() { return angle_; }

private:
    cv::Size2f axes_;
    double angle_;
};

#endif

#ifndef EKF_MONO_SLAM_ELLIPSE_H_
#define EKF_MONO_SLAM_ELLIPSE_H_

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

#endif /* EKF_MONO_SLAM_ELLIPSE_H_ */

#ifndef camera_parameters_h
#define camera_parameters_h

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <cv_bridge/cv_bridge.h>

const int imageWidth = 640;
const int imageHeight = 480;

// Intel RealSense rs_00
//double kImage[9] = {620.6, 0.0, 323.9, 0.0, 619.1, 212.4, 0.0, 0.0, 1.0};
//double kDepth[9] = {620.6, 0.0, 323.9, 0.0, 619.1, 212.4, 0.0, 0.0, 1.0};

// TUM freiburg3
double kImage[9] = {535.4, 0.0, 320.1, 0.0, 539.2, 247.6, 0.0, 0.0, 1.0};
double kDepth[9] = {535.4, 0.0, 320.1, 0.0, 539.2, 247.6, 0.0, 0.0, 1.0};

#endif

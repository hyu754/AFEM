/*
This header file includes some custom variables, macros (TODO), and functions to write to file opencv vectors.
*/
#ifndef CV_WRAPPER_CUSTOM_TYPES
#define CV_WRAPPER_CUSTOM_TYPES

#include <iostream>
#include <vector>
#include <fstream>

#include "opencv2/highgui.hpp"
//Make some custom variables
typedef std::vector<cv::Point2f> imagePointVector;
typedef std::vector<cv::Point3f> worldPointVector;

#define RS 1 // run success
#define RF 0// run failed


#endif !CV_WRAPPER_CUSTOM_TYPES
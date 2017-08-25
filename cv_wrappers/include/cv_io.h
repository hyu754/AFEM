/*
This header file includes some custom variables, macros (TODO), and functions to write to file opencv vectors.
*/
#ifndef CV_IO
#define CV_IO

#include <iostream>
#include <vector>
#include <fstream>
#include "opencv2/highgui.hpp"

#include "cv_custom_wrapper_types.h"
//Function will write points to file, it is able to write 2d and 3d points
//Input:	file_name - file to save to
//			id - vector id to write to file
//			pnts - vector of points
int write_2d_points_to_file(std::string filename, imagePointVector pnts);
int write_2d_points_to_file(std::string filename, std::vector<int> id, imagePointVector pnts);


//Function will write points to file, it is able to write 3d points
//Input:	file_name - file to save to
//			time-time of event
//			id - vector id to write to file
//			pnts - vector of points
int write_3d_points_to_file(std::string filename, worldPointVector pnts);
int write_3d_points_to_file(std::string filename, float time, int counter,worldPointVector pnts);
int write_3d_points_to_file(std::string filename, std::vector<int> id, worldPointVector pnts);
#endif !CV_IO
#include "opencv2\aruco\charuco.hpp"
#include "opencv2\aruco\dictionary.hpp"
#include "opencv2\aruco.hpp"
#include <opencv2\opencv.hpp>
#include "aruco_tools.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>

//Constructor: will create aruco dictionary class and parameter classes.
stereo_ar::stereo_ar()
{
	parameters = cv::aruco::DetectorParameters::create();
	//parameters->adaptiveThreshWinSizeStep = 4;

	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

stereo_ar::~stereo_ar()
{
}

//Find aruco center for an the kinect
//x_diff: color image width

std::vector<cv::Point3f> find_aruco_center(cv::Mat input_image, int x_diff, stereo_ar ar){
	//cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	//cv::Ptr < cv::aruco::Dictionary > dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	//Variables
	std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	std::vector< int > markerIds;


	cv::Mat gray;

	cv::cvtColor(input_image, gray, cv::COLOR_RGBA2GRAY);
	cv::resize(gray, gray, cv::Size(input_image.cols / 1, input_image.rows / 1));
	cv::flip(gray, gray, 1);


	cv::aruco::detectMarkers(gray, ar.dictionary, markerCorners, markerIds, ar.parameters, rejectedCandidates);

	std::vector<cv::Point3f> markerCenter;
	double x_ave, y_ave;
	int aruco_counter = 0;
	for (auto i = markerCorners.begin(); i != markerCorners.end(); ++i){
		x_ave = y_ave = 0;
		for (auto j = i->begin(); j != i->end(); ++j){
			//Multiply by multiplier if used
			x_ave += j->x;
			y_ave += j->y;
		}
		cv::Point3f dummy3;
		dummy3.x = (x_diff)-x_ave / 4.0;
		dummy3.y = y_ave / 4.0;
		dummy3.z = markerIds.at(aruco_counter);
		markerCenter.push_back(dummy3);
		aruco_counter++;
	}

#if 1  //DRAW

	cv::aruco::drawDetectedMarkers(gray, markerCorners, markerIds);
	cv::imshow("ARUCO", gray);
#endif // 0  //DRAW
	std::vector<cv::Point3f> empty;
	if (markerCenter.size() != 0){
		return markerCenter;
	}
	else {
		return empty;
	}

	//if (markerIds.size() <2 ){
	//	return  empty;
	//}


	//else if(markerIds.size()==2){
	//	int counter = 0;
	//	for (auto i_i = markerIds.begin(); i_i != markerIds.end(); ++i_i){
	//		if ((*i_i == 1)&& (counter == 1)){
	//			cv::Point2f store = markerCenter.at(0);
	//			markerCenter.at(0) = markerCenter.at(1);
	//			markerCenter.at(1) = store;
	//		}
	//		counter++;
	//		
	//	}
	//	return markerCenter;
	//}


}



//This function assumes that the image is "what we see", 
//which is not the case for the kinect image as it is flipped in the x direction.
//Input:	Undistorted image (RGBA)
//			if outputing an image
//Ouput:	A 2d vector of all the aruco center poitns

std::vector<cv::Point2f> stereo_ar::find_aruco_center_ovr(cv::Mat input_image,std::string viewer_name){
	cv::Mat draw_image;
	input_image.copyTo(draw_image);

	//Variables
	std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	std::vector< int > markerIds;


	cv::Mat gray;
	cv::resize(input_image, input_image, cv::Size(input_image.cols, input_image.rows));
	cv::cvtColor(input_image, gray, cv::COLOR_RGBA2GRAY);

	//cv::flip(gray, gray, 1);

	std::vector<cv::Point2f> empty;
	cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
	
	if (!markerCorners.empty()){

		cv::cvtColor(draw_image, draw_image, CV_RGBA2GRAY);
		cv::aruco::drawDetectedMarkers(draw_image, markerCorners, markerIds);

	}
		

	std::vector<cv::Point2f> markerCenter;
	double x_ave, y_ave;
	for (auto i = markerCorners.begin(); i != markerCorners.end(); ++i){
		x_ave = y_ave = 0;
		for (auto j = i->begin(); j != i->end(); ++j){
			x_ave += j->x;
			y_ave += j->y;
		}


		markerCenter.push_back(cv::Point2f(x_ave / 4.0, y_ave / 4.0));

		
	}
	cv::imshow(viewer_name, draw_image);


	if (markerCenter.size() != 0){
		return markerCenter;
	}
	else {
		return empty;
	}
	//
	//#if 0  //DRAW
	//	if (!markerCenter.empty())
	//		cv::circle(gray, markerCenter[0], 10, cv::Scalar(100, 200, 200));
	//	cv::aruco::drawDetectedMarkers(gray, markerCorners, markerIds);
	//	cv::imshow("ARUCO", gray);
	//#endif // 0  //DRAW
	//	std::vector<cv::Point2f> empty;
	//	if (markerIds.size() <2){
	//		return  empty;
	//	}
	//
	//
	//	else if (markerIds.size() == 2){
	//		int counter = 0;
	//		for (auto i_i = markerIds.begin(); i_i != markerIds.end(); ++i_i){
	//			if ((*i_i == 1) && (counter == 1)){
	//				cv::Point2f store = markerCenter.at(0);
	//				markerCenter.at(0) = markerCenter.at(1);
	//				markerCenter.at(1) = store;
	//			}
	//			counter++;
	//
	//		}
	//		return markerCenter;
	//	}


}



//This function assumes that the image is "what we see", 
//which is not the case for the kinect image as it is flipped in the x direction.
//Input:	Undistorted image (RGBA)
//Ouput:	A 2d vector of all the aruco center poitns
std::vector<cv::Point2f> stereo_ar::find_aruco_center_ovr(cv::Mat input_image){


	//Variables
	std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	std::vector< int > markerIds;


	cv::Mat gray;
	cv::resize(input_image, input_image, cv::Size(input_image.cols, input_image.rows));
	cv::cvtColor(input_image, gray, cv::COLOR_RGBA2GRAY);

	//cv::flip(gray, gray, 1);

	std::vector<cv::Point2f> empty;
	cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


	std::vector<cv::Point2f> markerCenter;
	double x_ave, y_ave;
	for (auto i = markerCorners.begin(); i != markerCorners.end(); ++i){
		x_ave = y_ave = 0;
		for (auto j = i->begin(); j != i->end(); ++j){
			x_ave += j->x;
			y_ave += j->y;
		}


		markerCenter.push_back(cv::Point2f(x_ave / 4.0, y_ave / 4.0));


	}



	if (markerCenter.size() != 0){
		return markerCenter;
	}
	else {
		return empty;
	}
	//
	//#if 0  //DRAW
	//	if (!markerCenter.empty())
	//		cv::circle(gray, markerCenter[0], 10, cv::Scalar(100, 200, 200));
	//	cv::aruco::drawDetectedMarkers(gray, markerCorners, markerIds);
	//	cv::imshow("ARUCO", gray);
	//#endif // 0  //DRAW
	//	std::vector<cv::Point2f> empty;
	//	if (markerIds.size() <2){
	//		return  empty;
	//	}
	//
	//
	//	else if (markerIds.size() == 2){
	//		int counter = 0;
	//		for (auto i_i = markerIds.begin(); i_i != markerIds.end(); ++i_i){
	//			if ((*i_i == 1) && (counter == 1)){
	//				cv::Point2f store = markerCenter.at(0);
	//				markerCenter.at(0) = markerCenter.at(1);
	//				markerCenter.at(1) = store;
	//			}
	//			counter++;
	//
	//		}
	//		return markerCenter;
	//	}


}






//This function assumes that the image is "what we see", NOTE: we assume only one aruco marker in this frame 
//TO DO : add multiple aruco marker functionality
//which is not the case for the kinect image as it is flipped in the x direction.
//Input:	Undistorted image (RGBA), we assume that only one aruco marker exists in this image.
//Ouput:	A 2d vector of the four corners to an aruco marker
std::vector<cv::Point2f> stereo_ar::find_aruco_center_four_corners(cv::Mat input_image){


	//Variables
	std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
	std::vector< int > markerIds;


	cv::Mat gray;
	cv::resize(input_image, input_image, cv::Size(input_image.cols / 1.0, input_image.rows / 1.0));
	cv::cvtColor(input_image, gray, cv::COLOR_RGBA2GRAY);

	cv::Mat color_HSV;
	cv::cvtColor(input_image, color_HSV, cv::COLOR_BGR2HSV);


	int col_tol = 18;
	//cv::bilateralFilter(color_HSV, color_HSV_out, 8, 15, 15);
	//cv::bilateralFilter(color_HSV_out, color_HSV, 8, 15, 15);
	//cv::GaussianBlur(color_HSV, color_HSV, cv::Size(3, 3), 3, 3);

	// for green cv::inRange(color_HSV, cv::Scalar(60 - col_tol, 100, 50), cv::Scalar(60 + col_tol, 255, 255), gray_image);
	/*cv::inRange(color_HSV, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 70), gray);
	gray = 255 - gray;*/

	//cv::flip(gray, gray, 1);

	std::vector<cv::Point2f> empty;
	//cv::imshow("cc", gray);
	cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


	std::vector<cv::Point2f> markerCenter;

	for (auto i = markerCorners.begin(); i != markerCorners.end(); ++i){

		for (auto j = i->begin(); j != i->end(); ++j){

			markerCenter.push_back(*j);
		}





	}



	if (markerCenter.size() != 0){
		return markerCenter;
	}
	else {
		return empty;
	}
	//
	//#if 0  //DRAW
	//	if (!markerCenter.empty())
	//		cv::circle(gray, markerCenter[0], 10, cv::Scalar(100, 200, 200));
	//	cv::aruco::drawDetectedMarkers(gray, markerCorners, markerIds);
	//	cv::imshow("ARUCO", gray);
	//#endif // 0  //DRAW
	//	std::vector<cv::Point2f> empty;
	//	if (markerIds.size() <2){
	//		return  empty;
	//	}
	//
	//
	//	else if (markerIds.size() == 2){
	//		int counter = 0;
	//		for (auto i_i = markerIds.begin(); i_i != markerIds.end(); ++i_i){
	//			if ((*i_i == 1) && (counter == 1)){
	//				cv::Point2f store = markerCenter.at(0);
	//				markerCenter.at(0) = markerCenter.at(1);
	//				markerCenter.at(1) = store;
	//			}
	//			counter++;
	//
	//		}
	//		return markerCenter;
	//	}

}


worldPointVector stereo_ar::triangulate_aruco(cv::Mat imleft, cv::Mat imright){

	imagePointVector left_pnts=find_aruco_center_ovr(imleft);
	imagePointVector right_pnts = find_aruco_center_ovr(imright);
	
	//Undistort points
	left_pnts = undistort_points(left_pnts, stereo::DIRECTIONS::LEFT);
	right_pnts = undistort_points(right_pnts, stereo::DIRECTIONS::RIGHT);
	
	worldPointVector points3d = triangulation(left_pnts, right_pnts);

	return points3d;
}
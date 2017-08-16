#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "cv_custom_wrapper_types.h"

#include <iostream>
#include <cstddef>
#include <ctype.h>

using namespace std;
class hough_circle
{
private:
	//Blob detector parameters
	cv::SimpleBlobDetector::Params params;
	
	//Smart pointer to blob detector
	cv::Ptr<cv::SimpleBlobDetector> detector_blob;

	//Threshold colour
	cv::Scalar lower_thres = cv::Scalar(0.0,0.0,0.0);
	cv::Scalar upper_thres = cv::Scalar(255.0, 255.0, 70.0);

	
	
public:
	
	//Default initializer
	hough_circle();
	hough_circle(float minArea_, float minCircularity_, float minConvexity_);
	//Destructor
	~hough_circle();


	//Run the algorithm
	//Input :  RGB image - image to find circles on
	//		   resize_val - rescale the image so that the computation time is decreased
	//		   cv::size  - size of the Gaussian kernel
	//		   sigma_x,sigma_y - size of sigma in x and y directions
	//Output : std::vector<cv::KeyPoint> - vector of the circles
	std::vector<cv::KeyPoint> find_circles(const cv::Mat color_image, float resize_val = 1.0, cv::Size kernel_size = cv::Size(5.0, 5.0), float sigma_x = 3.0f, float sigma_y = 3.0f);
	//If ROI is used
	std::vector<cv::KeyPoint> find_circles(const cv::Mat color_image, cv::Rect roi,   float resize_val = 1.0,std::string window_name = "hough_circle", cv::Size kernel_size = cv::Size(5.0, 5.0), float sigma_x = 3.0f, float sigma_y = 3.0f);

	//Perform ROI segmentation, this function will basically make non roi areas to be white
	//Input :	img - image in
	//			cv::Rect roi - roi 
	//Output :	segmented image - parts not in roi will be white
	cv::Mat roi_segmentation(const cv::Mat img, const cv::Rect roi);

	


	//Set the lower and upper thresholds
	//Input :  lower_thres - lower hsv threshold
	//		   upper_thres - upper hsv threshold
	void set_threshold(cv::Scalar lower_, cv::Scalar upper_){ lower_thres = lower_; upper_thres = upper_; }
};


//Simple circle tracker, it works by first initializing a set of keypoints
//Then it will attempt to keep track of the position, it works by it will consider that the ball is the same
//if the ||pt(t) - pt(t-1)|| < tol, and that the size are similar.
class hough_circle_ball_tracker
{
public:
	//Initialize the image point previous
	//Input:	pts_in - a vector of image points
	void initialize_points(imagePointVector pts_in){ image_points_previous = pts_in; for (auto iter : pts_in){ status.push_back(1); } }

	//Set the current image points
	//Input:	pts_in - a vector of image points
	void get_current_points(imagePointVector pts_in){ image_points_current = pts_in; }

	//Run the main tracking algorithm
	//Input:	tol - the tolerance for the maximum distance
	//Output:	A vector of tracked points, 
	std::vector<cv::Point2f> run(float tol);

	//Set initialize flag
	void set_initialized_flag(bool in_bool){ initialized = in_bool; }

	//Get intialize flag
	bool return_initialized_flag(){ return initialized; }

	//Draw the image points onto a given image
	void draw_image(cv::Mat image_in,std::string window_name);

	//Return the status vector
	std::vector<int> return_status(){ return status; }

	//Return image points 
	imagePointVector return_image_points(){ return image_points_previous; }
private:
	//Vectors to store previous and current image vectors
	imagePointVector image_points_previous;
	imagePointVector image_points_current;

	//A vector to store the status of the images points.
	//1 - tracked, 0- not tracked
	std::vector<int> status;

	//Initializer flag
	bool initialized = false;

};

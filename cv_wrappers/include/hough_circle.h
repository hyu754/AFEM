#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <iostream>
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
	
	//Set the lower and upper thresholds
	//Input :  lower_thres - lower hsv threshold
	//		   upper_thres - upper hsv threshold
	void set_threshold(cv::Scalar lower_, cv::Scalar upper_){ lower_thres = lower_; upper_thres = upper_; }
};


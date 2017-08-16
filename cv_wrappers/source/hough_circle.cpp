#include "hough_circle.h"

hough_circle::hough_circle(){
	std::cout << "Initialize hough circle" << std::endl;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 75;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.2;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.51;

	// Filter by Inertia
	//params.filterByInertia = true;
	//params.minInertiaRatio = 0.51;


	detector_blob = cv::SimpleBlobDetector::create(params);
}

hough_circle::hough_circle(float minArea_, float minCircularity_, float minConvexity_){
	std::cout << "Initialize hough circle" << std::endl;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = minArea_;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = minCircularity_;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = minConvexity_;

	// Filter by Inertia
	//params.filterByInertia = true;
	//params.minInertiaRatio = 0.51;


	detector_blob = cv::SimpleBlobDetector::create(params);
}


hough_circle::~hough_circle()
{
}


std::vector<cv::KeyPoint> hough_circle::find_circles(
	const cv::Mat color_image, 
	float resize_val ,
	cv::Size kernel_size ,
	float sigma_x,
	float sigma_y )
{
	cv::Mat image_ = color_image;
	//resize the size of the image
	cv::Size image_size = color_image.size();
	cv::resize(image_, image_, cv::Size(image_size.width/resize_val, image_size.height/resize_val));

	//convert to gray scale image
	cv::Mat csv_image;
	cv::cvtColor(image_, csv_image, cv::COLOR_BGR2HSV);

	//apply gaussian kernel
	cv::GaussianBlur(csv_image, csv_image, kernel_size, sigma_x, sigma_y);

	//Apply csv threshold
	cv::Mat thres_image;
	cv::inRange(csv_image, lower_thres, upper_thres, thres_image);
	thres_image = (float)255.0 - thres_image;

	//apply gaussian kernel again
	cv::GaussianBlur(thres_image, thres_image, kernel_size, sigma_x, sigma_y);

	//Run the blob detector 
	std::vector<cv::KeyPoint> circle_points;
	detector_blob->detect(thres_image, circle_points);
	return circle_points;

}


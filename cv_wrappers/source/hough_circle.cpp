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
	float resize_val,
	cv::Size kernel_size,
	float sigma_x,
	float sigma_y)
{
	cv::Mat image_ = color_image;
	//resize the size of the image
	cv::Size image_size = color_image.size();
	cv::resize(image_, image_, cv::Size(image_size.width / resize_val, image_size.height / resize_val));

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
	cv::imshow("debug", thres_image);
	detector_blob->detect(thres_image, circle_points);
	return circle_points;

}







std::vector<cv::KeyPoint> hough_circle::find_circles(
	const cv::Mat color_image,
	cv::Rect roi,
	float resize_val,
	std::string window_name,
	cv::Size kernel_size,
	float sigma_x,
	float sigma_y)
{
	cv::Mat image_ = color_image;


	//resize the size of the image
	cv::Size image_size = color_image.size();
	cv::resize(image_, image_, cv::Size(image_size.width / resize_val, image_size.height / resize_val));

	//Apply roi
	//But, first resize it
	roi.height /= resize_val;
	roi.width /= resize_val;
	roi.x /= resize_val;
	roi.y /= resize_val;

	image_ = image_(roi);

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


	////perform roi segmentation
	//
	////Scale up image, and after roi segmentation scale down the image
	//cv::resize(thres_image, thres_image, cv::Size(image_size.width , image_size.height ));
	//thres_image=roi_segmentation(thres_image, roi);
	//cv::resize(thres_image, thres_image, cv::Size(image_size.width / resize_val, image_size.height / resize_val));


	//Run the blob detector 
	std::vector<cv::KeyPoint> circle_points;


	//thres_image = thres_image(roi);
	cv::imshow("debug" + window_name, thres_image);
	detector_blob->detect(thres_image, circle_points);




	/*
	Once blob detector is ran, we must scale the points back .
	But, these points are in the roi, so first it must be put back into original image
	*/
	for (auto circle_points_ptr = circle_points.begin(); circle_points_ptr != circle_points.end(); ++circle_points_ptr){
		//Put roi points back into non-roi image
		circle_points_ptr->pt.x += roi.x;
		circle_points_ptr->pt.y += roi.y;

		circle_points_ptr->pt *= resize_val; //multiply by resize valuate
		circle_points_ptr->size *= resize_val;
	}


	return circle_points;

}


cv::Mat hough_circle::roi_segmentation(const cv::Mat img, const cv::Rect roi){

	//Make a white image
	cv::Mat white(img.size().height, img.size().width, CV_8U, 255);

	//Now make the white part contain roi image

	for (int i = 0; i < roi.height; i++){
		for (int j = 0; j < roi.width; j++){
			int row_, col_;
			row_ = roi.y + i;
			col_ = roi.x + j;
			white.at<char>(row_, col_) = img.at<char>(row_, col_);

		}
	}
	return white;
}


std::vector<cv::Point2f> hough_circle_ball_tracker::run(float tol){
	int counter = 0;
	//container for results
	imagePointVector result;
	//Loop through the previous points and see which of the current points' position are similar
    for (auto iter = image_points_previous.begin(); iter != image_points_previous.end(); ++iter){
		float max_distance = INFINITY;
		int counter_inner = 0;
		cv::Point2f point_consider;
		for (auto iter_cur = image_points_current.begin(); iter_cur != image_points_current.end(); ++iter_cur){

			cv::Point2f diff = (*iter) - (*iter_cur);
			float distance = cv::norm(diff);

			if (distance < max_distance){
				max_distance = distance;
				point_consider = *iter_cur;
			}
			counter_inner++;
		}


		if (max_distance > tol){
			status.at(counter) = 0;
			result.push_back(cv::Point2f(0, 0));
		}
		else{
			if (status.at(counter) == 1){
				status.at(counter) = 1;
				result.push_back(point_consider);

			}
			else {
				result.push_back(cv::Point2f(0, 0));
			}
		}
		counter++;

	}
	image_points_previous = result;
	return result;
}

void hough_circle_ball_tracker::draw_image(cv::Mat image_in, std::string window_name){
	int counter = 0;
	for (auto iter = image_points_previous.begin(); iter != image_points_previous.end(); ++iter){
		if (status.at(counter)){
			cv::circle(image_in, *iter, 20, cv::Scalar(200, 02, 100), 3);
			cv::putText(image_in, std::to_string(counter), *iter,1,5,cv::Scalar(100,200,10),3);
		}
		counter++;
	}
	cv::imshow("custom ball tracking" + window_name, image_in);


}
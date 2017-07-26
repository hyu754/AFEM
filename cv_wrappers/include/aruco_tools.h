#ifndef ARUCO_CENTER_H
#define ARUCO_CENTER_H
#include <opencv2\opencv.hpp>
#include "opencv2\aruco\charuco.hpp"
#include "opencv2\aruco\dictionary.hpp"
#include "opencv2\aruco.hpp"
std::vector<cv::Point2f> find_aruco_center(cv::Mat, int);

class stereo_ar
{
public:
	stereo_ar();
	~stereo_ar();
	//
	std::vector<cv::Point2f> find_aruco_center_ovr(cv::Mat);

	//
	std::vector<cv::Point2f> find_aruco_center_four_corners(cv::Mat);

	//The first elements are the x and y position of the aruco marker, the thrid is the id of the marker
	friend std::vector<cv::Point3f> find_aruco_center(cv::Mat, int, stereo_ar);

	////Returns four corners of aruco markers
	//std::vector< std::vector<cv::Point2f> > stereo_ar::return_aruco_corners(cv::Mat input_image);

protected:
	//This is a structure to store aruco marker info

	struct arucovec
	{
		cv::Point2f position2d;
	};
private:
	cv::Ptr<cv::aruco::DetectorParameters> parameters; //= cv::aruco::DetectorParameters::create();
	cv::Ptr < cv::aruco::Dictionary > dictionary; //= cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
};



#endif // !ARUCO_CENTER_H

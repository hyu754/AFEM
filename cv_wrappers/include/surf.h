#pragma once

#include "opencv2/opencv_modules.hpp"



#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"

#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <ctype.h>

using namespace std;

class surf_track
{
private:
	//variables for optic flow
	cv::cuda::GpuMat img1_gpu;
	cv::cuda::GpuMat img2_gpu;

	//key points and descriptors
	cv::cuda::GpuMat keypoints1GPU, keypoints2GPU;
	cv::cuda::GpuMat descriptors1GPU, descriptors2GPU;

	// matching descriptors
	cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher();
	vector<cv::DMatch> matches;

	// downloading results
	vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<float> descriptors1, descriptors2;


public:
	
	//get image 1 and 2;
	//input image should be RGB
	void get_image_1(cv::Mat input_mat);
	void get_image_2(cv::Mat input_mat);

	//Runs the main surf detector algorithm
	void run_surf(bool display_matches);
	
	//Return tracked points, image_number will specify which image the key points correspond to
	std::vector<cv::Point2f> return_keypoints(int image_number);


	//Surf pointer
	cv::cuda::SURF_CUDA *surf;

	surf_track();
	~surf_track();
};


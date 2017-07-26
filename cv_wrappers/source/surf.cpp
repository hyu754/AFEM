#include "surf.h"


surf_track::surf_track(){
	//Initialize cuda matricies
	img1_gpu.create(1, 1, CV_8U);
	img2_gpu.create(1, 1, CV_8U);

	//_hessianThreshold
	surf = new cv::cuda::SURF_CUDA;
	surf->hessianThreshold = 700;
}

surf_track::~surf_track(){
	std::cout << "SURF WRAPPER DESTROYED" << std::endl;
}

void surf_track:: get_image_1(cv::Mat input_mat){
	//first convert to gray
	cv::cvtColor(input_mat, input_mat, CV_RGB2GRAY);

	//upload to GPU mat
	img1_gpu.upload(input_mat);
}

void surf_track::get_image_2(cv::Mat input_mat){
	//first convert to gray
	cv::cvtColor(input_mat, input_mat, CV_RGB2GRAY);
	
	//upload to GPU mat
	img2_gpu.upload(input_mat);

}

void surf_track::run_surf(bool display_matches){

	(*surf)(img1_gpu,cv::cuda:: GpuMat(), keypoints1GPU, descriptors1GPU);
	(*surf)(img2_gpu, cv::cuda::GpuMat(), keypoints2GPU, descriptors2GPU);

	if (keypoints2GPU.cols != 0){
		matcher->match(descriptors1GPU, descriptors2GPU, matches);

	

		surf->downloadKeypoints(keypoints1GPU, keypoints1);
		surf->downloadKeypoints(keypoints2GPU, keypoints2);


		surf->downloadDescriptors(descriptors1GPU, descriptors1);
		surf->downloadDescriptors(descriptors2GPU, descriptors2);
		

		//OF.run_LK("LK FLOW");
		// drawing the results
		cv::Mat img_matches;
		double max_dist = 0; double min_dist = 1000;
		for (int k = 0; k < matches.size(); k++)
		{
			double dist = matches[k].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}

		std::vector< 	cv::DMatch > good_matches;

		for (int l = 0; l < matches.size(); l++)
		{
			if (matches[l].distance < 1.5 * min_dist)
			{
				good_matches.push_back(matches[l]);
			}
		}
		drawMatches(cv::Mat(img1_gpu), keypoints1, cv::Mat(img2_gpu), keypoints2, good_matches, img_matches, cv::Scalar(0, 20, 100));

		//namedWindow("matches", 0);
		if (display_matches == true)
			imshow("matches", img_matches);

	}
	cv::waitKey(1);
}

std::vector<cv::Point2f> surf_track::return_keypoints(int image_number){
	std::vector<cv::Point2f> output_vector;
	if (image_number == 1){
		cv::KeyPoint::convert(keypoints1, output_vector);
	}
	else if (image_number == 2){
		cv::KeyPoint::convert(keypoints2, output_vector);
	}
	else {
		std::cout << "Error : image_number = { 1,2} only." << std::endl;
		
	}
	return output_vector;

}
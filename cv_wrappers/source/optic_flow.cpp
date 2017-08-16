#include "optic_flow.h"




const int MAX_COUNT = 500;




void optic_flow::initialize_points(std::vector<cv::Point2f> in_vector){
	original_position = in_vector;
	points[1] = in_vector;
	points[0].clear();
	initialized = true;

}

void optic_flow::add_points(cv::Point2f in_point){
	vector<cv::Point2f> tmp;
	tmp.push_back(in_point);
	cornerSubPix(gray, tmp, winSize, cv::Size(-1, -1), termcrit);
	points[1].push_back(tmp[0]);
	std::swap(points[1], points[0]);

	initialized = true;
}

void optic_flow::clear_points(void){
	points[1].clear();
	points[0].clear();
	status.clear();
}

void optic_flow::run_LK(std::string name){


	frame.copyTo(image);

	//Convert image if not gray
	if (image.channels() == 1){

		gray = image;
	}
	else {
		cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	
	
	//gray = image;
	if (!points[0].empty()){

		
		vector<float> err;

		if (prevGray.empty()){
			gray.copyTo(prevGray);
		}
		calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
			3, termcrit, 0, 0.001);
		size_t i, k;
		/*for (i = k = 0; i < points[1].size(); i++)
		{
			if (addRemovePt)
			{
				if (norm(point - points[1][i]) <= 5)
				{
					addRemovePt = false;
					continue;
				}
			}

			if (!status[i])
				continue;

			points[1][k++] = points[1][i];
			circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
		}
		points[1].resize(k);*/
	/*	for (i = 0; i < points[1].size(); i++)
		{
			if (status.at(i))
				circle(image, points[1][i], 1, Scalar(0, 250, 1), 3, 8);
			
		}*/


	}
	if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
	{
		vector<cv::Point2f> tmp;
		tmp.push_back(point);
		cornerSubPix(gray, tmp, winSize, cv::Size(-1, -1), termcrit);
		points[1].push_back(tmp[0]);
		addRemovePt = false;
	}

	//draw line if still tracked
	int _count_status_ = 0;
	for (auto status_ptr = status.begin(); status_ptr != status.end(); ++status_ptr){
		if ((*status_ptr)){
			circle(image, points[1].at(_count_status_), 20, cv::Scalar(0, 250, 1), 3, 8);
			cv::line(image, original_position.at(_count_status_), points[1].at(_count_status_), cv::Scalar(0, 100, 200));
			
		} 

		_count_status_++;
		
	}

	imshow(name, image);
	cv::waitKey(1);
	/*char c = (char)waitKey(10);
	if (c == 27)
	break;
	switch (c)
	{
	case 'r':
	needToInit = true;
	break;
	case 'c':
	points[0].clear();
	points[1].clear();
	break;
	case 'n':
	nightMode = !nightMode;
	break;
	}*/

	std::swap(points[1], points[0]);
	cv::swap(prevGray, gray);


}

void optic_flow::draw_result(cv::Mat *input_image){
	int _count_status_ = 0;
	for (auto status_ptr = status.begin(); status_ptr != status.end(); ++status_ptr){
		if ((*status_ptr)){
			circle(*input_image, points[1].at(_count_status_), 1, cv::Scalar(0, 250, 1), 3, 8);
			cv::line(*input_image, original_position.at(_count_status_), points[1].at(_count_status_), cv::Scalar(0, 100, 200));

		}

		_count_status_++;

	}
}

optic_flow::optic_flow(){
	std::cout << "Initialize optic flow using LK" << std::endl;

	termcrit.maxCount = 10;
	termcrit.epsilon = 0.01;
	 
	subPixWinSize.height = 10;
	subPixWinSize.width = 10;
	winSize.height = 10;
	winSize.width = 10;
}


optic_flow::~optic_flow()
{
}

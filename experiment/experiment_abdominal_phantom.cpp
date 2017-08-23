#include <iostream>
#include <chrono>
#include <Windows.h>
#include "AFEM_geometry.hpp"
#include "AFEM_simulation.hpp"


#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "raytracer.h"
#include "viz_tools.h"
#include "surf.h"
#include "optic_flow.h"
#include "stereo.h"
#include "hough_circle.h"
#include "cv_io.h"
#include <ovrvision_pro.h>


//#include "AFEM_cuda.cuh"


std::vector<cv::Point2f> out_vector_unordered;
//On mouse function for the left mouse button
static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		out_vector_unordered.push_back(cv::Point2f(x, y));
	}

}

//This is just a function to track the tumour positions
int stereo_vision_tracking();

//This is a function to use finite elements validation
int fem_validation();

//This function will loop through different young's modulus' and poisson ratios
int fem_parameter_estimation();
int main(void){
	//stereo_vision_tracking();
	//fem_validation();
	fem_parameter_estimation();
	return 0;
}


int stereo_vision_tracking(){
	/*****************************************************/
	/***********Creating some classes*********************/
	/*****************************************************/

	/*AFEM geometry class*/
	AFEM::Geometry geo;

	/*Aruco class*/
	stereo_ar aruco_class;

	/*	visualisation class	*/
	viz_tools viz_class;


	/*Hough circle class*/
	hough_circle h_circle;

	/*Stereo class*/
	stereo stereo_class;

	//Getting camera properties
	std::string filename_left = "Left_Intrinsic_ovr.xml";
	std::string filename_right = "Right_Intrinsic_ovr.xml";
	std::string filename_stereo = "Stereo_Prop_ovr.xml";
	stereo_class.initialize_intrinsic_distortion(filename_left, filename_right);
	stereo_class.initialize_stereo_parameters(filename_stereo);


	aruco_class.initialize_intrinsic_distortion(filename_left, filename_right);
	aruco_class.initialize_stereo_parameters(filename_stereo);


	//hough circle ball tracking class
	hough_circle_ball_tracker tracker_class[2];


	/*ovrvision class*/
	OVR::OvrvisionPro ovrvision;

	OVR::Camprop cameraMode = OVR::OV_CAMHD_FULL;
	if (ovrvision.Open(0, cameraMode) == 0){
		std::cout << "OVRVISION camera not initialized " << std::endl;
		return -1;
	}

	/*
	Cam height and width
	*/
	int ovr_height, ovr_width;
	ovr_height = ovrvision.GetCamHeight();
	ovr_width = ovrvision.GetCamWidth();
	//Image containers for the ovrvision
	cv::Mat im_left = cv::Mat(ovr_height, ovr_width, CV_8UC4);
	cv::Mat im_right = cv::Mat(ovr_height, ovr_width, CV_8UC4);


	//Set the exposure, this is a picked value
	ovrvision.SetCameraExposure(7500);
	//ovrvision.SetCameraExposure(5500);
	//ovrvision.SetCameraExposure(7000);

	//Simple mapper that will map the left oright
	std::map<int, int> left_right_mapper;

	/*****************************************************/
	/*****************************************************/
	/*****************************************************/

	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");
	geo.read_tumour("FEM_Tumour.txt");


	AFEM::Simulation sim(geo);
	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION);



	std::vector<AFEM::element> element_ = geo.return_element_vector();
	std::vector<AFEM::position_3D> position_ = geo.return_position3D();
	std::vector<AFEM::stationary> stationary = geo.return_stationary_vector();
	std::vector<int> tumour_vector = geo.return_tumour_id_vector();
	cv::VideoCapture cap(0);





	viz_class.render_geometry_FEM(element_, position_);
	viz_class.render_stationary_FEM("stationary_vec", stationary);
	//viz_class.render_geometry_surface_FEM(element_, position_);
	viz_class.render_tumour_FEM("tumour_point", tumour_vector);

	//The transformation between the tumour coords and image coords
	cv::Affine3f transformation_tumour_image = cv::Affine3f().Identity();
	//Bool to determine if above has been set
	bool transformation_t_i_set = false;

	//universal time; 
	clock_t universal_start_time = std::clock();
	int time_counter = 0;

	/*infinite loop*/
	for (;;){
		/*	cap >> input_image;*/
		std::vector<cv::KeyPoint> keypoints_output_left;
		std::vector<cv::KeyPoint> keypoints_output_right;

		//lk image, these images will only have a dot at the centre of circle
		cv::Mat lk_image_left, lk_image_right;
		if (ovrvision.isOpen()){
			ovrvision.PreStoreCamData(OVR::Camqt::OV_CAMQT_DMS);
			unsigned char* p_image = ovrvision.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT);
			unsigned char* p2_image = ovrvision.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT);

			im_left.data = p_image;
			im_right.data = p2_image;
			if (stereo_class.roi_is_set() == false){
				stereo_class.set_roi(im_left, stereo::DIRECTIONS::LEFT);
				stereo_class.set_roi(im_right, stereo::DIRECTIONS::RIGHT);
			}
			else{
				keypoints_output_left = h_circle.find_circles(im_left, stereo_class.return_roi(stereo::LEFT), 1.0, "left");
				keypoints_output_right = h_circle.find_circles(im_right, stereo_class.return_roi(stereo::RIGHT), 1.0, "right");
			}




			if (!keypoints_output_left.empty() && !keypoints_output_right.empty()){
				cv::Mat left_gray, right_gray;
				cv::cvtColor(im_left, left_gray, CV_RGBA2GRAY);
				cv::cvtColor(im_right, right_gray, CV_RGBA2GRAY);
				cv::drawKeypoints(left_gray, keypoints_output_left, left_gray, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				cv::drawKeypoints(right_gray, keypoints_output_right, right_gray, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				char key_press1 = cv::waitKey(1);

				//LEFT tracker
				if ((tracker_class[0].return_initialized_flag() == false) && (key_press1 == ' ')){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_left, keypointvector);
					tracker_class[0].initialize_points(keypointvector);
					tracker_class[0].set_initialized_flag(true);
				}
				else if (tracker_class[0].return_initialized_flag() == true){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_left, keypointvector);
					//tracker_class.initialize_points(keypointvector);
					tracker_class[0].get_current_points(keypointvector);
					tracker_class[0].run(10.0);
					tracker_class[0].draw_image(im_left, "left");
				}

				//right tracker
				if ((tracker_class[1].return_initialized_flag() == false) && (key_press1 == ' ')){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_right, keypointvector);
					tracker_class[1].initialize_points(keypointvector);
					tracker_class[1].set_initialized_flag(true);
				}
				else if (tracker_class[1].return_initialized_flag() == true){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_right, keypointvector);
					//tracker_class.initialize_points(keypointvector);
					tracker_class[1].get_current_points(keypointvector);
					tracker_class[1].run(10.0);
					tracker_class[1].draw_image(im_right, "right");
				}

				/*std::vector<cv::Point2f> keyPoints_vector_left, keyPoints_vector_right;
				cv::KeyPoint::convert(keypoints_output_left, keyPoints_vector_right);

				stereo_class.set_points(keyPoints_vector_left, stereo::DIRECTIONS::LEFT);
				stereo_class.set_points(keyPoints_vector_right, stereo::DIRECTIONS::RIGHT);

				auto output = stereo_class.triangulation();*/







				cv::imshow("left points", left_gray);
				cv::imshow("right points ", right_gray);

			}

			cv::imshow("keypoints left", im_left);
			cv::imshow("keypoints right", im_right);

			//worldPointVector aruco_3d= aruco_class.triangulate_aruco(im_left, im_right);

			char keypress = cv::waitKey(1);

			if (keypress == '='){
				ovrvision.SetCameraExposure(ovrvision.GetCameraExposure() + 50);
			}
			else if (keypress == '-') {
				ovrvision.SetCameraExposure(ovrvision.GetCameraExposure() - 50);
			}
			std::cout << "Exposure : " << ovrvision.GetCameraExposure() << std::endl;


			if (keypress == 'c'){
				std::vector<int> tracer_status_left = tracker_class[0].return_status();
				std::vector<int> tracer_status_right = tracker_class[1].return_status();
				int counter = 0;
				//for (auto left_status_ptr = tracer_status_left.begin(); left_status_ptr != tracer_status_left.end(); ++left_status_ptr){
				//	
				//	if (*left_status_ptr == 1){
				//		std::cout << "Which point on the left : " << std::to_string(counter) << " correspond to what point of right? : ";
				//		int right_point;
				//		std::cin >> right_point;
				//		if (right_point != -1) // if -1 we skip this point
				//			left_right_mapper[counter] = right_point;
				//	}
				//	counter++;

				//}
				//populate mapper

				/*int left_pair=0, right_pair=0;
				std::cout << "Please enter pairs, to stop pairs enter -1 " << std::endl;
				while (left_pair != -1){

				std::cout << std::to_string(counter) << " st/th pair " << std::endl;
				std::cout << "Please enter left index : ";
				std::cin >> left_pair;
				if (left_pair == -1) break;
				std::cout << "Please enter right index: ";
				std::cin >> right_pair;
				left_right_mapper[left_pair] = right_pair;
				counter++;

				}*/

				//If we select the correct region, then the ith circle will correspond to the ith circle
				//on the other image.
				for (int i = 0; i < tracer_status_left.size(); i++){
					left_right_mapper[i] = i;
				}





			}
			/*
			if the mapper is set then we use triangulation
			*/
			if (left_right_mapper.size() != 0){
				std::vector<cv::Point2f> keyPoints_vector_left, keyPoints_vector_right;

				for (auto mapper_iter = left_right_mapper.begin(); mapper_iter != left_right_mapper.end(); ++mapper_iter){
					keyPoints_vector_left.push_back(tracker_class[0].return_image_points().at(mapper_iter->first));
					keyPoints_vector_right.push_back(tracker_class[1].return_image_points().at(mapper_iter->second));
				}


				stereo_class.set_points(keyPoints_vector_left, stereo::DIRECTIONS::LEFT);
				stereo_class.set_points(keyPoints_vector_right, stereo::DIRECTIONS::RIGHT);

				std::vector<cv::Point3f> output = stereo_class.triangulation();
				//viz_class.render_point_cloud(output,"tumour_pos",20.0f);


				/*
				TEMPORARY FIX, we will manually find correspondances between tumour pos and cv
				*/
				std::vector<cv::Point3f> output_reordered;
				output_reordered.push_back(output.at(1));
				output_reordered.push_back(output.at(0));
				output_reordered.push_back(output.at(3));
				output_reordered.push_back(output.at(2));
				output_reordered.push_back(output.at(4));
				std::vector<cv::Point3f> tumour_3d_pos;
				//Get tumour pos in 3d
				for (int i = 0; i < 5; i++){
					tumour_3d_pos.push_back(cv::Point3f(position_.at(tumour_vector.at(i)).x, position_.at(tumour_vector.at(i)).y, position_.at(tumour_vector.at(i)).z));

				}

				if (transformation_t_i_set == false){
					transformation_tumour_image = viz_class.find_affine_transformation_3d(output_reordered, tumour_3d_pos);
					transformation_t_i_set = true;
				}
				else{
					output_reordered = viz_class.transform_points3d(transformation_tumour_image, output_reordered);

					write_3d_points_to_file("output_stereo", float(std::clock()), time_counter, output_reordered);
					//write_3d_points_to_file("output_fem", float(std::clock()), time_counter, tumour_3d_pos);
					time_counter++;
					viz_class.render_point_cloud(output_reordered, "tumour_pos", 40.0f, cv::Scalar(100, 10, 10));
				}
				//viz_class.render();
			}
		}

		viz_class.render();

	}
}

int fem_validation(){
	AFEM::Geometry geo;
	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");
	geo.read_side_constraint("FEM_SideConstraints.txt");
	bool tumour_use = geo.read_tumour("FEM_Tumour.txt");

	//geo.make_K_matrix();

	AFEM::Simulation sim(geo);
	
	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::DYNAMIC_COROTATION);
	
	//sim.cuda_tools_class.set_alpha_energy_minisation(1000000.0);
	
	

	/*
	Set up viewer
	*/


	std::vector<AFEM::element> element_ = geo.return_element_vector();
	std::vector<AFEM::position_3D> position_ = geo.return_position3D();
	std::vector<AFEM::stationary> stationary = geo.return_stationary_vector();
	std::vector<int> tumour_vector;
	tumour_vector = geo.return_tumour_id_vector();

	//Initialize viz class
	viz_tools viz_class;

	viz_class.render_geometry_FEM(element_, position_);
	viz_class.render_stationary_FEM("stationary_vec", stationary);
	//viz_class.render_geometry_surface_FEM(element_, position_);
	
		viz_class.render_tumour_FEM("tumour_point", tumour_vector);
	
	
	int counter = 0;
	sim.set_poisson_ratio(0.485f);
	sim.set_young_modulus(20000.0f);
	
	while (sim.return_iteration_number() < 100){
		if (counter == 0){
			std::string string_c = "000" + std::to_string(counter);
			sim.write_position("../matlab/AFEM_FEM_OUT/fem_" + string_c + ".txt", geo.return_tumour_id_vector());
			counter++;

		}

		sim.run();
		std::string counter_string;
		if (counter < 10){
			counter_string = "000" + std::to_string(counter);
		}
		else if (counter < 100){
			counter_string = "00" + std::to_string(counter);
		}
		else if (counter < 1000){
			counter_string = "0" + std::to_string(counter);
		}
		else if (counter < 10000){
			counter_string =  std::to_string(counter);
		}
		sim.write_position("../matlab/AFEM_FEM_OUT/fem_"+counter_string  + ".txt",geo.return_tumour_id_vector());
		counter++;

		

		viz_class.update_mesh_position("geometryline", geo.get_num_nodes(),sim.get_position_vector());
		//update tumour pos
		
		if (tumour_use == true)
			viz_class.render_tumour_FEM("tumour_point", "geometryline", tumour_vector);
		viz_class.render();
		
	}
	return 1;
	
}


int fem_parameter_estimation(){
	AFEM::Geometry geo;
	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");
	geo.read_side_constraint("FEM_SideConstraints.txt");
	bool tumour_use = geo.read_tumour("FEM_Tumour.txt");

	//geo.make_K_matrix();

	AFEM::Simulation sim(geo);

	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::DYNAMIC_COROTATION);

	//sim.cuda_tools_class.set_alpha_energy_minisation(1000000.0);



	/*
	Set up viewer
	*/


	std::vector<AFEM::element> element_ = geo.return_element_vector();
	std::vector<AFEM::position_3D> position_ = geo.return_position3D();
	std::vector<AFEM::stationary> stationary = geo.return_stationary_vector();
	std::vector<int> tumour_vector;
	tumour_vector = geo.return_tumour_id_vector();
	int counter = 0;
	//loop through different material properties
	//for (int nu_counter = 485; nu_counter < 500; nu_counter+=3){
	for (int nu_counter = 4900; nu_counter > 4700; nu_counter -= 10){
		for (int E_counter = 15000; E_counter <30000; E_counter += 1000){
			float nu = nu_counter / (float)10000.0f;
			float E = (float)E_counter;
			std::ofstream file_out("../matlab/AFEM_PARAMETER/parameter/fem_parameter_" + std::to_string(counter) + ".txt");
			file_out << std::to_string(E) + " " << std::to_string(nu) << std::endl;

			file_out.close();
			//Write the initial position to file
			sim.write_position("../matlab/AFEM_PARAMETER/initial/fem_initial_" + std::to_string(counter) + ".txt", geo.return_tumour_id_vector());

			sim.set_young_modulus(E);
			sim.set_poisson_ratio(nu);
			if (sim.get_solver_type() == AFEM::elastic_solver_type::DYNAMIC_COROTATION){
				while (sim.return_iteration_number() < 60){
					sim.run();
				}
			}
			else if (sim.get_solver_type() == AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION){
				sim.run();
			}
			sim.write_position("../matlab/AFEM_PARAMETER/final/fem_final_" + std::to_string(counter) + ".txt", geo.return_tumour_id_vector());
			sim.reset_solver();


			counter++;
		}
	}
	return 1;

}
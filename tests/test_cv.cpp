#include <iostream>
#include "AFEM_geometry.hpp"
#include "AFEM_simulation.hpp"


#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"

#include "viz_tools.h"
//#include "AFEM_cuda.cuh"
#include "raytracer.h"

int main(void){
	AFEM::Geometry geo;
	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");

	AFEM::Simulation sim(geo);
	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION);
	
	sim.run();

	std::vector<AFEM::element> element_ =geo.return_element_vector();
	std::vector<AFEM::position_3D> position_ = geo.return_position3D();
	cv::VideoCapture cap(0);
	if (!cap.isOpened()){
		return -1;
	}

	cv::Mat input_image;
	cap >> input_image;


	viz_tools viz_class;
	viz_class.set_window_size(input_image.size());
	viz_class.set_virtual_camera_intrinsics();

	//viz_class.create_plane(cv::Vec2d(20, 20));
	viz_class.set_camera_position();
	viz_class.set_camera();
	viz_class.render_geometry_FEM(element_,position_);


	std::vector<cv::Point3f> position_3f ;
	for (auto position_ptr = position_.begin(); position_ptr != position_.end(); ++position_ptr){
		position_3f.push_back(cv::Point3f(position_ptr->x, position_ptr->y, position_ptr->z));
	}
	stereo_ar ar_class;
	/*
	
	*/

	viz_class.generate_rays();
	viz_class.ray_tracer();

	glm::vec3 A(0, 0, 0);
	glm::vec3 B(0, 10, 0);
	glm::vec3 C(10, 10, 0);

	glm::vec3 ray(0, 0,1);
	glm::vec3 start_ray(0.2, 0.7, 500);
	raytracer tr;
	glm::vec3 output = tr.ray_tracer(A, B, C, start_ray, ray);

	glm::vec3 reprojected = tr. reproject_trace(A, B, C, output.x, output.y);

	while (1){
		cap >> input_image;
		std::vector<cv::Point2f> out_vector_unordered= ar_class.find_aruco_center_four_corners(input_image);
		int text_counter = 0;
		for (auto ptr_auto = out_vector_unordered.begin(); ptr_auto != out_vector_unordered.end(); ++ptr_auto){
			cv::circle(input_image, *ptr_auto, 5, cv::Scalar(255, 100, 100), 5);
			cv::putText(input_image, std::to_string(text_counter), *ptr_auto, 1, 2, cv::Scalar(10, 200, 100));
			text_counter++;
		}
		cv::imshow("input_image", input_image);
		cv::waitKey(1);
		std::vector<int> indicies2d, indicies3d;
		indicies2d.push_back(0);		indicies2d.push_back(1);		indicies2d.push_back(2);		indicies2d.push_back(3);
		//127   124   112   115
		//15    12     0     3
		indicies3d.push_back(15);		indicies3d.push_back(12);		indicies3d.push_back(0);		indicies3d.push_back(3);

		//Temporary solution
		std::vector<cv::Point3f> points;


		cv::Vec2d size_plane = cv::Vec2d(20, 20);
		points.push_back(cv::Point3f(0, 0, 0));		points.push_back(cv::Point3f(0, size_plane(1), 0));		points.push_back(cv::Point3f(size_plane(0), 0, 0));		points.push_back(cv::Point3f(size_plane(0), size_plane(1), 0));

		if (points.size() == out_vector_unordered.size()){
			cv::Affine3f pose = viz_class.solve_pnp_matrix(position_3f, indicies3d, out_vector_unordered, indicies2d);
			cv::Mat augmented_image = viz_class.augment_mesh(input_image, "geometry", pose);
			cv::imshow("augmented image", augmented_image);
			cv::waitKey(1);

		}


		viz_class.render();

	}
	return 0;
}
//#include "viz_tools.h"
///**
//* @function main
//*/
//int main()
//{
//	/// Create a window
//	viz::Viz3d myWindow("Creating Widgets");
//
//	/// Create a triangle widget
//	for (int i = 0; i < 40; i++){
//		WTriangle tw(Point3f(0.0, 0.0, 0.0+i), Point3f(1.0, 1.0, 1.0+i), Point3f(0.0, 1.0, 0.0+i), viz::Color::red());
//		WLineTriangle lll(Point3f(0.0, 0.0, 0.0 + i), Point3f(1.0, 1.0, 1.0 + i), Point3f(0.0, 1.0, 0.0 + i), viz::Color::red());
//
//		/// Show widget in the visualizer window
//		myWindow.showWidget("triangle"+std::to_string(i), tw);
//		myWindow.showWidget("line" + std::to_string(i), lll);
//	}
//	/// Start event loop
//	myWindow.spin();
//
//	return 0;
//}
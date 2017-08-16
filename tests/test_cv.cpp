#include <iostream>
#include <chrono>
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

int main(void){
	AFEM::Geometry geo;
	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");



	AFEM::Simulation sim(geo);
	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION);



	std::vector<AFEM::element> element_ = geo.return_element_vector();
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
	viz_class.render_geometry_FEM(element_, position_);
	viz_class.render_geometry_surface_FEM(element_, position_);

	std::vector<cv::Point3f> position_3f;
	for (auto position_ptr = position_.begin(); position_ptr != position_.end(); ++position_ptr){
		position_3f.push_back(cv::Point3f(position_ptr->x, position_ptr->y, position_ptr->z));
	}
	stereo_ar ar_class;
	/*

	*/

	//viz_class.generate_rays();
	viz_class.ray_tracer();

	glm::vec3 A(0, 0, 0);
	glm::vec3 B(0, 10, 0);
	glm::vec3 C(10, 10, 0);

	glm::vec3 ray(0, 0, 1);
	glm::vec3 start_ray(0.2, 0.7, 500);
	raytracer tr;
	glm::vec3 output = tr.ray_tracer(A, B, C, start_ray, ray);

	glm::vec3 reprojected = tr.reproject_trace(A, B, C, output.x, output.y);
	int sin_counter = 0;

	/*
	Initialize optic flow and surf classes
	*/

	//initialize lk alg
	optic_flow OF;
	surf_track OS;

	viz_class.render_stationary_FEM("geometry", geo.return_stationary_vector());
	while (1){

		/*
		UPDATE VISUAL MESH FROM FEM SOLVER
		*/
		std::vector<cv::Point3f> position_3f;

		AFEM::position_3D * position_ptr = sim.get_position_vector();
		for (int node_i = 0; node_i < geo.get_num_nodes(); node_i++){

			AFEM::position_3D node_current = position_ptr[node_i];
			position_.at(node_i) = node_current;
			position_3f.push_back(cv::Point3f(node_current.x, node_current.y, node_current.z));
		}
		viz_class.update_mesh_position("geometry", position_); //update entire mesh
		viz_class.update_mesh_surface_position("geometry_surface", position_); //update the surface

		cap >> input_image;

		/*
		RUN LK AND SURF (if haven't already)
		*/

		OF.get_frame(input_image);
		OF.run_LK("LK FLOW");

		char key_input = cv::waitKey(1);
		if (key_input == ' '){
			OF.set_initialized_bool(false);
			std::cout << " " << std::endl;
		}
		if (OF.get_initialized_bool() == false){
			OS.get_image_1(input_image);
			OS.get_image_2(input_image);
			OS.run_surf(true);
			OF.clear_points();
			std::vector<cv::Point2f> points_vector = OS.return_keypoints(1);
			if (points_vector.size() > 0)
				OF.initialize_points(points_vector);
		}
#if ARUCO_TEST_AUGMENT
		std::vector<cv::Point2f> out_vector_unordered;
		out_vector_unordered = ar_class.find_aruco_center_four_corners(input_image);
#endif // ARUCO_TEST_AUGMENT



		/*
		AUGMENT THE MESH
		*/

		cv::setMouseCallback("input_image", onMouse, 0);
		//If extrinsic has not been found , we draw to screen
		if (viz_class.extrinsic_found == false){
			int text_counter = 0;
			for (auto ptr_auto = out_vector_unordered.begin(); ptr_auto != out_vector_unordered.end(); ++ptr_auto){
				cv::circle(input_image, *ptr_auto, 5, cv::Scalar(255, 100, 100), 5);
				cv::putText(input_image, std::to_string(text_counter), *ptr_auto, 1, 2, cv::Scalar(10, 200, 100));
				text_counter++;
			}
			cv::imshow("input_image", input_image);
			cv::waitKey(1);
		}


		std::vector<int> indicies2d, indicies3d;
		indicies2d.push_back(0);		indicies2d.push_back(1);		indicies2d.push_back(2);		indicies2d.push_back(3);
		//127   124   112   115
		//15    12     0     3
		//  16    28    31    19
		//36    66    71    41
		//97    91    49    55
		//   287   276   144   155
		indicies3d.push_back(97);		indicies3d.push_back(91);		indicies3d.push_back(49);		indicies3d.push_back(55);

		std::vector<cv::Point2f> tracked_points_vector = OF.return_tracked_point();
		std::vector<uchar> tracked_point_status_vector = OF.return_status();

		/*
		Below if statment surrounds code that will find the transformation between the camera frame and the world coordiante
		*/

		if (indicies3d.size() == out_vector_unordered.size()){


			if (viz_class.extrinsic_found == false){
				viz_class.solve_pnp_matrix(position_3f, indicies3d, out_vector_unordered, indicies2d);
				viz_class.extrinsic_found = true;
			}
			else{

				std::vector<cv::Point3f> tracked_points_nonhomo;
				for (auto t_p = tracked_points_vector.begin(); t_p != tracked_points_vector.end(); ++t_p){
					cv::Point3f worldPoint = viz_class.transform_image_to_original_frame(*t_p);
					tracked_points_nonhomo.push_back(worldPoint); // this should be in the original reference frame
				}
				if (viz_class.return_ray_tracer_class()->get_ray_tracer_status() == false)
					viz_class.generate_rays(tracked_points_nonhomo);


			}


			cv::Mat augmented_image = viz_class.augment_mesh(input_image, "geometry", viz_class.pose_affine);
			OF.draw_result(&augmented_image);
			cv::imshow("augmented image", augmented_image);
			cv::waitKey(1);

		}

		std::vector<viz_tools::face_information> ray_info = viz_class.return_ray_trace_vector();

		//If we have not performed the ray tracer
		if (viz_class.return_ray_tracer_class()->get_ray_tracer_status() == false){
			//run the ray tracer
			viz_class.ray_tracer();
		}
		else { //Else, if we have performed the ray tracer 
			//Find the new position of feature points on the mesh

			std::vector<cv::Point2f> tracked_points_vector = OF.return_tracked_point();
			std::vector<uchar> tracked_point_status_vector = OF.return_status();
			//Set sudo forces
			//First we loop through all of the different faces with intersection
			for (auto ray_info_ptr = ray_info.begin(); ray_info_ptr != ray_info.end(); ++ray_info_ptr){
				//Initialize the new position with values from initial position
				//This is because if they are the same , then it means we are not tracking it
				ray_info_ptr->intersection_vector_t = ray_info_ptr->intersection_vector_t0;

				//Then we loop through all of the intersections occuring with in that face
				int _intersect_counter = 0;
				for (auto intersec_ptr = ray_info_ptr->intersection_vector_t0.begin();
					intersec_ptr != ray_info_ptr->intersection_vector_t0.end(); ++intersec_ptr){
					if (tracked_point_status_vector.at(intersec_ptr->ray_id) == 1){
						intersec_ptr->istracked = true;
						cv::Point2f optic_flow_image_pt = tracked_points_vector.at(intersec_ptr->ray_id);

						cv::Point3f worldPoint = viz_class.transform_image_to_original_frame(optic_flow_image_pt);
						ray_info_ptr->intersection_vector_t.at(_intersect_counter).inversection_position =
							glm::vec3(worldPoint.x, worldPoint.y, worldPoint.z);


					}
					else {
						intersec_ptr->istracked = false;
					}
					_intersect_counter++;
				}
			}

		}


		//Set sudo forces
		std::vector<int> force_vector_indicies;
		std::vector<std::vector<float>> force_vector;
		//this mapper will find the average displacement of each node.
		//First add all of the different forces that this node is associated with, then divide by the number of forces added.
		std::map<int, std::vector<glm::vec3>> verticies_mapper;
		for (auto ray_ptr = ray_info.begin(); ray_ptr != ray_info.end(); ++ray_ptr){
			//std::vector<float> force_;
			for (int _i = 0; _i < ray_ptr->intersection_vector_t.size(); _i++){

				glm::vec3 force_;
				force_ = ray_ptr->intersection_vector_t.at(_i).inversection_position
					- ray_ptr->intersection_vector_t0.at(_i).inversection_position;

				if (ray_ptr->intersection_vector_t0.at(_i).istracked == true){
					//now loop through all of the verticies
					for (auto vertex_ptr = ray_ptr->indicies.begin(); vertex_ptr != ray_ptr->indicies.end(); ++vertex_ptr){
						if (glm::length<float>(force_) < 25.0) //only smallish deformation is allowed
							verticies_mapper[*vertex_ptr].push_back(force_);
						//	force_vector_indicies.push_back(*vertex_ptr);
						//	force_vector.push_back(force_);
					}
				}
			}

		}

		for (auto verticies_mapper_ptr = verticies_mapper.begin();
			verticies_mapper_ptr != verticies_mapper.end();	++verticies_mapper_ptr){
			force_vector_indicies.push_back(verticies_mapper_ptr->first);

			std::vector<glm::vec3> force_vec = verticies_mapper_ptr->second;
			glm::vec3 force_averaged;
			for (int i = 0; i < force_vec.size(); i++){
				force_averaged += force_vec.at(i);
			}

			force_averaged /= force_vec.size();
			std::vector<float> force_std;
			force_std.push_back(force_averaged.x);
			force_std.push_back(force_averaged.y);
			force_std.push_back(force_averaged.z);
			force_vector.push_back(force_std);

		}







		sim.cuda_tools_class.get_number_sudo_forces(force_vector.size());
		sim.cuda_tools_class.get_sudo_force_information(force_vector, force_vector_indicies);






		viz_class.render();

		sim.run();
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
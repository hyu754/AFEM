#pragma once

#include <iostream>
#include <ctime>
#include "opencv2/opencv_modules.hpp"



#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"

#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "surf.h"

#include "aruco_tools.h"

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/widget_accessor.hpp>
#include <AFEM_geometry.hpp>
#include "raytracer.h"

class viz_tools
{
public:
	/*
	Structure of intersection information
	u,v,w - barycentric coordinate
	intersection_position - the position on the face of intersection
	*/
	struct intersection{
		//barycentric coordinates
		float u, v, w;
		//The position on the face of intersection
		glm::vec3 inversection_position;
	};

	/*
	This struct will store the face_information.
	vector of intersections. Each intersection will have
	*/
	struct face_information{
		//face_id
		int face_id;

		//element_id, which element is it in
		int element_id;

		//vector of intersections at time 0, or the original position
		std::vector<intersection> intersection_vector_t0;
		
		//vector of intersections at time t
		std::vector<intersection> intersection_vector_t;
		
		//Vector of nodal positions, at the original time
		std::vector<glm::vec3> nodal_positions_t0;

		//Vector of nodal positions, at the current time
		std::vector<glm::vec3> nodal_positions_t;

		//Vector of indicies for all 3 nodes;
		std::vector<int> indicies;

		//Clear everything
		void clear(){
			intersection_vector_t0.clear();
			intersection_vector_t.clear();
			nodal_positions_t0.clear();
			nodal_positions_t.clear();
			indicies.clear();
		}
	};


private:


	//Viz camera pointer
	cv::viz::Viz3d *myWindow;

	//Intrinsic matrix for the camera
	cv::Mat camMat;

	//window width and height
	cv::Size size_window;

	//Camera of the viz3d window
	cv::viz::Camera *camera;

	//Camera position matrix
	cv::Affine3f cam_pose;

	//Transformation to make view direction the same
	cv::Affine3f transform;// = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);


	//Temporary pointer to a geometry in the field of view
	cv::viz::Mesh global_mesh_ptr;

	//Ray tracer class
	raytracer ray_tracer_class;

	//Vector of faces
	std::vector<face_information> face_information_vector;

	//vector of intersected faces
	std::vector<face_information> face_information_intersected;
public:

	//Get geometry

	//Create a plane of size
	void create_plane(cv::Vec2d size_plane);
	
	//Set window size
	void set_window_size(cv::Size winSize);

	//Set camera matrix, call after window size has been set
	void set_virtual_camera_intrinsics(void);

	//Set the camera of viz3d window to be user defined camera
	//Note the render must have rendered one scene before this function
	//can be called
	void set_camera(void);

	//Set the camera position with the same focal lengths, etc, from the camera being used
	void set_camera_position(void);

	//Render the current scene to the window
	void render(void){ myWindow->spinOnce(1,false); }

	//Wrapper function to solve for the PnP-problem
	//Inputs: points_3d - 3d points, points_2d - 2d points
	//Outputs: Affine3f matrix (or extrinsic matrix)
	
	cv::Affine3f solve_pnp_matrix(std::vector<cv::Point3f>, std::vector<cv::Point2f>); //Ordered case 
	cv::Affine3f solve_pnp_matrix(std::vector<cv::Point3f>, std::vector<int> indices_3d, std::vector<cv::Point2f>, std::vector<int> indices_2d); //Unordered case

	//Augment mesh function will transform the virtual model onto the camera fram
	//String_name will specify the name of the geometry,
	//Pose, is the new pose of the virtual object from solve_pnp_matrix
	cv::Mat augment_mesh(cv::Mat input_image,std::string object_name,cv::Affine3f pose);

	//Get geometry from AFEM::geometry
	void render_geometry_FEM(std::vector<AFEM::element> geometry, std::vector<AFEM::position_3D> position_vector);

	//Generates the rays used for ray tracing
	void generate_rays(void);

	//Ray tracer
	void ray_tracer(void);

	

	//GLM to CV for 2d
	template<typename T, typename S> 
	std::vector<S> glm_to_cv_2d(std::vector<T>);

	//GLM to CV for 3d
	template<typename T, typename S>
	std::vector<S> glm_to_cv_3d(std::vector<T>);


	//constructors
	viz_tools();
	viz_tools(std::string window_name);
	~viz_tools();
};




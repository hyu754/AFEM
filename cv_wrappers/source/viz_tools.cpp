#include "viz_tools.h"
#include "viz_widget.h"
#include "glm\glm.hpp"
#include <fstream>
viz_tools::viz_tools(){
	myWindow = new cv::viz::Viz3d("default window");
}

viz_tools::viz_tools(std::string window_name){
	myWindow = new cv::viz::Viz3d(window_name);
	
}

void viz_tools::create_plane(cv::Vec2d size_plane){

	cv::viz::Mesh plane;
	std::vector<	cv::Vec3d> points;
	std::vector<int> polygons;

	points.push_back(cv::Vec3d(0, 0, 0));
	points.push_back(cv::Vec3d(0, size_plane(1), 0));
	points.push_back(cv::Vec3d(size_plane(0), 0, 0));
	points.push_back(cv::Vec3d(size_plane(0), size_plane(1), 0));


	polygons.push_back(4);
	polygons.push_back(3);
	polygons.push_back(2);
	polygons.push_back(1);
	polygons.push_back(0);



	plane.cloud = cv::Mat(points, true).reshape(3, 1);
	plane.polygons = cv::Mat(polygons, true).reshape(1, 1);

	plane.colors = cv::viz::Color::blue();
	
	myWindow->showWidget("plane", cv::viz::WMesh(plane));
	myWindow->setRenderingProperty("plane", cv::viz::LINE_WIDTH, 4);

	std:vector<cv::viz::WLine > line_vector;
	line_vector.push_back(cv::viz::WLine(points.at(3), points.at(2), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(2), points.at(1), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(1), points.at(0), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(0), points.at(3), cv::viz::Color::orange()));



	int _line_pos = 0;
	/*for (auto line_ptr = line_vector.begin(); line_ptr != line_vector.end(); ++line_ptr){
		line_ptr->setRenderingProperty(cv::viz::LINE_WIDTH, 4);
		myWindow->showWidget(std::to_string(_line_pos), cv::viz::WLine(*line_ptr));
		_line_pos++;
	}*/
	
}



void viz_tools::set_window_size(cv::Size winSize){
	myWindow->setWindowSize(winSize);
	size_window = winSize;
}

void viz_tools::set_virtual_camera_intrinsics(void){
	//focal length
	float f = cv::max(size_window.width,size_window.height);

	float cx = size_window.width / 2.0f;
	float cy = size_window.height / 2.0f;

	 
	camMat = (cv::Mat_<float>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);

}

void viz_tools::set_camera(void){

	float fx = camMat.at<float>(0, 0);
	float fy = camMat.at<float>(1, 1);
	float cx = camMat.at<float>(0, 2);
	float cy = camMat.at<float>(1, 2);
	//initialize the camera with predetermined camera matrix
	camera = new cv::viz::Camera(fx, fy, cx, cy, size_window);

	//set the window camera to be the camera above
	//Note: the renderer must have rendered one scene
	myWindow->spinOnce();
	myWindow->setCamera(*camera);

}

void viz_tools::set_camera_position(void){

	float f = cv::max(size_window.width, size_window.height);
	float cx = size_window.width / 2.0f;
	float cy = size_window.height / 2.0f;

	
	cv::Vec3f cam_pos(cx, cy, f);
	cv::Vec3f cam_focal_point(cx, cy, 0);
	cv::Vec3f cam_y_dir(-1.0, 0.0, 0.0f);
	cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

	transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f, -1.0f, 0.0f), cv::Vec3f(-1.0f, 0.0f, 0.0f), cv::Vec3f(0.0f, 0.0f, -1.0f), cam_pos);
	myWindow->setViewerPose(cam_pose);
}


cv::Affine3f viz_tools::solve_pnp_matrix(std::vector<cv::Point3f>_p3d /*3d points*/, std::vector<cv::Point2f>/*3d points*/_p2d){
	cv::Mat rout = cv::Mat::zeros(1, 3, CV_32F);
	cv::Mat tout = cv::Mat::zeros(1, 3, CV_32F);

	cv::solvePnP( _p2d, _p3d,camMat, cv::Mat(), rout, tout);

	/// Construct pose

	std::vector < float> rout_temp;
	rout.copyTo(rout_temp);
	cv::Mat rot_vec = cv::Mat::zeros(1, 3, CV_32F);
	rot_vec.at<float>(0, 0) = rout_temp.at(0);
	rot_vec.at<float>(0, 1) = rout_temp.at(1);
	rot_vec.at<float>(0, 2) = rout_temp.at(2);

	std::vector<float> tout_temp;
	tout.copyTo(tout_temp);


	//std::cout << temp_vector << std::endl;
	//tout.copyTo(rot_vec)
	cv::Mat rot_mat = cv::Mat::Mat(3, 3, CV_32F);
	cv::Rodrigues(rot_vec, rot_mat);


	cv::Affine3f pose(rot_mat, cv::Vec3f(tout_temp.at(0), tout_temp.at(1), tout_temp.at(2)));
	
	return pose;

}


cv::Affine3f viz_tools::solve_pnp_matrix(
	std::vector<cv::Point3f>_p3d_unordered /*3d points*/,
	std::vector<int> _i3 /*3d indicies*/,
	std::vector<cv::Point2f>/*3d points*/_p2d_unordered,
	std::vector<int> _i2 /*2d indicies*/)
{
	cv::Mat rout = cv::Mat::zeros(1, 3, CV_32F);
	cv::Mat tout = cv::Mat::zeros(1, 3, CV_32F);


	std::vector<cv::Point3f> _p3d;
	std::vector<cv::Point2f> _p2d;

	//order the unordered vectors 3d
	for (auto _i3_ptr = _i3.begin(); _i3_ptr != _i3.end(); ++_i3_ptr){
		_p3d.push_back(_p3d_unordered.at(*_i3_ptr));
	}

	//order the unordered vectors 2d
	for (auto _i2_ptr = _i2.begin(); _i2_ptr != _i2.end(); ++_i2_ptr){
		_p2d.push_back(_p2d_unordered.at(*_i2_ptr));
	}

	cv::solvePnP( _p3d, _p2d,camMat, cv::Mat(), rout, tout);

	/// Construct pose

	std::vector < float> rout_temp;
	rout.copyTo(rout_temp);
	cv::Mat rot_vec = cv::Mat::zeros(1, 3, CV_32F);
	rot_vec.at<float>(0, 0) = rout_temp.at(0);
	rot_vec.at<float>(0, 1) = rout_temp.at(1);
	rot_vec.at<float>(0, 2) = rout_temp.at(2);

	std::vector<float> tout_temp;
	tout.copyTo(tout_temp);


	//std::cout << temp_vector << std::endl;
	//tout.copyTo(rot_vec)
	cv::Mat rot_mat = cv::Mat::Mat(3, 3, CV_32F);
	Rodrigues(rot_vec, rot_mat);

	cv::Affine3f pose(rot_mat, cv::Vec3f(tout_temp.at(0), tout_temp.at(1), tout_temp.at(2)));
	pose_affine = pose;
	return pose;

}

cv::Mat viz_tools::augment_mesh(cv::Mat input_image,std::string object_name, cv::Affine3f pose){
	pose = transform*pose;
	//Affine3f cloud_pose_global = transform * cloud_pose;
	myWindow->setWidgetPose("geometryline", pose);
	//myWindow->setWidgetPose("geometry", pose);
	//myWindow->setWidgetPose("geometryline", pose);
	if (stationary_exist ==true)
		myWindow->setWidgetPose("stationary_cloud", pose);
	
	if (geometry_display_bool["geometry_surface"] == true)
		myWindow->setWidgetPose("geometry_surface", pose);

	/*myWindow->setRenderingProperty("stationary_cloud", cv::viz::RenderingProperties::POINT_SIZE, 3.0);
	*/
	/*try{
		myWindow->setWidgetPose("cloud", pose);
		myWindow->setRenderingProperty("cloud", cv::viz::RenderingProperties::POINT_SIZE, 10.0);

	}
	catch(...){
		std::cout << "no cloud" << std::endl;
	}*/
	
	cv::Mat screenshot = myWindow->getScreenshot();
	
	//Perform ray tracing
	//ray_tracer();

	//myWindow->setWidgetPose("cloud", pose);
	
	//myWindow->removeAllLights();
	float cx = size_window.width / 2.0f;
	float cy = size_window.height / 2.0f;


	cv::Vec3f cam_pos(cx, cy, 640.0);
	cv::Vec3f cam_focal_point(cx, cy, 0);
	myWindow->addLight( cam_focal_point, cam_pos,cv::viz::Color::blue());
	//copy the color image with binary image as mask
	//screenshot.copyTo(screenshot, mask);


	cv::imshow("screenshot", screenshot);
	cv::Mat screenshot_gray;
	cv::cvtColor(screenshot, screenshot_gray, cv::COLOR_RGBA2GRAY);

	cv::Mat mask,mask_inv;
	cv::threshold(screenshot_gray, mask, 0.0001, 255, cv::THRESH_BINARY);
	cv::bitwise_not(mask, mask_inv );
	//backout the image in input image;
	//cv::GaussianBlur(mask_inv, mask_inv, cv::Size(5, 5), 2.0,2.0);
	cv::cvtColor(mask_inv, mask_inv, CV_GRAY2RGB);
	cv::cvtColor(mask, mask, CV_GRAY2BGR);
	cv::Mat input_image_not;
	cv::bitwise_and(input_image, mask, input_image_not);
	cv::bitwise_and(input_image, mask_inv, input_image);

	
	
	cv::bitwise_and(screenshot, mask, screenshot);
	cv::Mat dst;

	cv::imshow("input_image_from_VIZ", input_image);
	cv::imshow("mask", mask);
	cv::addWeighted(input_image_not, 0.4, screenshot, 0.6, 0, dst);
	cv::imshow("dst_dst", dst);
	dst = input_image + dst;
	
	
	return dst;
}

#ifdef AFEM_USE
//
//void viz_tools::render_stationary_FEM(
//	std::vector<AFEM::position_3D> pos_vec,
//	std::vector<AFEM::stationary> statvec,
//	float size,
//	std::string object_name,
//	cv::Scalar color)
//{
//
//	/*first change from afem to std*/
//	std::vector<cv::Point3f> stat_vec_cv;
//	for (auto iter = statvec.begin(); iter != statvec.end(); ++iter){
//		stat_vec_cv.push_back(cv::Point3f(pos_vec.at(iter->node_number).x, pos_vec.at(iter->node_number).y,
//			pos_vec.at(iter->node_number).z));
//
//
//	}
//
//
//	render_point_cloud(stat_vec_cv, object_name, size, color);
//	
//}

void viz_tools::render_geometry_FEM(std::vector<AFEM::element> element_vector,std::vector<AFEM::position_3D> position_vector, bool display_face){
	cv::viz::Mesh geometry;
	//cv::viz::Mesh geometry_surface; //This mesh is for visualizing some textures on the face 
	std::vector<cv::Vec3d> points;
	std::vector<cv::Point3f> points3f_;
	std::vector<glm::vec3> points_glm;
	std::vector<int> polygons;

	
	//assign nodes first
	for (auto node_ptr = position_vector.begin(); node_ptr != position_vector.end(); ++node_ptr){

		cv::Vec3d _pos_(node_ptr->x, node_ptr->y, node_ptr->z);
		points.push_back(_pos_);
		points3f_.push_back(cv::Point3f(_pos_));
	}


	std:vector<cv::viz::WLine > line_vector;
		
		int _element_counter_ = 0;
		int _face_counter_ = 0;
		//a vector to store existing combinations
		std::vector<std::vector<int>> non_repeated_indicies;
		for (auto element_ptr = element_vector.begin(); element_ptr != element_vector.end(); ++element_ptr){
			int indicies[4] = {
				element_ptr->nodes_in_elem[0],
				element_ptr->nodes_in_elem[1],
				element_ptr->nodes_in_elem[2],
				element_ptr->nodes_in_elem[3]
			};

			/*
			Possible loops:
			012
			031
			132
			023
			*/
			//Test for existence
			//bool exists = false;
			//for (auto index_ptr = non_repeated_indicies.begin(); index_ptr != non_repeated_indicies.end(); ++index_ptr){
			//	//If similar elements includes 3 elements then we will assume it does exist
			//	int sum = 0;
			//	for (auto index_ptr_ptr = index_ptr->begin(); index_ptr_ptr != index_ptr->end(); ++index_ptr_ptr){
			//		if ((*index_ptr_ptr == indicies[0]) || (*index_ptr_ptr == indicies[1]) || (*index_ptr_ptr == indicies[2])){
			//			sum++;
			//		}
			//	}

			//	if (sum == 3){
			//		exists = true;
			//		break;
			//	}
			//}
			//if (exists == false){
			//	polygons.push_back(3);
			//	polygons.push_back(indicies[0]);
			//	polygons.push_back(indicies[1]);
			//	polygons.push_back(indicies[2]);
			//}
			face_information f_info;

			//Initializing the face_information_vector
			/*node 1*/
			f_info.face_id = _face_counter_;
			f_info.element_id = _element_counter_;
			f_info.indicies.push_back(indicies[0]);
			f_info.indicies.push_back(indicies[1]);
			f_info.indicies.push_back(indicies[2]);

			face_information_vector.push_back(f_info);
			f_info.clear();
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[2]);




			/*node 2*/
			_face_counter_++;
			f_info.face_id = _face_counter_;
			f_info.element_id = _element_counter_;
			f_info.indicies.push_back(indicies[0]);
			f_info.indicies.push_back(indicies[3]);
			f_info.indicies.push_back(indicies[1]);

			face_information_vector.push_back(f_info);
			f_info.clear();
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[1]);

			/*node 3*/
			_face_counter_++;
			f_info.face_id = _face_counter_;
			f_info.element_id = _element_counter_;
			f_info.indicies.push_back(indicies[1]);
			f_info.indicies.push_back(indicies[3]);
			f_info.indicies.push_back(indicies[2]);
			face_information_vector.push_back(f_info);
			f_info.clear();
			polygons.push_back(3);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[2]);


			/*node 4*/
			_face_counter_++;
			f_info.face_id = _face_counter_;
			f_info.element_id = _element_counter_;
			f_info.indicies.push_back(indicies[0]);
			f_info.indicies.push_back(indicies[2]);
			f_info.indicies.push_back(indicies[3]);
			face_information_vector.push_back(f_info);
			f_info.clear();
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[2]);
			polygons.push_back(indicies[3]);


			_face_counter_++;


	
			
			//f_info.face_id = face_information_vector.size();
			//f_info.nodal_positions_t0.push_back(points_glm.at(indicies[0]));


			/*polygons.push_back(indicies[3]);*/
			

			
			



			//line_vector.push_back(cv::viz::WLine(points.at(indicies[0]), points.at(indicies[1]), cv::viz::Color::orange()));
			//line_vector.push_back(cv::viz::WLine(points.at(indicies[1]), points.at(indicies[2]), cv::viz::Color::orange()));
			//line_vector.push_back(cv::viz::WLine(points.at(indicies[2]), points.at(indicies[0]), cv::viz::Color::orange()));

			//		WTriangle tw(
			//		cv::Point3f(element_ptr->position_info[0].x, element_ptr->position_info[0].y, element_ptr->position_info[0].z),
			//		cv::Point3f(element_ptr->position_info[1].x, element_ptr->position_info[1].y, element_ptr->position_info[1].z),
			//		cv::Point3f(element_ptr->position_info[2].x, element_ptr->position_info[2].y, element_ptr->position_info[2].z),
			//		cv::viz::Color::red());
			//		WTriangle lll(
			//		cv::Point3f(element_ptr->position_info[0].x, element_ptr->position_info[0].y, element_ptr->position_info[0].z),
			//		cv::Point3f(element_ptr->position_info[1].x, element_ptr->position_info[1].y, element_ptr->position_info[1].z),
			//		cv::Point3f(element_ptr->position_info[2].x, element_ptr->position_info[2].y, element_ptr->position_info[2].z),
			//		cv::viz::Color::red());
			////WTetraHedron zzz(points3f_.at(element_ptr->nodes_in_elem[0]), points3f_.at(element_ptr->nodes_in_elem[1]), points3f_.at(element_ptr->nodes_in_elem[2]), points3f_.at(element_ptr->nodes_in_elem[3]));
			////WTetraHedronLine zzzline(points3f_.at(element_ptr->nodes_in_elem[0]), points3f_.at(element_ptr->nodes_in_elem[1]), points3f_.at(element_ptr->nodes_in_elem[2]), points3f_.at(element_ptr->nodes_in_elem[3]));
			///// Show widget in the visualizer window
			//myWindow->showWidget("triangle" + std::to_string(_element_counter_), lll);
			//myWindow->setRenderingProperty("triangle" + std::to_string(_element_counter_), cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
			////	myWindow->showWidget("line" + std::to_string(_element_counter_), zzzline);
			
			_element_counter_++;
			//myWindow.showWidget("line" + std::to_string(i), lll);





		}
		//points3f_.at(0).x = points3f_.at(0).x + 0.1;
		geometry.cloud = cv::Mat(points3f_, true).reshape(3, 1);
		geometry.polygons = cv::Mat(polygons, true).reshape(1, 1);
		cv::viz::WMesh mesh_geometry = cv::viz::WMesh(geometry);
		cv::viz::WMesh mesh_geometry_line = cv::viz::WMesh(geometry);
		mesh_geometry.setColor(cv::viz::Color::blue());
		myWindow->setBackgroundColor();// cv::viz::Color::white());//cv::viz::Color::white()
		
		mesh_geometry_line.setColor(cv::viz::Color::orange_red());
		//mesh_geometry_line.
		myWindow->showWidget("geometryline", mesh_geometry_line);
		geometry_mapper["geometryline"] = geometry;
		if (display_face == true){
			myWindow->showWidget("geometry", mesh_geometry);
			myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::OPACITY, 0.5);
			geometry_mapper["geometry"] = geometry;
		}
		myWindow->setRenderingProperty("geometryline", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
		myWindow->setRenderingProperty("geometryline", cv::viz::RenderingProperties::SHADING, cv::viz::ShadingValues::SHADING_FLAT);
		//	myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
		myWindow->setRenderingProperty("geometryline", cv::viz::RenderingProperties::LINE_WIDTH,1);
		//myWindow->removeAllLights();
		/*cv::Vec3f cam_pos(400, 400 , 640.0);
		cv::Vec3f cam_focal_point(400, 400 , 0);
		myWindow->addLight(cam_focal_point, cam_pos, cv::viz::Color::blue());*/
		myWindow->setBackgroundColor(cv::viz::Color::celestial_blue());
		

		global_mesh_ptr = geometry;
		

		//Add geometries to mapper for easy access

		
	//WTetraHedron zzz;
	//myWindow->showWidget("triangle" + std::to_string(1), zzz);
	/*WTetraHedron zzz(
		cv::Point3f(0,0,0),
		cv::Point3f(0, 0, 0),
		cv::Point3f(0, 0, 0),
		cv::viz::Color::red());
	myWindow->showWidget("triangle" + std::to_string(1), zzz);*/
	//int _line_pos = 0;
	//for (auto line_ptr = line_vector.begin(); line_ptr != line_vector.end(); ++line_ptr){
	//	line_ptr->setRenderingProperty(cv::viz::LINE_WIDTH, 4);
	//	//myWindow->showWidget(std::to_string(_line_pos), cv::viz::WLine(*line_ptr));
	//	_line_pos++;
	//}

	/*geometry.cloud = Mat(points, true).reshape(3, 1);
	geometry.polygons = Mat(polygons, true).reshape(1, 1);


	myWindow->showWidget("geometry", cv::viz::WMesh(geometry));*/

}


void viz_tools::render_geometry_surface_FEM(std::vector<AFEM::element> element_vector, std::vector<AFEM::position_3D> position_vector){
	cv::viz::Mesh geometry;
	//cv::viz::Mesh geometry_surface; //This mesh is for visualizing some textures on the face 
	std::vector<cv::Vec3d> points;
	std::vector<cv::Point3f> points3f_;
	std::vector<glm::vec3> points_glm;
	std::vector<int> polygons;
	std::vector<cv::Vec2d> tcoords;
	float max_x = -INFINITY;
	float max_y = -INFINITY;
	//assign nodes first
	for (auto node_ptr = position_vector.begin(); node_ptr != position_vector.end(); ++node_ptr){

		cv::Vec3d _pos_(node_ptr->x, node_ptr->y, node_ptr->z);
		if (node_ptr->x > max_x){
			max_x = node_ptr->x;
		}
		if (node_ptr->y > max_y){
			max_y = node_ptr->y;
		}
		points.push_back(_pos_);
		points3f_.push_back(cv::Point3f(_pos_));
	}
	for (auto node_ptr = position_vector.begin(); node_ptr != position_vector.end(); ++node_ptr){

		tcoords.push_back(cv::Vec2d(node_ptr->x / max_x, node_ptr->y / max_y));
	

	}


	

	//Texture coordinate variables
	


	std:vector<cv::viz::WLine > line_vector;

	int _element_counter_ = 0;
	int _face_counter_ = 0;
	//a vector to store existing combinations
	std::vector<std::vector<int>> non_repeated_indicies;
	for (auto element_ptr = element_vector.begin(); element_ptr != element_vector.end(); ++element_ptr){
		int indicies[4] = {
			element_ptr->nodes_in_elem[0],
			element_ptr->nodes_in_elem[1],
			element_ptr->nodes_in_elem[2],
			element_ptr->nodes_in_elem[3]
		};

		/*node 1*/

		if ((points[indicies[0]].val[2] > 7.9) &(points[indicies[1]].val[2] > 7.9)&(points[indicies[2]].val[2] > 7.9)){
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[2]);

		/*	tcoords.push_back(cv::Vec2d(points[indicies[0]].val[0] / max_x, points[indicies[0]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[1]].val[0] / max_x, points[indicies[1]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[2]].val[0] / max_x, points[indicies[2]].val[1] / max_y));
*/
		}




		/*node 2*/
		if ((points[indicies[0]].val[2] > 7.9) &(points[indicies[3]].val[2] > 7.9)&(points[indicies[1]].val[2] > 7.9)){
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[1]);
		/*	tcoords.push_back(cv::Vec2d(points[indicies[0]].val[0] / max_x, points[indicies[0]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[3]].val[0] / max_x, points[indicies[3]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[1]].val[0] / max_x, points[indicies[1]].val[1] / max_y));
*/
		}

		/*node 3*/
		if ((points[indicies[1]].val[2] > 7.9) &(points[indicies[3]].val[2] > 7.9)&(points[indicies[2]].val[2] > 7.9)){
			polygons.push_back(3);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[2]);
			/*tcoords.push_back(cv::Vec2d(points[indicies[1]].val[0] / max_x, points[indicies[1]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[3]].val[0] / max_x, points[indicies[3]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[2]].val[0] / max_x, points[indicies[2]].val[1] / max_y));
*/
		}

		/*node 4*/
		if ((points[indicies[0]].val[2] > 7.9) &(points[indicies[1]].val[2] > 7.9)&(points[indicies[2]].val[3] > 7.9)){
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[2]);
			polygons.push_back(indicies[3]);
		/*	tcoords.push_back(cv::Vec2d(points[indicies[0]].val[0] / max_x, points[indicies[0]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[2]].val[0] / max_x, points[indicies[2]].val[1] / max_y));
			tcoords.push_back(cv::Vec2d(points[indicies[3]].val[0] / max_x, points[indicies[3]].val[1] / max_y));
*/
		}

		



	}

	cv::Mat test = cv::imread( "UOA_LOGO.jpg");
	if (polygons.size() != 0){
		geometry.cloud = cv::Mat(points3f_, true).reshape(3, 1);
		geometry.polygons = cv::Mat(polygons, true).reshape(1, 1);
		geometry.tcoords = cv::Mat(tcoords, true).reshape(2, 1);
		geometry.texture = test;
		cv::viz::WMesh mesh_geometry = cv::viz::WMesh(geometry);
		myWindow->showWidget("geometry_surface", cv::viz::WMesh(mesh_geometry));
		myWindow->setRenderingProperty("geometry_surface", cv::viz::SHADING, cv::viz::SHADING_PHONG);

		geometry_mapper["geometry_surface"] = geometry;
		geometry_display_bool["geometry_surface"] = true;
	}
	else{
		geometry_display_bool["geometry_surface"] = false;
	}

}

void viz_tools::update_mesh_position(std::string object_name, std::vector<AFEM::position_3D> new_position){

	cv::viz::Mesh _mesh = geometry_mapper[object_name];
	
	std::vector<cv::Point3f> points3f_;




	//assign nodes first
	for (auto node_ptr = new_position.begin(); node_ptr != new_position.end(); ++node_ptr){

		cv::Vec3d _pos_(node_ptr->x, node_ptr->y, node_ptr->z);
		//points.push_back(_pos_);
		points3f_.push_back(cv::Point3f(_pos_));
	}

	_mesh.cloud = cv::Mat(points3f_, true).reshape(3, 1);
	geometry_mapper[object_name] = _mesh;
	

	cv::viz::WMesh WMesh_ = cv::viz::WMesh(_mesh);
	cv::viz::WMesh WMesh_line = cv::viz::WMesh(_mesh);
	WMesh_.setColor(cv::viz::Color::blue());
	WMesh_line.setColor(cv::viz::Color::cherry());
	myWindow->showWidget(object_name, WMesh_);
	myWindow->showWidget(object_name + "line", WMesh_line);
	myWindow->setRenderingProperty(object_name, cv::viz::RenderingProperties::OPACITY, 1);
	myWindow->setRenderingProperty(object_name + "line", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
	//	myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
	myWindow->setRenderingProperty(object_name + "line", cv::viz::RenderingProperties::LINE_WIDTH, 3);

}

void viz_tools::update_mesh_position(std::string object_name, int numberOfNodes,AFEM::position_3D *new_position,bool display_face){

	cv::viz::Mesh _mesh = geometry_mapper[object_name];

	std::vector<cv::Point3f> points3f_;


	

	//assign nodes first
	for (int i = 0; i < numberOfNodes; i++){

		cv::Vec3d _pos_(new_position[i].x, new_position[i].y, new_position[i].z);
		//points.push_back(_pos_);
		points3f_.push_back(cv::Point3f(_pos_));
	}

	_mesh.cloud = cv::Mat(points3f_, true).reshape(3, 1);
	geometry_mapper[object_name] = _mesh;



	cv::viz::WMesh WMesh_line = cv::viz::WMesh(_mesh);

	WMesh_line.setColor(cv::viz::Color::cherry());
	if (display_face == true){
		cv::viz::WMesh WMesh_ = cv::viz::WMesh(_mesh);
		WMesh_.setColor(cv::viz::Color::blue());
		myWindow->showWidget(object_name, WMesh_);
		myWindow->setRenderingProperty(object_name, cv::viz::RenderingProperties::OPACITY, 1);
	}
	myWindow->showWidget(object_name , WMesh_line);

	myWindow->setRenderingProperty(object_name , cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
	//	myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
	myWindow->setRenderingProperty(object_name , cv::viz::RenderingProperties::LINE_WIDTH, 1);

}
void viz_tools::update_mesh_surface_position(std::string object_name, std::vector<AFEM::position_3D> new_position){

	if (geometry_display_bool[object_name] == true){
		cv::viz::Mesh _mesh = geometry_mapper[object_name];

		std::vector<cv::Point3f> points3f_;




		//assign nodes first
		for (auto node_ptr = new_position.begin(); node_ptr != new_position.end(); ++node_ptr){

			cv::Vec3d _pos_(node_ptr->x, node_ptr->y, node_ptr->z);
			//points.push_back(_pos_);
			points3f_.push_back(cv::Point3f(_pos_));
		}

		_mesh.cloud = cv::Mat(points3f_, true).reshape(3, 1);
		geometry_mapper[object_name] = _mesh;


		cv::viz::WMesh WMesh_ = cv::viz::WMesh(_mesh);
		myWindow->showWidget(object_name, WMesh_);
	}
}


void viz_tools::render_stationary_FEM(std::string geometry_name,std::vector<AFEM::stationary> stationary_vec){
	std::vector<cv::Point3f> cloud_;

	cv::viz::Mesh _mesh = global_mesh_ptr;
	for (auto stationary_ptr = stationary_vec.begin(); stationary_ptr != stationary_vec.end(); ++stationary_ptr){
		cv::Vec3f pos_ = _mesh.cloud.at<cv::Vec3f>((int)stationary_ptr->node_number);
		cloud_.push_back(cv::Point3f(pos_.val[0],pos_.val[1],pos_.val[2]));
	}
	if (cloud_.size() != 0){
		cv::viz::WCloud cloud_temp(cloud_);
		cloud_temp.setColor(cv::viz::Color::magenta());
		myWindow->showWidget(geometry_name, cloud_temp);
		myWindow->setRenderingProperty(geometry_name, cv::viz::RenderingProperties::POINT_SIZE, 15.0);
	}
	else {
		stationary_exist = false;
	}
	
}

void viz_tools::render_tumour_FEM(std::string geometry_name, std::vector<int> tumour_vec){
	std::vector<cv::Point3f> cloud_;

	cv::viz::Mesh _mesh = global_mesh_ptr;
	for (auto iter= tumour_vec.begin(); iter!= tumour_vec.end(); ++iter){
		cv::Vec3f pos_ = _mesh.cloud.at<cv::Vec3f>(*iter);
		cloud_.push_back(cv::Point3f(pos_.val[0], pos_.val[1], pos_.val[2]));
	}
	if (cloud_.size() != 0){
		cv::viz::WCloud cloud_temp(cloud_);
		cloud_temp.setColor(cv::viz::Color::green());
		myWindow->showWidget(geometry_name, cloud_temp);
		myWindow->setRenderingProperty(geometry_name, cv::viz::RenderingProperties::POINT_SIZE, 25);
	}
	

}


void viz_tools::render_tumour_FEM(std::string geometry_name, std::string base_geometry,std::vector<int> tumour_vec){
	std::vector<cv::Point3f> cloud_;

	cv::viz::Mesh _mesh = geometry_mapper[base_geometry];
	for (auto iter = tumour_vec.begin(); iter != tumour_vec.end(); ++iter){
		cv::Vec3f pos_ = _mesh.cloud.at<cv::Vec3f>(*iter);
		cloud_.push_back(cv::Point3f(pos_.val[0], pos_.val[1], pos_.val[2]));
	}
	if (cloud_.size() != 0){
		cv::viz::WCloud cloud_temp(cloud_);
		cloud_temp.setColor(cv::viz::Color::green());
		myWindow->showWidget(geometry_name, cloud_temp);
		myWindow->setRenderingProperty(geometry_name, cv::viz::RenderingProperties::POINT_SIZE, 25);
	}


}
#endif
void viz_tools::generate_rays(std::vector<cv::Point3f> features_in_original_reference){

	int height = size_window.height;
	int width = size_window.width;

	int line_counter = 0;

	std::vector<raytracer::ray_struct> ray_vec;
	for (int i = 0; i < width; i += 40){

		for (int j = 0; j < height; j += 40){
			cv::Point3d p1(i, j, 0);
			cv::Point3d p2(i, j, 1);
			cv::viz::WLine line(p1, p2);
			raytracer::ray_struct _ray_st;
			_ray_st.ray = glm::vec3(0, 0, -1.0);
			_ray_st.start_position = glm::vec3(i, j, 100);
			//myWindow->showWidget(std::to_string(line_counter) + "ray", line);
			line_counter++;
			ray_vec.push_back(_ray_st);
		}
	}
	std::vector<raytracer::ray_struct> ray_vec2;
	raytracer::ray_struct _ray_st;
	std::vector<cv::Point3f> ray_start_vec;
	int _id = 0;
	for (auto feature_ptr = features_in_original_reference.begin(); feature_ptr != features_in_original_reference.end(); ++feature_ptr){
		_ray_st.ray = glm::vec3(0, 0, -1.0);
		_ray_st.start_position = glm::vec3(feature_ptr->x , feature_ptr->y, feature_ptr->z);
		//Used when we are using optic flow
		_ray_st.ray_id = _id;
		ray_start_vec.push_back(cv::Point3f(_ray_st.start_position.x, _ray_st.start_position.y, _ray_st.start_position.z));
		ray_vec2.push_back(_ray_st);
		_id++;
	}
	cv::viz::WCloud cloud(ray_start_vec);
	myWindow->showWidget("cloud_start", cloud);
	
	if (!ray_vec2.empty())
		ray_tracer_class.set_ray_vector(ray_vec2);
	
}
template<typename T, typename S>
std::vector<S> viz_tools::glm_to_cv_2d(std::vector<T> glm_vec){
	std::vector<S> output_;
	for (auto glm_ptr = glm_vec.begin(); glm_ptr != glm_vec.end(); ++glm_ptr){
		S temp_S(glm_ptr->x, glm_ptr->y);
		output_.push_back(temp_S);
	}
	return output_;
}

template<typename T, typename S>
std::vector<S> viz_tools::glm_to_cv_3d(std::vector<T> glm_vec){
	std::vector<S> output_;
	for (auto glm_ptr = glm_vec.begin(); glm_ptr != glm_vec.end(); ++glm_ptr){
		S temp_S(glm_ptr->x, glm_ptr->y, glm_ptr->z);
		output_.push_back(temp_S);
	}
	return output_;
}

cv::Point3f viz_tools::transform_image_to_original_frame(const cv::Point2f t_p){

	cv::Point3f worldPoint = cv::Point3f(t_p.x, t_p.y, 1.0);
	cv::Mat cam_matrix = camMat;
	//cv::Affine3f cam_matrix_affine(cam_matrix);
	//cv::Affine3f pose_inv = (cam_matrix_affine*pose_affine);//
	//pose_inv = pose_inv.inv();
	//worldPoint = pose_inv * worldPoint;

	//cv::Affine3f pose_aff = pose_affine;

	//rot_(3, 3, cv::DataType<double>::type);

	cv::Mat rot_ = cv::Mat(pose_affine.rotation());
	cv::Mat tr_ = cv::Mat(pose_affine.translation());
	cv::Mat uvw = cv::Mat::ones(3, 1, CV_32F);
	cv::Mat t_vec = cv::Mat::ones(3, 1, CV_32F);
	uvw.at<float>(0) = t_p.x;
	uvw.at<float>(1) = t_p.y;
	cv::Mat LHS = rot_.inv()*cam_matrix.inv()*uvw;
	t_vec.at<float>(0) = tr_.at<float>(0);
	t_vec.at<float>(1) = tr_.at<float>(1);
	t_vec.at<float>(2) = tr_.at<float>(2);


	cv::Mat RHS = rot_.inv()*t_vec;

	float s = 8.0 + RHS.at<float>(2, 0);
	s /= LHS.at<float>(2, 0);
	cv::Mat world_point_mat = rot_.inv()*(s*cam_matrix.inv()*uvw - t_vec);
	worldPoint.x = world_point_mat.at<float>(0, 0);
	worldPoint.y = world_point_mat.at<float>(1, 0);
	worldPoint.z = world_point_mat.at<float>(2, 0);

	return worldPoint;


}

void viz_tools::ray_tracer(void){
	cv::viz::Widget geo_temp = myWindow->getWidget("geometry");
	
	cv::viz::WMesh mesh_W = geo_temp.cast<cv::viz::WMesh>();
	cv::viz::WCloud cl = mesh_W.cast<cv::viz::WCloud>();
	/*
	TO DO : PUT BELOW INTO OWN MEMBER FUNCTION
	*/
	cv::viz::Mesh global_mesh_ptr_temporary;

	global_mesh_ptr.cloud.copyTo(global_mesh_ptr_temporary.cloud);
	global_mesh_ptr.polygons.copyTo(global_mesh_ptr_temporary.polygons);
	std::vector<raytracer::ray_struct> original_rays = ray_tracer_class.get_ray_vector_original();
	//std::ofstream node_out("node_out.txt");
	for (int _i = 0; _i < global_mesh_ptr_temporary.cloud.size().width; _i++){
		//std::cout << "Before : " << global_mesh_ptr.cloud.at<cv::Vec3f>(_i)<<std::endl;

		global_mesh_ptr_temporary.cloud.at<cv::Vec3f>(_i) = global_mesh_ptr.cloud.at<cv::Vec3f>(_i);
		//node_out <<
		//	global_mesh_ptr.cloud.at<cv::Vec3f>(_i).val[0] << " " <<
		//	global_mesh_ptr.cloud.at<cv::Vec3f>(_i).val[1] << " " <<
		//	global_mesh_ptr.cloud.at<cv::Vec3f>(_i).val[2] << " " << std::endl;

		//std::cout << "After : " << global_mesh_ptr->cloud.at<cv::Vec3f>(_i) << std::endl;
	}
	//node_out.close();
	//Transform rays from original to pose of the mesh
	//for (auto ray_ptr = original_rays.begin(); ray_ptr != original_rays.end(); ++ray_ptr){
	//	cv::Point3f temp_ray(ray_ptr->ray.x, ray_ptr->ray.y, ray_ptr->ray.z);
	//	cv::Point3f temp_original(ray_ptr->start_position.x, ray_ptr->start_position.y, ray_ptr->start_position.z);

	//	temp_ray = pose*temp_ray;
	//	temp_original = pose*temp_original;

	//	ray_ptr->ray.x = temp_ray.x;
	//	ray_ptr->ray.y = temp_ray.y;
	//	ray_ptr->ray.z = temp_ray.z;

	//}
	//ray_tracer_class.set_ray_vector(original_rays);
	//
	//cv::viz::WMesh mesh_geometry = cv::viz::WMesh(global_mesh_ptr_temporary);
	std::cout << global_mesh_ptr_temporary.polygons.size() << std::endl;
	//myWindow->showWidget("geometry", mesh_geometry);

	//loop through all the polygons
	//vector of positions of elements


	std::vector<std::vector<glm::vec3>> face_verticies;

	std::vector<cv::Point3f> ray_trace_output_cv_delete_this;
	//std::ofstream node_out("node_out.txt");
	for (int face_index = 0; face_index < global_mesh_ptr_temporary.polygons.size().width; face_index){
		std::vector<glm::vec3> face_verticies_one;
		for (int loop = 0; loop < 3; loop++){
			face_index++;
			int idx1 = global_mesh_ptr_temporary.polygons.at<int>(face_index);
			cv::Vec3f pos_ = global_mesh_ptr_temporary.cloud.at<cv::Vec3f>(idx1);
			face_verticies_one.push_back(glm::vec3(pos_.val[0], pos_.val[1], pos_.val[2]));

		/*	node_out <<
				pos_.val[0] << " " <<
				pos_.val[1] << " " <<
				pos_.val[2] << " " << std::endl;*/
			ray_trace_output_cv_delete_this.push_back(cv::Point3f(pos_.val[0], pos_.val[1], pos_.val[2]));
		}

		//glm::vec3 bary_out = ray_tracer_class.ray_tracer_for_ray_vector(face_verticies_one.at(0), face_verticies_one.at(1), face_verticies_one.at(2));
		face_verticies.push_back(face_verticies_one);
		face_index++;

	}

	//node_out.close();
	ray_tracer_class.set_face_vector(face_verticies);
	std::vector<raytracer::intersected_rays> ray_trace_output = ray_tracer_class.ray_tracer_run();

	//change from glm to cv
	std::vector<cv::Point3f> ray_trace_output_cv;// = glm_to_cv_3d<glm::vec3, cv::Point3f>(ray_trace_output);
	for (auto ray_trace_ptr = ray_trace_output.begin(); ray_trace_ptr != ray_trace_output.end(); ++ray_trace_ptr){

		ray_trace_output_cv.push_back(cv::Point3f(ray_trace_ptr->intersected_position.x, ray_trace_ptr->intersected_position.y, ray_trace_ptr->intersected_position.z));
	}
	if (ray_trace_output_cv.size() != 0){
		ray_tracer_class.set_ray_tracer_status(true);//Set the status of ray tracing

		cv::viz::WCloud cloud(ray_trace_output_cv);

		myWindow->showWidget("cloud", cloud);
		myWindow->setRenderingProperty("cloud", cv::viz::RenderingProperties::POINT_SIZE, 10);
	}
//	mesh_W.cast();	cv::viz::Mesh mesh312 =mesh_W.cast<cv::viz::Mesh>();
	face_information_intersected.clear();

	//Use a temporary variable so that the original does not change
	std::vector<face_information> face_information_vector_temporary = face_information_vector;
	for (auto intersected_ptr = ray_trace_output.begin(); intersected_ptr != ray_trace_output.end(); ++intersected_ptr){
		intersection _intersection_;
		_intersection_.inversection_position = intersected_ptr->intersected_position;
		_intersection_.u = intersected_ptr->u;
		_intersection_.v = intersected_ptr->v;
		_intersection_.w = intersected_ptr->w;
		_intersection_.ray_id = intersected_ptr->ray_id;
		face_information_vector_temporary.at(intersected_ptr->face_id).intersection_vector_t0.push_back(_intersection_);
		//_face_.intersection_vector_t = intersected_ptr->intersected_position;

		
		
	}

	//If the face has rays passing through, then we add it to face_information_intersected
	for (auto face_info_ptr = face_information_vector_temporary.begin(); face_info_ptr != face_information_vector_temporary.end(); ++face_info_ptr){
		if (face_info_ptr->intersection_vector_t0.size() != 0){
			face_information_intersected.push_back(*face_info_ptr);
		}
	}
}


cv::Affine3f viz_tools::find_affine_transformation_3d(worldPointVector orig_state, worldPointVector final_state){

	cv::Affine3f final_result;

	cv::Mat matrix_output;
	std::vector<uchar> status;
	if (!orig_state.empty() && !final_state.empty()){
		if (orig_state.size() == final_state.size()){
			cv::estimateAffine3D(orig_state, final_state, matrix_output  ,status);
		}
	}
	else{
		std::cout << "Error : Points must be non-empty" << std::endl;
		return final_result;
	}
	
	for (int i = 0; i < 3; i++){//row
		for (int j = 0; j < 4; j++){ //col
			final_result.matrix.val[j+4*i] = matrix_output.at<double>(i,j);
		} 		
	}

	
	
	return final_result;


}

void viz_tools::render_point_cloud(std::vector<cv::Point3f> pnts_3D, std::string name, float size, cv::Scalar color){

	cv::viz::WCloud cloud_temp(pnts_3D);
	cloud_temp.setColor(cv::viz::Color(color[0], color[1], color[2]));
	
	int id = 0;
	for (auto pnts_ptr = pnts_3D.begin(); pnts_ptr != pnts_3D.end(); ++pnts_ptr){
		cv::viz::WText3D text3d(std::to_string(id), *pnts_ptr,0.001);
		id++;
		myWindow->showWidget(std::to_string(id), text3d);
	}

	
	myWindow->showWidget(name, cloud_temp);
	myWindow->setRenderingProperty(name, cv::viz::RenderingProperties::POINT_SIZE, size);
	
}

worldPointVector viz_tools::transform_points3d(cv::Affine3f t_mat, worldPointVector pnts_3D){
	for (auto iter = pnts_3D.begin(); iter != pnts_3D.end(); ++iter){

		(*iter) = t_mat*(*iter);
	}
	return pnts_3D;
}

void viz_tools::render_point_cloud(std::vector<cv::Point3f> pnts_3D, cv::Affine3f pose, std::string name, float size, cv::Scalar color){

	cv::viz::WCloud cloud_temp(pnts_3D);
	cloud_temp.setColor(cv::viz::Color(color[0], color[1], color[2]));
	
	
	myWindow->showWidget(name, cloud_temp);
	
	myWindow->setRenderingProperty(name, cv::viz::RenderingProperties::POINT_SIZE, size);
	myWindow->setRenderingProperty(name, cv::viz::RenderingProperties::REPRESENTATION, cv::viz::REPRESENTATION_WIREFRAME);

}



viz_tools::~viz_tools(){
	//std::cout << "SURF WRAPPER DESTROYED" << std::endl;
}

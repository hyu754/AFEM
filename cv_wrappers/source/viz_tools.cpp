#include "viz_tools.h"
#include "viz_widget.h"
#include "glm\glm.hpp"
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

	return pose;

}

cv::Mat viz_tools::augment_mesh(cv::Mat input_image,std::string object_name, cv::Affine3f pose){
	pose = transform*pose;
	//Affine3f cloud_pose_global = transform * cloud_pose;
	myWindow->setWidgetPose(object_name, pose);
	//myWindow->setWidgetPose("geometry", pose);


	cv::Mat screenshot = myWindow->getScreenshot();

	//copy the color image with binary image as mask
	//screenshot.copyTo(screenshot, mask);


	cv::imshow("screenshot", screenshot);
	cv::Mat screenshot_gray;
	cv::cvtColor(screenshot, screenshot_gray, cv::COLOR_RGBA2GRAY);

	cv::Mat mask,mask_inv;
	cv::threshold(screenshot_gray, mask, 10, 255, cv::THRESH_BINARY);
	cv::bitwise_not(mask, mask_inv );
	//backout the image in input image;
	cv::cvtColor(mask_inv, mask_inv, CV_GRAY2RGB);
	cv::bitwise_and(input_image, mask_inv, input_image);
	cv::Mat dst;

	cv::imshow("input_image", input_image);
	cv::imshow("mask", mask);
	dst = input_image + screenshot;
	//cv::addWeighted(input_image, 1, screenshot, 1, 0, dst);
	
	return dst;
}

void viz_tools::render_geometry_FEM(std::vector<AFEM::element> element_vector,std::vector<AFEM::position_3D> position_vector){
	cv::viz::Mesh geometry;
	std::vector<cv::Vec3d> points;
	std::vector<cv::Point3f> points3f_;
	std::vector<int> polygons;

	
	//assign nodes first
	for (auto node_ptr = position_vector.begin(); node_ptr != position_vector.end(); ++node_ptr){

		cv::Vec3d _pos_(node_ptr->x, node_ptr->y, node_ptr->z);
		points.push_back(_pos_);
		points3f_.push_back(cv::Point3f(_pos_));
	}


	std:vector<cv::viz::WLine > line_vector;
		
		int _element_counter_ = 0;
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
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[2]);
			/*polygons.push_back(indicies[3]);*/
			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[1]);

			polygons.push_back(3);
			polygons.push_back(indicies[1]);
			polygons.push_back(indicies[3]);
			polygons.push_back(indicies[2]);

			polygons.push_back(3);
			polygons.push_back(indicies[0]);
			polygons.push_back(indicies[2]);
			polygons.push_back(indicies[3]);



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
		mesh_geometry.setColor(cv::viz::Color::orange());
		myWindow->setBackgroundColor();
		
		mesh_geometry_line.setColor(cv::viz::Color::orange());
		//mesh_geometry_line.
		myWindow->showWidget("geometryline", mesh_geometry_line);
		myWindow->showWidget("geometry", mesh_geometry);
		myWindow->setRenderingProperty("geometryline", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
	//	myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::REPRESENTATION, cv::viz::RepresentationValues::REPRESENTATION_WIREFRAME);
		myWindow->setRenderingProperty("geometryline", cv::viz::RenderingProperties::LINE_WIDTH,2);
		//myWindow->setRenderingProperty("geometry", cv::viz::RenderingProperties::OPACITY, 0.5);
		
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


void viz_tools::generate_rays(void){

	int height = size_window.height;
	int width = size_window.width;

	int line_counter = 0;
	for (int i = 0; i < width; i += 50){

		for (int j = 0; j < width; j += 50){
			cv::Point3d p1(i, j, 0);
			cv::Point3d p2(i, j, 600);
			cv::viz::WLine line(p1, p2);
			myWindow->showWidget(std::to_string(line_counter) + "ray", line);
			line_counter++;
		}
	}
	
}

void viz_tools::ray_tracer(void){
	cv::viz::Widget geo_temp = myWindow->getWidget("geometry");
	
	cv::viz::WMesh mesh_W = geo_temp.cast<cv::viz::WMesh>();
	mesh_W.cast();	cv::viz::Mesh mesh312 =mesh_W.cast<cv::viz::Mesh>();
}
viz_tools::~viz_tools(){
	//std::cout << "SURF WRAPPER DESTROYED" << std::endl;
}

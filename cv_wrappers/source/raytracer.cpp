
#include "raytracer.h"
#include "viz_tools.h"

glm::vec3 raytracer::ray_tracer(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 start_ray,glm::vec3 ray){

	//Plane vectors
	glm::vec3 AB = B - A;
	glm::vec3 AC = C - A;

	//Normal vector
	glm::vec3 N = glm::cross(AB, AC);

	float d = -glm::dot(N, A);

	glm::vec3 N1 = glm::cross(AC, N) / (glm::length(N)*glm::length(N));

	glm::vec3 N2 = glm::cross(N, AB) / (glm::length(N)*glm::length(N));

	float d1 = -glm::dot(N1, A);
	float d2 = -glm::dot(N2, A);

	float _det_ = glm::dot(ray, N);
	float t_ = -(d + glm::dot(start_ray, N));
	glm::vec3 P_ = _det_*start_ray + t_*ray;

	float u_ = glm::dot(P_, N1) + _det_*d1;
	float v_ = glm::dot(P_, N2) + _det_*d2;

	float t = t_ / _det_;
	float u = u_ / _det_;
	float v = v_ / _det_;

	glm::vec3 output;
	output.x = u;
	output.y = v;
	output.z = t;

	return output;

}

glm::vec3 raytracer::ray_tracer_for_ray_vector(glm::vec3 A, glm::vec3 B, glm::vec3 C){
	glm::vec3 output;
	int number_intersected = 0;
	float _min_ = INFINITY;
	for (auto ray_vector_ptr = ray_vector.begin(); ray_vector_ptr != ray_vector.end(); ++ray_vector_ptr){
		glm::vec3 ans= ray_tracer(A, B, C, ray_vector_ptr->start_position, ray_vector_ptr->ray);
		if (ans.x*ans.x + ans.y*ans.y < _min_){

			_min_ = ans.x*ans.x + ans.y*ans.y;
			std::cout << "min distance  : " << std::endl;
		}
		if ((ans.x>=0.0f) && (ans.y>=0.0f)&&(ans.x + ans.y <= 1.0f)){
			output = ans;
			number_intersected++;
		}
	}

	std::cout << "Number of intersections of this face : " << number_intersected << std::endl;
	return output;

}

std::vector<raytracer::intersected_rays> raytracer::ray_tracer_run(){

	int number_intersected = 0;
	std::vector<raytracer::intersected_rays> output_vector;
	for (auto ray_vector_ptr = ray_vector.begin(); ray_vector_ptr != ray_vector.end(); ++ray_vector_ptr){
		float _max_ = -INFINITY;
		float _min_ = INFINITY;
		glm::vec3 ray_intersect;
		raytracer::intersected_rays min_point;
		viz_tools::face_information face_information_temp;
		bool found = false;
		int _face_counter_ = 0;
		for (auto face_ptr = faces_vector.begin(); face_ptr != faces_vector.end(); ++face_ptr){
			glm::vec3 A = face_ptr->at(0);
			glm::vec3 B = face_ptr->at(1);
			glm::vec3 C = face_ptr->at(2);
			

			std::vector<glm::vec3> nodes_ABC;
			nodes_ABC.push_back(A);
			nodes_ABC.push_back(B);
			nodes_ABC.push_back(C);
			glm::vec3 ans = ray_tracer(A, B, C, ray_vector_ptr->start_position, ray_vector_ptr->ray);
			
			if ((ans.x >= 0.0f) && (ans.y >= 0.0f) && (ans.x + ans.y <= 1.0f)){
				
				glm::vec3 rep_ = reproject_trace(A, B, C, ans.x, ans.y);
				if (_max_<=  rep_.z){
					
					_max_ = rep_.z;
					min_point.intersected_position = rep_;
					min_point.nodes = nodes_ABC;
					min_point.face_id = _face_counter_;
					min_point.u = ans.x;
					min_point.v = ans.y;
					min_point.w = 1.0f - ans.x - ans.y;
	
					ray_intersect = ans;
				
					number_intersected++;
					//output_vector.push_back(glm::vec3(rep_.x,rep_.y,rep_.z));

				}
			}

			_face_counter_++;
		}

		if (glm::length(ray_intersect) != 0.0){
			
			
			
			output_vector.push_back(min_point);
		}


		
	}

	//std::cout << "Number of intersections of this face : " << number_intersected << std::endl;
	return output_vector;

}


glm::vec3 raytracer::reproject_trace(glm::vec3 A, glm::vec3 B, glm::vec3 C, float u, float v){
	glm::mat3 X(
		A.x, B.x, C.x, 
		A.y, B.y, C.y, 
		A.z, B.z, C.z
		);

	X = glm::transpose(X);
	//barycentric vector
	glm::vec3 n( 1.0f - u - v ,u, v);

	glm::vec3 reprojected_point = X*n;

	return reprojected_point;
	

}
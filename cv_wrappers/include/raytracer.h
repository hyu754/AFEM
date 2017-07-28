
#ifndef RAY_TRACER_H
#define RAY_TRACER_H





#include <iostream>
#include <glm/glm.hpp>
#include <vector>
class raytracer
{
public:
	//A structure to store information on rays
	struct ray_struct{
		glm::vec3 ray;
		glm::vec3 start_position;
	};

	//A structure to store information on intersected rays
	struct intersected_rays{
		int face_id;
		//The nodes in the face, A,B,C
		std::vector<glm::vec3> nodes;

		//intersected position
		glm::vec3 intersected_position;

		//Barycentric coordinates
		float u, v, w;
	};
private:
	//A set of all rays to be used in the ray tracing algorithm
	std::vector<ray_struct> ray_vector, ray_vector_original;

	//Vector of all of the faces to be analysed 
	std::vector<std::vector<glm::vec3>> faces_vector;
	
public:


	//Get all tracing rays
	//Finds the ray has intersected the triangle.
	//Inputs:	Verticies - A,B,C
	//			Ray
	//Output:	glm::vec3 , where first u and v are the first two values, the third is t
	//Where, P = start_ray + t*ray
	glm::vec3 ray_tracer(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 start_ray, glm::vec3 ray);

	//If start_ray and ray are unspecified, then we will use the variable ray_vector
	glm::vec3 ray_tracer_for_ray_vector(glm::vec3 A, glm::vec3 B, glm::vec3 C);

	//If we have both ray_vector and face_vector
	std::vector<intersected_rays> ray_tracer_run(void);

	


	//Finds the point on the face that corresponds to the u and v found using ray_tracer(..)
	//Input:	Verticies - A,B,C
	//			Barycentric- u,v
	//Output:	Position , P, on the face
	glm::vec3 reproject_trace(glm::vec3 A, glm::vec3 B, glm::vec3 C, float u, float v);

	//Sets the ray_vector
	void set_ray_vector(std::vector<ray_struct> ray_in){ ray_vector = ray_in; if (ray_vector_original.size() == 0){ ray_vector_original = ray_in; } }

	//Gets the original set of ray_vectors in its default orientation
	std::vector<ray_struct> get_ray_vector_original(){ return ray_vector_original; }




	//Sets the vector of faces to be analysed
	void set_face_vector(std::vector<std::vector<glm::vec3>> face_vec_in){ faces_vector = face_vec_in; }
private:

};




#endif
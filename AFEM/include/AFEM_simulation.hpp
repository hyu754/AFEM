#include "AFEM_geometry.hpp"
#include "AFEM_cuda.cuh"
#ifndef AFEM_SIMULATION_H
#define AFEM_SIMULATION_H

namespace AFEM{
	class Simulation;
}

class AFEM::Simulation
{
public:
	Simulation(AFEM::Geometry geo_in);
	~Simulation();
	
	//This function will change the element std::vector to an array form for CUDA and then allocate device memory
	void element_std_to_array(void);
	

	//run the loop
	void run(void);
	
	
	//These variables will be populated when the class is initialized
	std::vector<AFEM::position_3D> pos_vec;
	std::vector<AFEM::element> element_vec;
	std::vector<AFEM::stationary> stationary_vec;



	//This will be the vector that will be updated every iteration
	element *element_array;
	//An array to store the node positions that will be stationary
	stationary *stationary_array;
	//An array for the pos_vec
	position_3D *pos_array;
private:
	cuda_tools cuda_tools_class;
	AFEM::Geometry afem_geometry;


	

	

};



#endif AFEM_SIMULATION_H

#ifndef AFEM_SIMULATION_H
#define AFEM_SIMULATION_H
#include "AFEM_geometry.hpp"
#include "AFEM_cuda.cuh"
namespace AFEM{
	class Simulation;
	//solver types 
	

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

	
	//sets the solver type
	void set_solver_type(elastic_solver_type type_input){ solver_type = type_input; }

	//return solver type
	elastic_solver_type get_solver_type(){ return solver_type; }

	//return position vector , this will be updated to the new position every time this->run() is called
	position_3D* get_position_vector(void){ return pos_array; }

	cuda_tools cuda_tools_class;
private:
	
	AFEM::Geometry afem_geometry;


	//Solver type, can be dynamic (with or without corotation), and energy minimisation
	AFEM::elastic_solver_type solver_type;
	

	

};



#endif AFEM_SIMULATION_H
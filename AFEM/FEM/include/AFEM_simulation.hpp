
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

	//Write position at time t to file, this is all of the nodes
	//Input:	file_name - the file plus directory
	void write_position(std::string file_name);
	
	//Write position at time t to file, will only output the ids specified in the 'ids' vector
	//Input:	file_name - the file plus directory
	//			ids		- vector of node ids to writeout
	void write_position(std::string file_name,std::vector<int> ids);

	//These variables will be populated when the class is initialized
	std::vector<AFEM::position_3D> pos_vec;
	std::vector<AFEM::element> element_vec;
	std::vector<AFEM::stationary> stationary_vec;
	std::vector<int> tumour_vec;
	std::vector<int> side_constraint_vec;


	//This will be the vector that will be updated every iteration
	element *element_array;
	//An array to store the node positions that will be stationary
	stationary *stationary_array;
	//An array for the pos_vec
	position_3D *pos_array;
	
	//An array of the initial positions
	position_3D *intial_pos_array;

	
	//sets the solver type
	void set_solver_type(elastic_solver_type type_input){ solver_type = type_input; }

	//return solver type
	elastic_solver_type get_solver_type(){ return solver_type; }

	//return position vector , this will be updated to the new position every time this->run() is called
	position_3D* get_position_vector(void){ return pos_array; }

	cuda_tools cuda_tools_class;

	//Set the young's modulus
	void set_young_modulus(float E_in){ E_young = E_in;}

	//Set poisson's ratio
	void set_poisson_ratio(float nu_in){ nu_poisson = nu_in; }

	//Get young's modulus
	float return_young_modulus(){ return E_young; }

	//Get the poisson's ratio
	float return_poisson_ratio(){ return nu_poisson; }
	
	//Return the iteration number
	long return_iteration_number(){ return iteration_number; }

	//Reset the solver
	void reset_solver(){ cuda_tools_class.reset(); iteration_number = 0; cuda_tools_class.copy_data_from_cuda(element_array, pos_array); }

	
private:
	//Geometry class to store geometry
	AFEM::Geometry afem_geometry;


	//Solver type, can be dynamic (with or without corotation), and energy minimisation
	AFEM::elastic_solver_type solver_type;
	
	
	//Young's modulus and Poisson's ratio for homogeneous material
	float E_young = 15000.0f;
	float nu_poisson = 0.493f;
	
	//this is the iteration number and will be incremented by 1 after each run
	long iteration_number = 0;

};



#endif AFEM_SIMULATION_H
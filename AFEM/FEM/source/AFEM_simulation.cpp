#include <iostream>
#include "AFEM_simulation.hpp"
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#define WRITE_TO_FILE
//#define CPU_CG_SOLVER
//#define MANUAL_FORCE_ENTRY
#ifdef CPU_CG_SOLVER
#include <Eigen\Eigen>
#include <Eigen\IterativeLinearSolvers>
#include <Eigen/Sparse>
#endif




AFEM::Simulation::Simulation(AFEM::Geometry geo_in){
	std::cout << "Geometry added to simulation" << std::endl;
	afem_geometry = geo_in;
	pos_vec = afem_geometry.return_position3D();
	element_vec = afem_geometry.return_element_vector();
	stationary_vec = afem_geometry.return_stationary_vector();
	tumour_vec = afem_geometry.return_tumour_id_vector();
	std::cout << "With " << element_vec.size() << "elements" << " and with " << pos_vec.size() << " nodes" << std::endl;


	cuda_tools_class.initialize_cholesky_variables(pos_vec.size(), element_vec.size(), 3);
	std::cout << sizeof(AFEM::Geometry) << std::endl;

}

AFEM::Simulation::~Simulation(){

	afem_geometry.~Geometry();



}


void AFEM::Simulation::element_std_to_array(){
	element_array = new element[element_vec.size()];
	stationary_array = new stationary[stationary_vec.size()];
	pos_array = new position_3D[pos_vec.size()];
	

	//Populating the element array with AFEM::element structures
	for (int i = 0; i < element_vec.size(); i++){
		element_array[i] = element_vec.at(i);
	}

	for (int i = 0; i < stationary_vec.size(); i++){
		stationary_array[i] = stationary_vec.at(i);
	}

	for (int i = 0; i < pos_vec.size(); i++){
		pos_array[i] = pos_vec.at(i);
	}
	std::cout << "Converted std vectors to arrays " << std::endl;
	//	cuda_tools_class.allocate_CUDA_geometry_data((void**)&element_array, element_vec.size());
	if (afem_geometry.get_dim() == AFEM::dimension::THREE_DIMENSION){
		cuda_tools_class.allocate_copy_CUDA_geometry_data(element_array, stationary_array, pos_array, stationary_vec.size(), element_vec.size(), pos_vec.size(), 3);
	}
	else{
		cuda_tools_class.allocate_copy_CUDA_geometry_data(element_array, stationary_array, pos_array, stationary_vec.size(), element_vec.size(), pos_vec.size(), 2);
	}


	std::cout << "allocated data to GPU memory " << std::endl;

}

bool origional_position_set = false;
double orig_x, orig_y, orig_z;
std::ofstream file_output("tumour_pos.txt");
float sin_in = 0.0f;
void AFEM::Simulation::run(){
	//set corotational bool variable
	if ((solver_type == AFEM::elastic_solver_type::DYNAMIC_COROTATION) || (solver_type == AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION)){
		cuda_tools_class.set_corotational_bool(true);
	}
	else if (solver_type == AFEM::DYNAMIC_NON_COROTATION){
		cuda_tools_class.set_corotational_bool(false);
	}




	cuda_tools_class.make_K(solver_type, element_vec.size(), pos_vec.size());

#ifdef MANUAL_FORCE_ENTRY
	std::vector<int> force_vector_indicies;
	//int dummy_array[8] = { 124, 120, 116, 112, 127, 123, 119, 115 };// , 120  , 116   ,112 };
	int dummy_array[1] = { 166 };
	force_vector_indicies.assign(dummy_array, dummy_array + 1);
	std::vector<float> zero_force;
	zero_force.push_back(0.0f); // x
	zero_force.push_back(0.01 * sin(sin_in));
	zero_force.push_back(0.0f); //y

	sin_in = sin_in + 0.1;
	std::vector<std::vector<float>> force_vector;
	for (int i = 0; i < force_vector_indicies.size(); i++)
		force_vector.push_back(zero_force);
	//force_vector_indicies.push_back(10);
	if ((solver_type == AFEM::elastic_solver_type::DYNAMIC_COROTATION) || (solver_type == AFEM::elastic_solver_type::DYNAMIC_NON_COROTATION)){
	//	cuda_tools_class.make_f(force_vector_indicies, force_vector, pos_vec.size(), 3);
	}
	else if (solver_type == AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION){

		cuda_tools_class.get_number_sudo_forces(force_vector.size());
		cuda_tools_class.get_sudo_force_information(force_vector, force_vector_indicies);
	}

#endif // MANUAL_FORCE_ENTRY

	if ((solver_type == AFEM::DYNAMIC_COROTATION) || (solver_type == AFEM::DYNAMIC_NON_COROTATION)){
		cuda_tools_class.make_f(pos_vec.size(),3);
		cuda_tools_class.dynamic();
	}
	else if (solver_type == AFEM::ENERGY_MINISATION_COROTATION){
		cuda_tools_class.energy_minisation();
	}



#ifdef CPU_CG_SOLVER
	cuda_tools_class.cg_cpu();
#else //else we use CUDA CG
	cuda_tools_class.cg();
#endif
	cuda_tools_class.update_geometry(solver_type);
	cuda_tools_class.copy_data_from_cuda(element_array, pos_array);

	cuda_tools_class.reset_K(element_vec.size(), pos_vec.size());



}

void AFEM::Simulation::write_position(std::string file_name){
	std::ofstream file_out(file_name);
	for (int i = 0; i < afem_geometry.get_num_nodes(); i++){
		file_out << pos_array[i].x << " " << pos_array[i].y << " " << pos_array[i].z << std::endl;
	}

	file_out.close();
	
}




/*

//int node_force = 174;
//if (origional_position_set == false){
//
//
//	orig_x = pos_array[node_force].x;
//	orig_y = pos_array[node_force].y;
//	orig_z = pos_array[node_force].z;
//	origional_position_set = true;
//}
//else {
//	std::cout << "Position considered : " << pos_array[node_force].x-orig_x << " " << pos_array[node_force].y-orig_y << " " << pos_array[node_force].z -orig_z<< std::endl;
//}


//#ifdef WRITE_TO_FILE
//	int node_of_interest = 767;
//	file_output<< pos_array[node_of_interest].x << " " <<pos_array[node_of_interest].y << " " << pos_array[node_of_interest].z <<std::endl;
//#endif
*/



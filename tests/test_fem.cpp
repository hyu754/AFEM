#include <iostream>
#include <time.h>
#include "AFEM_geometry.hpp"
#include "AFEM_simulation.hpp"
#include "AFEM_visualization.hpp"
//#include "AFEM_cuda.cuh"

int main(void){
	
	AFEM::Geometry geo;
	geo.set_dim(AFEM::THREE_DIMENSION);
	geo.read_nodes("FEM_Nodes.txt");
	geo.read_elem("FEM_Elem.txt");
	geo.read_stationary("FEM_Stationary.txt");
	//geo.make_K_matrix();

	AFEM::Simulation sim(geo);
	sim.element_std_to_array();
	sim.set_solver_type(AFEM::elastic_solver_type::ENERGY_MINISATION_COROTATION);


	////Finding which alpha gives the best time
	//float min_time = INFINITY;
	//int min_time_alpha;
	//for (unsigned long long int i = 0; i < 100000000000; i += 100000000){
	//	std::cout << " RUNNING ALPHA : " << i << std::endl;
	//	sim.cuda_tools_class.set_alpha_energy_minisation((float(i)));
	//	//find time for run
	//	float start = clock();
	//	sim.run();
	//	float time_taken = (start - clock()) / CLOCKS_PER_SEC;

	//	if (time_taken < min_time){
	//		min_time = time_taken;
	//		min_time_alpha = i;
	//	}


	//}

	//std::cout << "The alpha that gave the minimum time of : " << min_time << " seconds, was when alpha is : " << min_time_alpha << std::endl;
	//	

	AFEM::Visualization viz(&sim);

	viz.run_visualization();
	
	/*cuda_tools cc;
	cc.hello();*/
	//"FEM_Elem.txt"
	//"FEM_Nodes.txt"
	return 0;
}
#include <iostream>
#include <vector>
//#include "AFEM_cuda.cuh"
#ifndef AFEM_GEOMETRY_H
#define AFEM_GEOMETRY_H

#if 0

#include <stdio.h> 
#include <stdlib.h>
#include <vector>
#include <cassert>
#include <ctime>
#include "fstream"
#include "string"
#include "fstream"
//CUDA
#include <cuda_runtime.h> 
#include <cuda.h>
#include <cusolverSp.h>
#include "device_launch_parameters.h"
#include <cusolverDn.h>
#include <cusparse.h>
#include <cuda_runtime.h> 
#include "Utilities.cuh"
#include "cudaFEM_read.cuh"


#include "cublas_v2.h" 



#endif // 0


//header files



namespace AFEM{
	//Solver types, currently only linear methods are available
	enum elastic_solver_type{ 
		STATIC_NON_COROTATION,
		DYNAMIC_NON_COROTATION, 
		DYNAMIC_COROTATION, 
		ENERGY_MINISATION_COROTATION 
	};
	enum dimension{ TWO_DIMENSION, THREE_DIMENSION };
	//Structures to store geometry information

	struct position_2D{
		double x, y;
	};

	struct position_3D{
		double x=0.0;
		double y=0.0;
		double z=0.0;
		int displacement_index[3];
	};


	//structure for sudo forces
	struct sudo_force_struct
	{
		float fx, fy, fz;
	};


	struct element{
		int nodes_in_elem[4]; // The node numbers that is in the geometry (currently supports only tetra elemnts)
		//Make change so that we do not use t
#if 1
		float volume;
		position_3D position_info[4];

#endif // 0

		//For used during simulation
		float local_K[12 * 12];

		//For used during simulation
		float local_M[12 * 12];

		//local R*Kx ; used for corotational formulation
		float local_RKx[12];

		

		//For used for body forces
		float f_body[12];
		//Jacobian of the element
		float Jacobian;

		//Density of the elemnt
		float density;

	
	};

	//For stationary BC we only need its dof indices
	struct stationary{
		int node_number;
		int displacement_index[3];
	};
	class Geometry;
	

}

class AFEM::Geometry{
	
	//Two/three dimensional vectors
	std::vector<position_3D> position_vector_3D, position_vector_2D;
	
	//Vector for storing the different elements
	std::vector<element> element_vector;


	//Vector of stationary nodes
	std::vector<stationary> stationary_vector;

	//Vector of tumour nodes ids
	std::vector<int> tumour_id;


	//Vector of tumour nodes ids
	std::vector<int> side_constraint_id;

	//Dimensions
	dimension dim;


public:
	Geometry();
	~Geometry();


	//First set dimension of problem. The inputs is enum dimensions (3D supported only at time)
	void set_dim(dimension dim_in){ 
		if (dim_in == dimension::THREE_DIMENSION){
			dim = dim_in; std::cout << "Dimension is: " << 3 << std::endl;
		}
		else{
			std::cout << "Only 3D supported at time " << std::endl;
			std::exit;
		}
	};

	//Returns dimension as AFEM::dimension enum
	AFEM::dimension get_dim(void){ return dim; }

	//This function will be responsible for reading the nodes of the geometry.
	bool read_nodes(std::string s_in);


	//Reading the elements for the geometry
	bool read_elem(std::string s_in);

	//Read in tumour id's
	bool read_tumour(std::string s_in);

	//Read the stationary vector id
	bool read_stationary(std::string s_in);


	//Read the side constraint id
	bool read_side_constraint(std::string s_in);


	void make_K_matrix(void);

	//Return number of nodes
	int get_num_nodes(void){ return position_vector_3D.size(); }


	//return position 
	std::vector<position_3D> return_position3D(){ return position_vector_3D; }

	//return element
	std::vector<element> return_element_vector(){ return element_vector; }

	//return stationary
	std::vector<stationary> return_stationary_vector(){ return stationary_vector; }

	//return tumour id vector
	std::vector<int> return_tumour_id_vector(){ return tumour_id; }

	//return side constraint id vector
	std::vector<int> return_side_constraint_id_vector(){ return side_constraint_id; }

};


#endif //AFEM_GEOMETRY_H


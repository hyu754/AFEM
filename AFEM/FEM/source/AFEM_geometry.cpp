#include <iostream>
#include "AFEM_geometry.hpp"

#include "fstream"
#if 0


#include "string"




#include <stdlib.h>
#include <cuda_runtime.h> 
#include "cublas_v2.h" 
#include <iostream>

#include "fstream"

#include <cuda.h>
#include <cusolverSp.h>
#include "device_launch_parameters.h"
#include <cusolverDn.h>
#include <cusparse.h>
#include <vector>
#include <cassert>
//#include "Utilities.cuh"
#include <ctime>
#include "cuda_functions.cuh"  
#endif // 0

#define max(a,b) ((a) > (b) ? (a) : (b))
#define IDX2C(i,j,ld) (((j)*(ld))+( i )) //first entry is columns and second entry is rows.
#define threeD21D(row_d,col_d,el_d,width_d,depth_d) (row_d+width_d*(col_d+depth_d*el_d))
#define nodesinelemX(node,el,nodesPerElem) (node + nodesPerElem*el)
#define nodesDisplacementX(dof,node,dimension) (dof + node*dimension)



AFEM::Geometry::Geometry(){
	std::cout << "Geometry Object created" << std::endl;

}

AFEM::Geometry::~Geometry(){
}

bool AFEM::Geometry::read_nodes(std::string s_in){
	
	std::ifstream in_matrix(s_in);

	if (!in_matrix){
		std::cout << "cannot open Nodes \n";
		return false;
	}
	int numNodes;
	in_matrix >> numNodes;

	//x = new double[numNodes];
	//y = new double[numNodes];
	//z = new double[numNodes];

	//x_init = new double[numNodes];
	//y_init = new double[numNodes];

	int _displacement_index = 0;
	if (dim == dimension::THREE_DIMENSION){
		position_3D input_3d;
		for (int i = 0; i < numNodes; i++){
			in_matrix >> input_3d.x >> input_3d.y >> input_3d.z;
			for (int k = 0; k < 3; k++){
				input_3d.displacement_index[k] = _displacement_index;
				_displacement_index++;
			}		
			position_vector_3D.push_back(input_3d);
		}
	}


	/*else if (dim == dimension::TWO_DIMENSION){
		for (int i = 0; i < numNodes; i++){
		in_matrix >> x[i] >> y[i];
		x_init[i] = x[i];
		y_init[i] = y[i];
		z[i] = 0;
		}
		}*/

	in_matrix.close();

	//u = new double[numNodes*dim];
	//b_rhs = new float[numNodes*dim];

	return true;



}

bool AFEM::Geometry::read_elem(std::string element_file){

	std::ifstream in_elem(element_file);
	std::cout << "Reading in element files" << std::endl;
	if (!in_elem){
		std::cout << "cannot open Element file \n";
		return false;
	}
	int numE,numNodesPerElem;
	in_elem >> numE >> numNodesPerElem;

	
	for (int e = 0; e < numE; e++) {
		element element_input;
		int element_disp_index = 0; 
		for (int i = 0; i < numNodesPerElem; i++){
			in_elem >> element_input.nodes_in_elem[i];
		}


		element_vector.push_back(element_input);



	}


	in_elem.close();


	//Adding node information to element vector;
	for (auto ele_ptr = element_vector.begin(); ele_ptr != element_vector.end();++ele_ptr) {
		for (int i = 0; i < numNodesPerElem; i++){
			int node_idx =ele_ptr->nodes_in_elem[i];
			position_3D pos_d = position_vector_3D.at(node_idx);
			ele_ptr->position_info[i] = pos_d;
		}
	}


}

bool AFEM::Geometry::read_stationary(std::string s_in){
	std::ifstream station_in(s_in);
	
	if (!station_in){
		std::cout << "cannot open stationary file \n";
		return false;
	}
	int a;
	station_in >> a;
	for (int i = 0; i < a; i++){
		int node;

		//Read in what nodes is stationary
		station_in >> node; 
		stationary stat_d;
		stat_d.node_number = node;
		for (int j = 0; j < 3; j++){
			
			stat_d.displacement_index[j ] = position_vector_3D.at(node).displacement_index[j];
			
		}
		stationary_vector.push_back(stat_d);

	}
	std::cout << "Reading in stationary nodes" << std::endl;
	
}

void AFEM::Geometry::make_K_matrix(){
	//cuda_tools_class.hello();
	//Linear3DBarycentric_B_CUDA_host();
#if 0

	ApplySudoForcesBarycentric(numNodes*dim, sudo_node_force, localcoordForce, elemForce, sudo_force_x, sudo_force_y, f, nodesInElem, thickness, x, y, displaceInElem);

#endif // 0

	/*for (int i = 0; i < numNodes*dim; i++){
	std::cout << f[i] << std::endl;
	}*/
	//std::cout << "FPS time local K matrix: " << duration_K_local << std::endl;
	//std::cout << "FPS time global K matrix: " << duration_K_global << std::endl;
	//std::cout << "sudo force x: " << sudo_force_x << " sudo_force y: " << sudo_force_y << std::endl;
}




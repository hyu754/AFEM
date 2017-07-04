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


#if 0
	for (int e = 0; e < numE; e++) {
		for (int i = 0; i < numNodesPerElem; i++){
			nodesInElem_host[nodesinelemX(i, e, numNodesPerElem)] = nodesInElem[e][i];
			//std::cout << nodesInElem_host[nodesinelemX(i, e, numNodesPerElem)] << std::endl;
		}
		//std::cout << std::endl;

	}

	//cudaMemcpy(nodesInElem_device, nodesInElem_host, numE*numNodesPerElem*sizeof(int), cudaMemcpyHostToDevice);


	std::ifstream in_disp("FEM_displacement.txt");

	if (!in_disp){
		std::cout << "cannot open displacement file \n";
	}
	displaceInElem = new int*[numNodes];
	displaceInElem_host = new int[numNodes*dim];
	displaceInElem_device = new int[numNodes*dim];
	for (int i = 0; i < numNodes; i++){
		displaceInElem[i] = new int[3];

	}
	//cudaMalloc((void**)&displaceInElem_device, numNodes*dim*sizeof(int));

	for (int i = 0; i < numNodes; i++){
		for (int j = 0; j < dim; j++){
			in_disp >> displaceInElem[i][j];

		}

	}

	for (int i = 0; i < numNodes; i++){
		for (int j = 0; j < dim; j++){

			displaceInElem_host[nodesDisplacementX(j, i, dim)] = displaceInElem[i][j];
		}

	}

	cudaMemcpy(displaceInElem_device, displaceInElem_host, numNodes*dim*sizeof(int), cudaMemcpyHostToDevice);
	in_disp.close();
#endif // 0




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
#if 0
void Geometry::Linear3DBarycentric_B_CUDA_host(){

	
	dim3 blocks(84, 196);//numE / (dim)
	dim3 threads(12, 12);
	
	cudaMemcpy(d_x_dist, x, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	cudaMemcpy(d_y_dist, y, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	cudaMemcpy(d_z_dist, z, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	//cudaMemcpy(nodesInElem_device, nodesInElem, numE*numNodesPerElem*sizeof(int), cudaMemcpyHostToDevice);

	int max_limit = (numNodesPerElem*dim*numNodesPerElem*dim*numE);
	int threadsPerBlock = 256;
	int blocksPerGrid = (max_limit + threadsPerBlock - 1) / threadsPerBlock;
	/*cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, 0);*/
	cudaMemset(d_A_dense, 0, numNodes*dim*numNodes*dim*sizeof(*d_A_dense));
	cudaMemcpy(dev_numNodes, &numNodes, 1 * sizeof(int), cudaMemcpyHostToDevice);

	make_K_cuda3d << < 192, 128 >> >(E_vector_device, nodesInElem_device, d_x_dist, d_y_dist, d_z_dist, displaceInElem_device, d_A_dense, dev_numNodes);

	
	//cudaMemcpy(h_A_dense, d_A_dense, numNodes*dim*numNodes*dim*sizeof(*d_A_dense), cudaMemcpyDeviceToHost);
	
	//cudaMemcpy(E_vector_host, E_vector_device, numNodesPerElem*dim*numNodesPerElem*dim*numE*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(x, d_x, numNodes*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(nodesInElem_host, nodesInElem_device, numE*numNodesPerElem*sizeof(int), cudaMemcpyDeviceToHost);

	//std::cout << " K _ CUDA " << std::endl;
	////for (int j = 0; j < 2; j++){
	////	for (int i = 0; i < numNodesPerElem; i++){
	////		std::cout << nodesInElem_host[nodesinelemX(i, j, numNodesPerElem)] << "  ";
	////	}
	////	std::cout << std::endl;
	////}
	//for (int j = 0; j < 10; j++){
	//	for (int i = 0; i < 10; i++){
	//		std::cout << h_A_dense[IDX2C(i, j, 3000)] << "  ";
	//	}
	//	std::cout << std::endl;
	//}



	////Print local K matrix
	//for (int e = 0; e < numE; e++){

	//	//std::cout << "element : " << e << std::endl;
	//	for (int i = 0; i < numNodesPerElem*dim; i++){
	//		for (int j = 0; j < numNodesPerElem*dim; j++){
	//			
	//			//E[e][i][j] = E_vector_host[threeD21D(i, j, e, numNodesPerElem*dim, numNodesPerElem*dim)];
	//			 //std::cout << E[e][i][j] << " ";
	//		}
	//		//std::cout << std::endl;
	//	}
	//	//std::cout << std::endl;
	//}

	//std::cout << std::endl << " the x value : " << x[0] << std::endl;
	/*(cudaMemcpy(&c, dev_c, sizeof(int),
	cudaMemcpyDeviceToHost));
	printf("2 + 7 = %d\n", c);
	(cudaFree(dev_c));*/

}
#endif
#if 0
Geometry::Geometry(){
	std::cout << "Geometry Object created" << std::endl;
	cuda_use = false;
}

Geometry::~Geometry(){
	std::cout << "Geometry Object deleted" << std::endl;

	//deleteing dynamic arrays
	delete[] x;
	delete[] y;
	delete[] z;


	for (int e = 0; e < numE; e++){

		for (int i = 0; i < numNodesPerElem*dim; i++){
			delete E[e][i];
			delete M[e][i];
		}
		delete E[e];
		delete M[e];
		delete nodesInElem[e];
	}

	for (int i = 0; i < numNodes; i++){
		delete displaceInElem[i];

	}






	for (int i = 0; i < numNodes*dim; i++) {
		delete K[i];

	}
	delete[] K;
	delete[] u;
	delete[] f;
	delete[] displaceInElem;
	delete[] E;
	delete[] M;
	delete[] nodesInElem;
	delete[] E_vector_host;
	delete[] elemForce;
	delete[] forceVec_x;
	delete[] forceVec_y;
	delete[] K_vector_form;


	//
	cudaFree(d_A_dense);
	cudaFree(d_nnzPerVector);
	cudaFree(d_A);
	cudaFree(d_A_RowIndices);
	cudaFree(d_A_ColIndices);
	cudaFree(nodesInElem_device);
	cudaFree(d_x);
	cudaFree(d_y);
	cudaFree(d_z);
	cudaFree(E_vector_device);
	free(h_nnzPerVector);

	//free(h_A_dense);


}





void Geometry::read_force(){

	std::ifstream in_matrix("FEM_force.txt");

	if (!in_matrix){
		std::cout << "cannot open force file \n";
	}
	else{
		in_matrix >> numForceBC;
		elemForce = new int[numForceBC];
		localcoordForce = new int[numForceBC];
		forceVec_x = new double[numForceBC];
		forceVec_y = new double[numForceBC];
		for (int i = 0; i < numForceBC; i++){
			in_matrix >> elemForce[i] >> localcoordForce[i] >> forceVec_x[i] >> forceVec_y[i];
		}

	}

	in_matrix.close();



}


void Geometry::initilizeMatrices(){

#if 0
	cudaMalloc((void**)&d_x, numNodes*sizeof(double));
	cudaMalloc((void**)&d_y, numNodes*sizeof(double));
	cudaMalloc((void**)&d_z, numNodes*sizeof(double));
#endif // 0

	cudaMalloc((void**)&d_x_dist, numNodes*sizeof(*d_x_dist));
	cudaMalloc((void**)&d_y_dist, numNodes*sizeof(*d_x_dist));
	cudaMalloc((void**)&d_z_dist, numNodes*sizeof(*d_x_dist));
	K = new double*[numNodes*dim];
	h_A_dense = new float[numNodes*dim*numNodes*dim*sizeof(*h_A_dense)];
	h_M_dense = new double[numNodes*dim*numNodes*dim*sizeof(*h_M_dense)];
	L = new float[numNodes*dim*numNodes*dim*sizeof(*L)];
	//d_A_dense_double = new double[numNodes*dim*numNodes*dim*sizeof(*d_A_dense_double)];
	h_A_dense_double = new double[numNodes*dim*numNodes*dim*sizeof(*h_A_dense_double)];


	gpuErrchk(cudaMalloc((void**)&d_A_dense, numNodes*dim*numNodes*dim* sizeof(*d_A_dense)));
	gpuErrchk(cudaMalloc((void**)&device_L, numNodes*dim*numNodes*dim* sizeof(*device_L)));
	gpuErrchk(cudaMalloc((void**)&d_A_dense_double, numNodes*dim*numNodes*dim* sizeof(*d_A_dense_double)));

	//B = new double*[3];
	for (int i = 0; i < 6; i++){
		//B[i] = new double[3];
	}

	for (int i = 0; i < numNodes*dim; i++) {
		K[i] = new double[numNodes*dim];

	}

	u = new double[numNodes*dim];
	f = new double[numNodes*dim];
	u_dot = new double[numNodes*dim];
	u_doubledot = new double[numNodes*dim];
	u_doubledot_old = new double[numNodes*dim];
	for (int i = 0; i < numNodes*dim; i++){
		f[i] = 0;
	}

}


//void Geometry::call_sudo_force_func(void){
//
//	//call this to apply the sudo forces
//	//ApplySudoForcesBarycentric(numNodes*dim, sudo_node_force, localcoordForce, elemForce, sudo_force_x, sudo_force_y, f, nodesInElem, thickness, x, y, displaceInElem, force_reset);
//
//}
void Geometry::AssembleGlobalElementMatrixBarycentric(int numP, int numE, int nodesPerElem, int **elem, double ***E, double ***M, float *K, double *global_M, int **displaceInElem){
	//cout << numP << endl << endl << endl << endl;


	//Initialising several variables
	int i;
	int j;
	int row;
	int col;

	//Make a numPxnumP matrix all equal to zero
	for (j = 0; j < numP; j++){
		for (i = 0; i < numP; i++){
			K[IDX2C(j, i, numP)] = 0;
			L[IDX2C(j, i, numP)] = 0;
			global_M[IDX2C(j, i, numP)] = 0;
		}
	}
	int dummy_node;
	int loop_node;
	int dummy_row;
	int dummy_col;
	int *DOF = new int[numNodes*dim];
	int counter;

	for (int k = 0; k < numE; k++){
		counter = 0;
		for (int npe = 0; npe < numNodesPerElem; npe++){
			dummy_node = elem[k][npe]; // The row of the matrix we looking at will be k_th element and npe (nodes per element) 	
			for (int dof = 0; dof < dim; dof++){
				row = displaceInElem[dummy_node][dof];
				DOF[counter] = row;
				//cout << DOF[counter] << endl;
				counter++;
			}
		}
		for (int c = 0; c < numNodesPerElem*dim; c++){
			for (int r = 0; r < numNodesPerElem*dim; r++){

				K[IDX2C(DOF[c], DOF[r], numP)] = K[IDX2C(DOF[c], DOF[r], numP)] + E[k][r][c];
				global_M[IDX2C(DOF[c], DOF[r], numP)] = global_M[IDX2C(DOF[c], DOF[r], numP)] + M[k][r][c];
				L[IDX2C(DOF[c], DOF[r], numP)] = (dt*c_xi*beta_1 + dt*dt*beta_2 / 2.0)*K[IDX2C(DOF[c], DOF[r], numP)] + (1 + dt*beta_1*c_alpha)*global_M[IDX2C(DOF[c], DOF[r], numP)]; //
				//K[IDX2C(DOF[r], DOF[c], numP*dim)] = K[IDX2C(DOF[r], DOF[c], numP*dim)] + E[k][r][c];
			}
		}
	}

	//for (int k = 0; k < numE; k++){
	//	counter = 0;
	//	for (int npe = 0; npe < numNodesPerElem; npe++){
	//		dummy_node = elem[k][npe]; // The row of the matrix we looking at will be k_th element and npe (nodes per element) 	
	//		for (int dof = 0; dof < dim; dof++){
	//			row = displaceInElem[dummy_node][dof];
	//			DOF[counter] = row;
	//			//cout << DOF[counter] << endl;
	//			counter++;
	//		}
	//	}
	//	for (int c = 0; c < numNodesPerElem*dim; c++){
	//		for (int r = 0; r < numNodesPerElem*dim; r++){

	//			
	//			L[IDX2C(DOF[c], DOF[r], numP)] = (dt*c_xi*beta_1 + dt*dt*beta_2 / 2.0)*K[IDX2C(DOF[c], DOF[r], numP)] + (1 + dt*beta_1*c_alpha)*global_M[IDX2C(DOF[c], DOF[r], numP)]; //
	//			//K[IDX2C(DOF[r], DOF[c], numP*dim)] = K[IDX2C(DOF[r], DOF[c], numP*dim)] + E[k][r][c];
	//		}
	//	}
	//}

	//for (int k = 0; k < numE; k++){
	//	
	//	for (int c = 0; c < numNodesPerElem*dim; c++){
	//		for (int r = 0; r < numNodesPerElem*dim; r++){

	//			
	//			L[IDX2C(DOF[c], DOF[r], numP)] = (dt*c_xi*beta_1 + dt*dt*beta_2 / 2.0)*K[IDX2C(DOF[c], DOF[r], numP)] + (1 + dt*beta_1*c_alpha)*global_M[IDX2C(DOF[c], DOF[r], numP)]; //
	//			//K[IDX2C(DOF[r], DOF[c], numP*dim)] = K[IDX2C(DOF[r], DOF[c], numP*dim)] + E[k][r][c];
	//		}
	//	}
	//}


	//for (i = 0; i < 10; i++){
	//	for (j = 0; j < 10; j++){
	//		std::cout << global_M[IDX2C(j, i, numP)] << "   ";
	//	}
	//	std::cout << std::endl;
	//}

}
void Geometry::find_b(){
	int du = numNodes*dim;
	double use_number;
	double dummy_row;
	double dummy_row1;
	//I am going to apply a forcef only at the initial time step, and then will be zero.


	for (int i = 0; i < numNodes*dim; i++){
		use_number = 0;
		for (int j = 0; j < numNodes*dim; j++){
			dummy_row = 0;
			dummy_row1 = 0;
			dummy_row = u[j] + dt*u_dot[j] + (dt*dt / 2.0)*(1.0 - beta_2)*u_doubledot[j];
			dummy_row1 = u_dot[j] + dt*(1.0 - beta_1)*u_doubledot[j];
			use_number = use_number + h_A_dense[IDX2C(i, j, du)] * dummy_row + (h_A_dense[IDX2C(i, j, du)] * c_xi + h_M_dense[IDX2C(i, j, du)] * c_alpha)*dummy_row1;
			//use_number = use_number + h_A_dense[IDX2C(i, j, du)] * dummy_row;
		}
		b_rhs[i] = f[i] - use_number;

		if (f[i]>0){
			std::cout << "f" << std::endl;
		}
		//std::cout << b_rhs[i] ;
	}


}

//initializing the dynamic array 
void Geometry::initialize_zerovector(int numberofpoints){
	numNodesZero = numberofpoints;
	vector_zero_nodes = new int[numberofpoints];
}
void Geometry::initialize_dynamic(){

	for (int i = 0; i < numNodes*dim; i++){
		u[i] = u_dot[i] = u_doubledot[i] = u_doubledot_old[i] = 0.0;
	}

}

void Geometry::update_dynamic_vectors(){

	for (int i = 0; i < numNodes*dim; i++){
		u[i] = dt*u_dot[i] + (dt*dt / 2.0)*((1 - beta_2)*u_doubledot_old[i] + beta_2*u_doubledot[i]);
		u_dot[i] = u_dot[i] + dt*((1 - beta_1)*u_doubledot_old[i] + beta_1*u_doubledot[i]);
	}

	//int row1 = displaceInElem[sudo_force_index[0]][0];
	//int row2 = displaceInElem[sudo_force_index[0]][1];
	//u[row1] = sudo_force_value1[0]/100.0;
	//u[row2] = sudo_force_value2[1]/100.0;
}

void Geometry::update_dynamic_xyz(){
	for (int j = 0; j < numNodesZero; j++){
		int row1 = displaceInElem[vector_zero_nodes[j]][0];
		int row2 = displaceInElem[vector_zero_nodes[j]][1];
		u[row1] = 0;
		u[row2] = 0;
	}
	//int row1, row2;
	//for (int i = 0; i < numNodesZero; i++){
	//	row1 = displaceInElem[vector_zero_nodes[i]][0];
	//	row2 = displaceInElem[vector_zero_nodes[i]][1];
	//	
	//	u[row1] = u[row2] = 0;
	//}
	///*for (int i = 0; i < numNodes; i++) {
	// //	u[180] = u[181] = 0.0;

	//	double d = (x_init[i] - x[i])*(x_init[i] - x[i]) + (y_init[i] - y[i])*(y_init[i] - y[i]);
	//	if (d > 0.00001){
	//		u[i * dim] = 0;
	//		u[i * dim + 1] = 0;
	//	}

	//}*/

	//}
	//
	for (int i = 0; i < numNodes; i++) {
		//u[180] = u[181] = 0.0;
		x[i] = x[i] + u[i * dim];
		y[i] = y[i] + u[i * dim + 1];


	}
}
void Geometry::update_vector(){ //solve Ax=b for the dynamics case

	double duration_K;

	this->set_zero_AxB();
	/*
	for (int col = 0; col < Ncols; col++){

	L[IDX2C(col, 0, N)] = 0;
	L[IDX2C(col, 1, N)] = 0;
	L[IDX2C(col,2, N)] = 0;
	L[IDX2C(col, 3, N)] = 0;

	if (dim == 3){
	L[IDX2C(col, 2, N)] = 0;
	}
	}
	L[IDX2C(0, 0, N)] = 1.0;
	L[IDX2C(1, 1, N)] = 1.0;
	L[IDX2C(2, 2, N)] = 1.0;
	L[IDX2C(3, 3, N)] = 1.0;
	if (dim == 3){
	L[IDX2C(2, 2, N)] = 1.0;
	}
	*/



	/*for (int j = 0; j <10; j++){
		for (int i = 0; i < 10; i++){
		std::cout<< L[IDX2C(i, j, N)] << std::endl;
		}
		std::cout << std::endl;
		}*/


	// --- Create device array and copy host array to it

	gpuErrchk(cudaMemcpy(d_A_dense, L, Nrows * Ncols * sizeof(*d_A_dense), cudaMemcpyHostToDevice));

	// --- Descriptor for sparse matrix A




	// --- Device side number of nonzero elements per row

	cusparseSafeCall(cusparseSnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, &nnz));
	// --- Host side number of nonzero elements per row

	gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));

	/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
	for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
	printf("\n");*/

	// --- Device side dense matrix
	gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
	gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));


	cusparseSafeCall(cusparseSdense2csr(handle, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));
	std::clock_t start_K;
	start_K = std::clock();
	// --- Host side dense matrix
	float *h_A = (float *)malloc(nnz * sizeof(*h_A));
	int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
	int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
	gpuErrchk(cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost));
	std::cout << nnz << std::endl;
	/*printf("\nOriginal matrix in CSR format\n\n");
	for (int i = 0; i < 10; ++i) printf("A[%i] = %.0f ", i, h_A[i]); printf("\n");

	printf("\n");
	for (int i = 0; i < (10 + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");

	for (int i = 0; i < 10; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);
	*/
	// --- Allocating and defining dense host and device data vectors

	float *h_x = (float *)malloc(Nrows * sizeof(float));
	/*h_x[0] = 100.0;  h_x[1] = 200.0; h_x[2] = 400.0; h_x[3] = 500.0;*/
	for (int i = 0; i < N; i++){
		h_x[i] = b_rhs[i];
	}
	/*if (dim == 3){
		h_x[0] = h_x[1] = h_x[2] = 0;
		}
		else {
		h_x[0] = h_x[1] = 0;
		h_x[2] = h_x[3] = 0;
		}*/

	float *d_x;        gpuErrchk(cudaMalloc(&d_x, Nrows * sizeof(float)));
	gpuErrchk(cudaMemcpy(d_x, h_x, Nrows * sizeof(float), cudaMemcpyHostToDevice));




	/******************************************/
	/* STEP 1: CREATE DESCRIPTORS FOR L AND U */
	/******************************************/




	/********************************************************************************************************/
	/* STEP 2: QUERY HOW MUCH MEMORY USED IN CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
	/********************************************************************************************************/


	int pBufferSize_M, pBufferSize_L, pBufferSize_Lt;
	cusparseSafeCall(cusparseScsric02_bufferSize(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, &pBufferSize_M));
	cusparseSafeCall(cusparseScsrsv2_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, &pBufferSize_L));
	cusparseSafeCall(cusparseScsrsv2_bufferSize(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, &pBufferSize_Lt));

	int pBufferSize = max(pBufferSize_M, max(pBufferSize_L, pBufferSize_Lt));
	void *pBuffer = 0;  gpuErrchk(cudaMalloc((void**)&pBuffer, pBufferSize));


	/******************************************************************************************************/
	/* STEP 3: ANALYZE THE THREE PROBLEMS: CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
	/******************************************************************************************************/
	int structural_zero;

	cusparseSafeCall(cusparseScsric02_analysis(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));

	cusparseStatus_t status = cusparseXcsric02_zeroPivot(handle, info_A, &structural_zero);
	if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("A(%d,%d) is missing\n", structural_zero, structural_zero); }

	cusparseSafeCall(cusparseScsrsv2_analysis(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
	cusparseSafeCall(cusparseScsrsv2_analysis(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));

	/*************************************/
	/* STEP 4: FACTORIZATION: A = L * L' */
	/*************************************/
	int numerical_zero;

	cusparseSafeCall(cusparseScsric02(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
	status = cusparseXcsric02_zeroPivot(handle, info_A, &numerical_zero);
	/*if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("L(%d,%d) is zero\n", numerical_zero, numerical_zero); }
	*/

	gpuErrchk(cudaMemcpy(h_A, d_A, nnz * sizeof(float), cudaMemcpyDeviceToHost));
	/*printf("\nNon-zero elements in Cholesky matrix\n\n");
	for (int k = 0; k<10; k++) printf("%f\n", h_A[k]);*/

	cusparseSafeCall(cusparseScsr2dense(handle, Nrows, Ncols, descrA, d_A, d_A_RowIndices, d_A_ColIndices, d_A_dense, Nrows));

	/*printf("\nCholesky matrix\n\n");
	for (int i = 0; i < 10; i++) {
	std::cout << "[ ";
	for (int j = 0; j < 10; j++)
	std::cout << h_A_dense[i * Ncols + j] << " ";
	std::cout << "]\n";
	}*/

	/*********************/
	/* STEP 5: L * z = x */
	/*********************/
	// --- Allocating the intermediate result vector
	float *d_z;        gpuErrchk(cudaMalloc(&d_z, N * sizeof(float)));

	const float alpha = 1.;
	cusparseSafeCall(cusparseScsrsv2_solve(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, d_x, d_z, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));

	/**********************/
	/* STEP 5: L' * y = z */
	/**********************/
	// --- Allocating the host and device side result vector
	float *h_y = (float *)malloc(Ncols * sizeof(float));
	float *d_y;        gpuErrchk(cudaMalloc(&d_y, Ncols * sizeof(float)));

	cusparseSafeCall(cusparseScsrsv2_solve(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, d_z, d_y, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));

	cudaMemcpy(h_x, d_y, N * sizeof(float), cudaMemcpyDeviceToHost);
	printf("\n\nFinal result\n");
	/*for (int k = 0; k<20; k++) printf("dx[%i] = %f\n", k, h_x[k]);
	for (int k = 0; k<20; k++) printf("xs[%i] = %f\n", k, x[k]);*/


	for (int i = 0; i < numNodes*dim; i++) {
		u_doubledot_old[i] = u_doubledot[i];
		u_doubledot[i] = h_x[i];



	}

	free(h_A);
	free(h_A_RowIndices);
	free(h_A_ColIndices);
	//free(h_x);
	free(h_y);
	cudaFree(d_x);
	cudaFree(pBuffer);
	cudaFree(d_z);
	cudaFree(d_y);
	duration_K = (std::clock() - start_K) / (double)CLOCKS_PER_SEC;
	//std::cout << " change status : " << changeNode << std::endl;

	//std::cout << "FPS time: " <<1/duration_K << std::endl;

	//std::cout << "Duration: " << duration_K << std::endl;

}

void Geometry::Linear2DBarycentric_B(int *nodes, double *x, double *y, double **term){
	//
	double J = Linear2DJacobianDet_Barycentric(nodes, x, y);

	double y23 = y[nodes[1]] - y[nodes[2]];//y23
	double y31 = y[nodes[2]] - y[nodes[0]];//y31
	double y12 = y[nodes[0]] - y[nodes[1]];//y12
	double x32 = x[nodes[2]] - x[nodes[1]];//x32
	double x13 = x[nodes[0]] - x[nodes[2]];//x13
	double x21 = x[nodes[1]] - x[nodes[0]];//x21
	for (int row = 0; row < 3; row++){
		for (int col = 0; col < 6; col++){
			term[row][col] = 0;
		}
	}

	term[0][0] = term[2][1] = y23 / (J);
	term[0][2] = term[2][3] = y31 / (J);
	term[0][4] = term[2][5] = y12 / (J);
	term[1][1] = term[2][0] = x32 / (J);
	term[1][3] = term[2][2] = x13 / (J);
	term[1][5] = term[2][4] = x21 / (J);

	/*else {
		double **A = new double*[4];
		double **T = new double*[3];
		double **result = new double*[3];
		for (int i = 0; i < 4; i++){
		A[i] = new double[6];
		}
		for (int i = 0; i < 3; i++){
		T[i] = new double[4];
		result[i] = new double[6];
		}

		for (int row = 0; row < 3; row++){
		for (int col = 0; col < 4; col++){
		T[row][col] = 0;
		}
		}

		T[0][0] = T[1][3] = T[2][1] = T[2][2] = 1;
		A[0][1] = A[0][3] = A[0][5] = 0;
		A[1][1] = A[1][3] = A[1][5] = 0;
		A[2][0] = A[2][2] = A[2][4] = 0;
		A[3][0] = A[3][2] = A[3][4] = 0;

		A[0][0] = A[2][1] = y23/J;
		A[0][2] = A[2][3] = y31/J;
		A[0][4] = A[2][5] = y12 / J;
		A[1][0] = A[3][1] = x32 / J;
		A[1][2] = A[3][3] = x13 / J;
		A[1][4] = A[3][5] = x21 / J;
		}
		*/



	//MatrixTimes(term, T, A, 3, 4, 4, 6);

}


void Geometry::Linear3DBarycentric_B(int *nodes, double *x, double *y, double *z, double **term){
	//
	double x14 = x[nodes[0]] - x[nodes[3]];
	double x24 = x[nodes[1]] - x[nodes[3]];
	double x34 = x[nodes[2]] - x[nodes[3]];
	double y14 = y[nodes[0]] - y[nodes[3]];
	double y24 = y[nodes[1]] - y[nodes[3]];
	double y34 = y[nodes[2]] - y[nodes[3]];
	double z14 = z[nodes[0]] - z[nodes[3]];
	double z24 = z[nodes[1]] - z[nodes[3]];
	double z34 = z[nodes[2]] - z[nodes[3]];

	double J = x14*(y24*z34 - y34*z24) - y14*(x24*z34 - z24 * x34) + z14*(x24*y34 - y24*x34);
	double J_bar11 = (y24*z34 - z24*y34) / J;
	double J_bar12 = (z14*y34 - y14*z34) / J;
	double J_bar13 = (y14*z24 - z14*y24) / J;
	double J_bar21 = (z24*x34 - x24*z34) / J;
	double J_bar22 = (x14*z34 - z14*x34) / J;
	double J_bar23 = (z14*x24 - x14*z24) / J;
	double J_bar31 = (x24*y34 - y24*x34) / J;
	double J_bar32 = (y14*x34 - x14*y34) / J;
	double J_bar33 = (x14*y24 - y14*x24) / J;

	/* term[0][0]  = (y24*z34 - z24*y34) / J;
	 term[0][1]= (z14*y34 - y14*z34) / J;
	 term[0][2] =(y14*z24 - z14*y24) / J;
	 term[1][0]= (z24*x34 - x24*z24) / J;
	 term[1][1] = (x14*z34 - z14*x34) / J;
	 term[1][2] = (z14*x24 - x14*z24) / J;
	 term[2][0]= (x24*y34 - y24*x34) / J;
	 term[2][1]= (y14*x34 - x14*y34) / J;
	 term[2][2] = (x14*y24 - y14*x24) / J;*/

	double J_star1 = -(J_bar11 + J_bar12 + J_bar13);
	double J_star2 = -(J_bar21 + J_bar22 + J_bar23);
	double J_star3 = -(J_bar31 + J_bar32 + J_bar33);


	/*double **A = new double*[4];
	double **T = new double*[3];
	double **result = new double*[3];
	for (int i = 0; i < 4; i++){
	A[i] = new double[6];
	}
	for (int i = 0; i < 3; i++){
	T[i] = new double[4];
	result[i] = new double[6];
	}

	for (int row = 0; row < 3; row++){
	for (int col = 0; col < 4; col++){
	T[row][col] = 0;
	}
	}

	T[0][0] = T[1][3] = T[2][1] = T[2][2] = 1;
	A[0][1] = A[0][3] = A[0][5] = 0;
	A[1][1] = A[1][3] = A[1][5] = 0;
	A[2][0] = A[2][2] = A[2][4] = 0;
	A[3][0] = A[3][2] = A[3][4] = 0;

	A[0][0] = A[2][1] = y23/J;
	A[0][2] = A[2][3] = y31/J;
	A[0][4] = A[2][5] = y12 / J;
	A[1][0] = A[3][1] = x32 / J;
	A[1][2] = A[3][3] = x13 / J;
	A[1][4] = A[3][5] = x21 / J;*/

	for (int row = 0; row < 6; row++){
		for (int col = 0; col < 12; col++){
			term[row][col] = 0;
		}

	}
	term[0][0] = term[3][1] = term[5][2] = J_bar11;
	term[1][1] = term[3][0] = term[4][2] = J_bar21;
	term[2][2] = term[5][0] = term[4][1] = J_bar31;

	term[0][3] = term[3][4] = term[5][5] = J_bar12;
	term[1][4] = term[3][3] = term[4][5] = J_bar22;
	term[2][5] = term[4][4] = term[5][3] = J_bar32;

	term[0][6] = term[3][7] = term[5][8] = J_bar13;
	term[1][7] = term[3][6] = term[4][8] = J_bar23;

	term[2][8] = term[4][7] = term[5][6] = J_bar33;

	term[0][9] = term[3][10] = term[5][11] = J_star1;
	term[1][10] = term[3][9] = term[4][11] = J_star2;
	term[2][11] = term[4][10] = term[5][9] = J_star3;

	//for (int row = 0; row < 6; row++){
	//	for (int col = 0; col < 12; col++){
	//		std::cout << term[row][col] << "   ";
	//	}
	//	std::cout << std::endl;
	//}
	//MatrixTimes(term, T, A, 3, 4, 4, 6);

}
double Geometry::Linear2DJacobianDet_Barycentric(int *nodes, double *x, double *y){
	double x13 = x[nodes[0]] - x[nodes[2]];
	double x23 = x[nodes[1]] - x[nodes[2]];
	double y13 = y[nodes[0]] - y[nodes[2]];
	double y23 = y[nodes[1]] - y[nodes[2]];

	return (x13*y23 - y13*x23);

}
double Geometry::Linear3DJacobianDet_Barycentric(int *nodes, double *x, double *y, double *z){
	double x14 = x[nodes[0]] - x[nodes[3]];
	double x24 = x[nodes[1]] - x[nodes[3]];
	double x34 = x[nodes[2]] - x[nodes[3]];
	double y14 = y[nodes[0]] - y[nodes[3]];
	double y24 = y[nodes[1]] - y[nodes[3]];
	double y34 = y[nodes[2]] - y[nodes[3]];
	double z14 = z[nodes[0]] - z[nodes[3]];
	double z24 = z[nodes[1]] - z[nodes[3]];
	double z34 = z[nodes[2]] - z[nodes[3]];

	//std::cout << x14*(y24*z34 - y34*z24) - y14*(x24*z34 - z24 * 34) + z14*(x24*y34 - y24*x34) << std::endl;
	return (x14*(y24*z34 - y34*z24) - y14*(x24*z34 - z24 *x34) + z14*(x24*y34 - y24*x34));



}


void Geometry::Linear2DBarycentric_D(double **term, double nu, double youngE){
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			term[i][j] = 0;
		}
	}

	/*term[0][0] = term[1][1] = 1;

	term[0][1] = term[1][0] = nu;
	term[2][2] = (1 - nu) / 2;*/

#if 1 // for plane stress
	term[0][0] = term[1][1] = 1.0;
	term[0][1] = term[1][0] = nu;
	term[2][2] = (1 - nu) / 2.0;
#endif // 0

#if 0 //for plane strain
	term[0][0] = 1.0;

	term[1][1] = nu;
	term[0][1] = term[1][0] = nu;
	term[2][2] = (1.0 - nu) / 2.0;
#endif // 0

#if 0   //We won't use this here becuase too much floating errors
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			//term[i][j] = (youngE / (1 - nu*nu))*term[i][j];
			//term[i][j] = (youngE / ((1.0 - nu*nu)))*term[i][j]; I will multiply this huge number after the B^T D B
		}
	}
#endif

}


void Geometry::Linear3DBarycentric_D(double **term, double nu, double youngE){
	int multi = 2;
	for (int i = 0; i < 3 * multi; i++){
		for (int j = 0; j < 3 * multi; j++){
			term[i][j] = 0;
		}
	}

	/*term[0][0] = term[1][1] = 1;

	term[0][1] = term[1][0] = nu;
	term[2][2] = (1 - nu) / 2;*/

	term[0][0] = term[1][1] = term[2][2] = (1.0 - nu);
	term[0][1] = term[1][0] = term[0][2] = term[2][0] = term[1][2] = term[2][1] = nu;
	term[3][3] = term[4][4] = term[5][5] = (1.0 - nu) / 2.0;

	for (int i = 0; i < 3 * multi; i++){
		for (int j = 0; j < 3 * multi; j++){
			//term[i][j] = (youngE / (1 - nu*nu))*term[i][j];
			term[i][j] = (youngE / ((1 - 2 * nu)*(1 + nu)))*term[i][j];
		}
	}

	//for (int i = 0; i < 3 * multi; i++){
	//	for (int j = 0; j < 3 * multi; j++){
	//		std::cout << term[i][j] << "   ";
	//	}
	//	std::cout << std::endl;
	//}

}

void Geometry::AssembleLocalElementMatrixBarycentric2D(int elem_n, int *nodes, int **displaceinE, double *x, double *y, int dimension, double **E, double **M, double nu, double youngE, double thickness)
{
	// thte dimension for B is 3x6
	int n = 3;
	//declear how many rows that we will have, 
	double **B = new double*[n];
	double **D = new double*[n];
	double **B_TXD = new double*[n * 2];
	double **integrand = new double*[n * 2];
	double **DB = new double*[n];
	double *stress = new double[n];


	//Now we will loop through the columns
	for (int i = 0; i < n; i++){

		B[i] = new double[n * 2];
		D[i] = new double[n];
		DB[i] = new double[n * 2];
		stress[i] = 0;//initializing the stress vector
	}
	for (int i = 0; i < n * 2; i++){

		B_TXD[i] = new double[n];
		integrand[i] = new double[n * 2];
	}


	double J = Linear2DJacobianDet_Barycentric(nodes, x, y);


	for (int row = 0; row < n * 2; row++){
		for (int col = 0; col < n; col++){
			B_TXD[row][col] = 0;
		}
	}

	for (int row = 0; row < n; row++){
		for (int col = 0; col < n * 2; col++){
			DB[row][col] = 0;
		}
	}

	for (int row = 0; row < n * 2; row++){
		for (int col = 0; col < n * 2; col++){
			integrand[row][col] = 0;
		}
	}

	//Allocating the B and D matrices
	Linear2DBarycentric_B(nodes, x, y, B);

	Linear2DBarycentric_D(D, nu, youngE);

	//std::cout << "B:MATRIX: " << std::endl;
	//for (int row = 0; row < 3; row++){
	//	for (int col = 0; col < 6; col++){
	//		std::cout << B[row][col]<< "    ";
	//	}
	//	std::cout << std::endl;
	//}

	//Finding B^T*D
	for (int row = 0; row < n * 2; row++){
		for (int col = 0; col < n; col++){
			for (int k = 0; k < n; k++){
				B_TXD[row][col] = B_TXD[row][col] + B[k][row] * D[k][col];
			}
		}
	}
	//Finding B^T*D*B
	for (int row = 0; row < n * 2; row++){
		for (int col = 0; col < n * 2; col++){
			for (int k = 0; k < n; k++){
				integrand[row][col] = integrand[row][col] + B_TXD[row][k] * B[k][col];
			}
		}
	}

	if (get_dynamic()){
		//Find von-mises stresses
		//First is D*B [DONT FORGET TO MULTIPLY BY (youngE / ((1.0 - nu*nu)))*thickness]

		for (int row = 0; row < n; row++){
			for (int col = 0; col < n * 2; col++){
				for (int k = 0; k < n; k++){
					DB[row][col] = DB[row][col] + D[row][k] * B[k][col];
				}
			}
		}


		for (int row = 0; row < n; row++){
			for (int col = 0; col < n; col++){
				for (int k = 0; k < 2; k++){
					stress[row] = stress[row] + DB[row][col * 2 + k] * u[displaceinE[nodes[col]][k]];
				}



			}
#if 1 // for plain stress
			stress[row] = stress[row] * (youngE / ((1.0 - nu*nu)))*thickness*(J / 2.0);
#endif // 0 // for plain stress
#if 0 // for plain strain
			stress[row] = stress[row] * (youngE / ((1.0 + nu)*(1 - 2 * nu)))*thickness*(J / 2.0);
#endif // 0 // for plain stress
		}

		global_stress_mises[elem_n] = sqrt((stress[0] + stress[1])*(stress[0] + stress[1]) - 3 * (stress[0] * stress[1] - stress[2] * stress[2]));
#if 0  ///Print strain out
		std::cout << "DB : " << std::endl;
		for (int row = 0; row < 3; row++){
			for (int col = 0; col < 6; col++){
				std::cout << DB[row][col] << "    ";
			}
			std::cout<<std::endl;
		}
#endif

#if 0
		std::cout << " stress : " << std::endl;

		for (int row = 0; row < 3; row++){
			std::cout << stress[row] << std::endl;
		}

#endif
#if 0

		std::cout << global_stress_mises[elem_n] << std::endl;
#endif

#if 0
		for (int row = 0; row < 3; row++){
			for (int k = 0; k < 2; k++){
				std::cout << DB[row][nodes[row]] << std::endl;
			}
		}
#endif



		//for (int row = 0; row < 6; row++){
		//	for (int col = 0; col < 3; col++){
		//		B_T[row][col] = B[col][row];
		//	}
		//}

	}
	for (int row = 0; row < n * 2; row++){
		for (int col = 0; col < n * 2; col++){
			//row = row ^ 2;
			E[row][col] = (youngE / ((1.0 - nu*nu)))*thickness*(J / 2.0) * integrand[row][col];

		}

	}


	//std::cout << "K_elem : " << std::endl;
	//for (int row = 0; row < 6; row++){
	//	for (int col = 0; col < 6; col++){
	//		std::cout << E[row][col] << "    ";
	//	}
	//	std::cout << std::endl;
	//}

	//
	double  A = J / 2;
	double X_i = x[0];
	double X_j = x[1];
	double X_k = x[2];

	double Y_i = y[0];
	double Y_j = y[1];
	double Y_k = y[2];

	double Z_i = z[0];
	double Z_j = z[1];
	double Z_k = z[2];

	double a_i = X_j*Y_k - X_k*Y_j;
	double a_j = X_k*Y_i - X_i*Y_k;
	double a_k = X_i*Y_j - X_j*Y_i;
	double b_i = Y_j - Y_k;
	double b_j = Y_k - Y_i;
	double b_k = Y_i - Y_j;
	double c_i = X_k - X_j;
	double c_j = X_i - X_k;
	double c_k = X_j - X_i;
	double rho = 1000.0;
	if (get_dynamic()){
		M[0][0] = 2 * A*rho*thickness / 3.0;
		M[0][1] = 0.0;
		M[0][2] = A*rho*thickness / 2.0;
		M[0][3] = 0.0;
		M[0][4] = -A*rho*thickness / 6.0;
		M[0][5] = 0.0;
		M[1][0] = 0.0;
		M[1][1] = 2 * A*rho*thickness / 3.0;
		M[1][2] = 0.0;
		M[1][3] = A*rho*thickness / 2.0;
		M[1][4] = 0.0;
		M[1][5] = -A*rho*thickness / 6.0;
		M[2][0] = A*rho*thickness / 2.0;
		M[2][1] = 0.0;
		M[2][2] = 2 * A*rho*thickness / 3.0;
		M[2][3] = 0.0;
		M[2][4] = -A*rho*thickness / 6.0;
		M[2][5] = 0.0;
		M[3][0] = 0.0;
		M[3][1] = A*rho*thickness / 2.0;
		M[3][2] = 0.0;
		M[3][3] = 2.0* A*rho*thickness / 3.0;
		M[3][4] = 0.0;
		M[3][5] = -A*rho*thickness / 6.0;
		M[4][0] = -A*rho*thickness / 6.0;
		M[4][1] = 0.0;
		M[4][2] = -A*rho*thickness / 6.0;
		M[4][3] = 0.0;
		M[4][4] = A*rho*thickness / 3.0;
		M[4][5] = 0.0;
		M[5][0] = 0.0;
		M[5][1] = -A*rho*thickness / 6.0;
		M[5][2] = 0.0;
		M[5][3] = -A*rho*thickness / 6.0;
		M[5][4] = 0.0;
		M[5][5] = A*rho*thickness / 3.0;
	}

	for (int i = 0; i < n; i++){

		delete B[i];
		delete D[i];
		delete DB[i];
	}
	for (int i = 0; i < n * 2; i++){

		delete B_TXD[i];
		delete integrand[i];
	}

	delete[] B;
	delete[] D;
	delete[] B_TXD;
	delete[] integrand;
	delete[] DB;
	delete[] stress;

}

//**************************3D************************************//
//3333333333333333333333333333333333333333333333333333333333333333//
//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD//
//****************************************************************//
void Geometry::AssembleLocalElementMatrixBarycentric3D(int *nodes, double *x, double *y, double *z, int dimension, double **E, double nu, double youngE, double thickness)
{
	int multi = 2;
	double **B = new double*[3 * multi];
	double **D = new double*[3 * multi];
	double **B_TXD = new double*[6 * multi];
	double **integrand = new double*[6 * multi];


	for (int i = 0; i < 3 * multi; i++){

		B[i] = new double[6 * multi];
		D[i] = new double[3 * multi];
	}
	for (int i = 0; i < 6 * multi; i++){

		B_TXD[i] = new double[3 * multi];
		integrand[i] = new double[6 * multi];
	}


	double J = Linear3DJacobianDet_Barycentric(nodes, x, y, z);


	for (int row = 0; row < 6 * multi; row++){
		for (int col = 0; col < 3 * multi; col++){
			B_TXD[row][col] = 0;
		}
	}

	for (int row = 0; row < 6 * multi; row++){
		for (int col = 0; col < 6 * multi; col++){
			integrand[row][col] = 0;
		}
	}

	//Allocating the B and D matrices
	Linear3DBarycentric_B(nodes, x, y, z, B);
	Linear3DBarycentric_D(D, nu, youngE);

	//std::cout << "B:MATRIX: " << std::endl;
	//for (int row = 0; row < 3*multi; row++){
	//	for (int col = 0; col < 6*multi; col++){
	//		std::cout << B[row][col]<< "    ";
	//	}
	//	std::cout << std::endl;
	//}

	//Finding B^T*D
	for (int row = 0; row < 6 * multi; row++){
		for (int col = 0; col < 3 * multi; col++){
			for (int k = 0; k < 3 * multi; k++){
				B_TXD[row][col] = B_TXD[row][col] + B[k][row] * D[k][col];
			}
		}
	}
	//Finding B^T*D*B
	for (int row = 0; row < 6 * multi; row++){
		for (int col = 0; col < 6 * multi; col++){
			for (int k = 0; k < 3 * multi; k++){
				integrand[row][col] = integrand[row][col] + B_TXD[row][k] * B[k][col];
			}
		}
	}

	//std::cout << "B_T x D : " << std::endl;
	//for (int row = 0; row < 6; row++){
	//	for (int col = 0; col < 3; col++){
	//		std::cout << B_TXD[row][col] << "    ";
	//	}
	//	std::cout<<std::endl;
	//}



	//for (int row = 0; row < 6; row++){
	//	for (int col = 0; col < 3; col++){
	//		B_T[row][col] = B[col][row];
	//	}
	//}





	for (int row = 0; row < 6 * multi; row++){
		for (int col = 0; col < 6 * multi; col++){

			E[row][col] = integrand[row][col] * J / 6.0;

		}

	}


	//std::cout << "K_elem : " << std::endl;
	//for (int row = 0; row < 6*multi; row++){
	//	for (int col = 0; col < 6*multi; col++){
	//		std::cout << E[row][col] << " ";
	//	}
	//	std::cout << std::endl;
	//}





	for (int i = 0; i < 3 * multi; i++){

		delete B[i];
		delete D[i];
	}
	for (int i = 0; i < 6 * multi; i++){

		delete B_TXD[i];
		delete integrand[i];
	}

	delete[] B;
	delete[] D;
	delete[] B_TXD;
	delete[] integrand;

}
//3D
void Geometry::Linear3DBarycentric_B_CUDA_host(){

	int dummy_var;
	//dim3 blocks(1, 1, numE/5);//numE / (dim)
	//dim3 threads(numNodesPerElem*dim, numNodesPerElem*dim, 5);
	//dim3 blocks(144, (int)numE /( 32*15));//numE / (dim)
	//dim3 threads(1, (int)(32 * 15));

	//working 2d cuda
	dim3 blocks(84, 196);//numE / (dim)
	dim3 threads(12, 12);
	/*for (int j = 0; j < numE;j++){
		for (int i = 0; i < numNodesPerElem; i++){
		nodesInElem_device[j][i] = nodesInElem[j][i];
		}
		}
		*/

	cudaMemcpy(d_x_dist, x, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	cudaMemcpy(d_y_dist, y, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	cudaMemcpy(d_z_dist, z, numNodes*sizeof(*d_x_dist), cudaMemcpyHostToDevice);
	//cudaMemcpy(nodesInElem_device, nodesInElem, numE*numNodesPerElem*sizeof(int), cudaMemcpyHostToDevice);

	int max_limit = (numNodesPerElem*dim*numNodesPerElem*dim*numE);
	int threadsPerBlock = 256;
	int blocksPerGrid = (max_limit + threadsPerBlock - 1) / threadsPerBlock;
	/*cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, 0);*/
	cudaMemset(d_A_dense, 0, numNodes*dim*numNodes*dim*sizeof(*d_A_dense));
	cudaMemcpy(dev_numNodes, &numNodes, 1 * sizeof(int), cudaMemcpyHostToDevice);

	make_K_cuda3d << < 192, 128 >> >(E_vector_device, nodesInElem_device, d_x_dist, d_y_dist, d_z_dist, displaceInElem_device, d_A_dense, dev_numNodes);

	std::clock_t cuda_K;
	cuda_K = std::clock();
	//cudaMemcpy(h_A_dense, d_A_dense, numNodes*dim*numNodes*dim*sizeof(*d_A_dense), cudaMemcpyDeviceToHost);
	double duration_K = (std::clock() - cuda_K) / (double)CLOCKS_PER_SEC;
	std::cout << "cuda k assmeble: " << duration_K << std::endl;
	//cudaMemcpy(E_vector_host, E_vector_device, numNodesPerElem*dim*numNodesPerElem*dim*numE*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(x, d_x, numNodes*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(nodesInElem_host, nodesInElem_device, numE*numNodesPerElem*sizeof(int), cudaMemcpyDeviceToHost);

	//std::cout << " K _ CUDA " << std::endl;
	////for (int j = 0; j < 2; j++){
	////	for (int i = 0; i < numNodesPerElem; i++){
	////		std::cout << nodesInElem_host[nodesinelemX(i, j, numNodesPerElem)] << "  ";
	////	}
	////	std::cout << std::endl;
	////}
	//for (int j = 0; j < 10; j++){
	//	for (int i = 0; i < 10; i++){
	//		std::cout << h_A_dense[IDX2C(i, j, 3000)] << "  ";
	//	}
	//	std::cout << std::endl;
	//}



	////Print local K matrix
	//for (int e = 0; e < numE; e++){

	//	//std::cout << "element : " << e << std::endl;
	//	for (int i = 0; i < numNodesPerElem*dim; i++){
	//		for (int j = 0; j < numNodesPerElem*dim; j++){
	//			
	//			//E[e][i][j] = E_vector_host[threeD21D(i, j, e, numNodesPerElem*dim, numNodesPerElem*dim)];
	//			 //std::cout << E[e][i][j] << " ";
	//		}
	//		//std::cout << std::endl;
	//	}
	//	//std::cout << std::endl;
	//}

	//std::cout << std::endl << " the x value : " << x[0] << std::endl;
	/*(cudaMemcpy(&c, dev_c, sizeof(int),
		cudaMemcpyDeviceToHost));
		printf("2 + 7 = %d\n", c);
		(cudaFree(dev_c));*/

}

//2D
void Geometry::Linear2DBarycentric_B_CUDA_host(){




	cudaMemcpy(d_x, x, numNodes*sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(d_y, y, numNodes*sizeof(double), cudaMemcpyHostToDevice);

	dim3 blocks((numE + 35) / 35, 1);//numE / (dim)
	dim3 threads(36, 36);


	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, 0);
	cudaMemset(d_A_dense, 0.0, numNodes*dim*numNodes*dim*sizeof(*d_A_dense));

	make_K_cuda2d << <147, 112 >> >(E_vector_device, nodesInElem_device, d_x, d_y, displaceInElem_device, d_A_dense, numNodes, thickness, Young, Poisson, c_alpha, beta_1, beta_2, density, dt, c_xi, numE);
	//make_K_cuda2d << <blocks, threads >> >(E_vector_device, nodesInElem_device, d_x, d_y, displaceInElem_device, d_A_dense, numNodes, thickness, Young, Poisson, c_alpha, beta_1, beta_2, density, dt, c_xi, numE);

	cudaMemcpy(h_A_dense, d_A_dense, numNodes*dim*numNodes*dim*sizeof(*d_A_dense), cudaMemcpyDeviceToHost);

	//cudaMemcpy(E_vector_host, E_vector_device, numNodesPerElem*dim*numNodesPerElem*dim*numE*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(x, d_x, numNodes*sizeof(double), cudaMemcpyDeviceToHost);
	//cudaMemcpy(nodesInElem_host, nodesInElem_device, numE*numNodesPerElem*sizeof(int), cudaMemcpyDeviceToHost);

	//std::cout << " K _ CUDA " << std::endl;
	////for (int j = 0; j < 2; j++){
	////	for (int i = 0; i < numNodesPerElem; i++){
	////		std::cout << nodesInElem_host[nodesinelemX(i, j, numNodesPerElem)] << "  ";
	////	}
	////	std::cout << std::endl;
	////}
	//for (int j = 0; j < 10; j++){
	//	for (int i = 0; i < 10; i++){
	//		std::cout << h_A_dense[IDX2C(i, j, numNodes*2)] << "  ";
	//	}
	//	std::cout << std::endl;
	//}




	////Print local K matrix
	//for (int e = 0; e < numE; e++){

	//	//std::cout << "element : " << e << std::endl;
	//	for (int i = 0; i < numNodesPerElem*dim; i++){
	//		for (int j = 0; j < numNodesPerElem*dim; j++){
	//			
	//			//E[e][i][j] = E_vector_host[threeD21D(i, j, e, numNodesPerElem*dim, numNodesPerElem*dim)];
	//			 //std::cout << E[e][i][j] << " ";
	//		}
	//		//std::cout << std::endl;
	//	}
	//	//std::cout << std::endl;
	//}

	//std::cout << std::endl << " the x value : " << x[0] << std::endl;
	/*(cudaMemcpy(&c, dev_c, sizeof(int),
	cudaMemcpyDeviceToHost));
	printf("2 + 7 = %d\n", c);
	(cudaFree(dev_c));*/

}

void Geometry::make_surface_f(){


}
void Geometry::ApplyEssentialBoundaryConditionsBarycentric(int numP, int numBC, int *localcoord, int *elemForce, double forceVec_x, double forceVec_y, double *f, double **K, int **nodesInElem, double thickness, double *x, double *y, int **displaceInElem){
	int local; // used to store local coord info
	int node_interest[2];// use two ints to tell us which 2 of the nodes in the element would be useful
	int row, col;
	int element;
	int node;
	double length;
	double x_1, y_1, x_2, y_2;
	//for (int i = 0; i < numBC; i++){
	elemForce[0] = numBC;

	//local = localcoord[i];
	int i = 0;
	local = 1;
	if (local == 0){//Opposite to xi_1 direction
		node_interest[0] = 1;
		node_interest[1] = 2;
		x_1 = x[nodesInElem[elemForce[i]][node_interest[0]]];
		y_1 = y[nodesInElem[elemForce[i]][node_interest[0]]];
		x_2 = x[nodesInElem[elemForce[i]][node_interest[1]]];
		y_2 = y[nodesInElem[elemForce[i]][node_interest[1]]];
		length = sqrt(pow(x_1 - x_2, 2.0) + pow(y_1 - y_2, 2.0));
	}
	else if (local == 1){//Opposite to xi_2 direction
		node_interest[0] = 0;
		node_interest[1] = 2;
		x_1 = x[nodesInElem[elemForce[i]][node_interest[0]]];
		y_1 = y[nodesInElem[elemForce[i]][node_interest[0]]];
		x_2 = x[nodesInElem[elemForce[i]][node_interest[1]]];
		y_2 = y[nodesInElem[elemForce[i]][node_interest[1]]];
		length = sqrt(pow(x_1 - x_2, 2.0) + pow(y_1 - y_2, 2.0));
	}
	else if (local == 2){ // Opposite to xi_3 direction
		node_interest[0] = 0;
		node_interest[1] = 1;
		x_1 = x[nodesInElem[elemForce[i]][node_interest[0]]];
		y_1 = y[nodesInElem[elemForce[i]][node_interest[0]]];
		x_2 = x[nodesInElem[elemForce[i]][node_interest[1]]];
		y_2 = y[nodesInElem[elemForce[i]][node_interest[1]]];
		length = sqrt(pow(x_1 - x_2, 2.0) + pow(y_1 - y_2, 2.0));
	}
	//cout << endl << "length: " << length << endl;
	element = elemForce[i];
	for (int node_c = 0; node_c < 2; node_c++){
		node = nodesInElem[element][node_interest[node_c]];
		for (int dof = 0; dof < 2; dof++){
			row = displaceInElem[node][dof];

			for (int dummy_V = 0; dummy_V < numP; dummy_V++){
				//K[row][dummy_V] = 0;
			}
			//K[row][row] = 1;
			if (dof == 0){
				//f[row] = f[row] + (length*thickness / 2)*forceVec_x[i];
				f[row] = f[row] + (length*thickness / 2)*forceVec_x;
			}
			else if (dof == 1){
				//f[row] = f[row] + (length*thickness / 2)*forceVec_y[i];
				f[row] = f[row] + (length*thickness / 2)*forceVec_y;
			}
		}
	}



	//}
}


void Geometry::ApplySudoForcesBarycentric(int numP, int node_applied, int *localcoord, int *elemForce, double forceVec_x, double forceVec_y, double *g, int **nodesInElem, double thickness, double *x, double *y, int **displaceInElem){
	int local; // used to store local coord info
	int node_interest[2];// use two ints to tell us which 2 of the nodes in the element would be useful
	int row, col;
	int element;
	int node;
	double length;
	double x_1, y_1, x_2, y_2;


	for (int dummy_V = 0; dummy_V < numP; dummy_V++){
		f[dummy_V] = 0;
	}
#if 1
	//cout << endl << "length: " << length << endl;

	//int node_c = node_applied;
	double f1, f2;
	for (int findex = 0; findex < 2; findex++){

		int node_c = sudo_force_index[findex];

		//******************************THIS NEEDS CHANGING **************************************//

		if (findex == 0){
			f1 = sudo_force_value1[0];
			f2 = sudo_force_value1[1];

		}
		else if (findex == 1){
			f1 = sudo_force_value2[0];
			f2 = sudo_force_value2[1];

		}
		else if (findex == 2){
			f1 = sudo_force_value3[0];
			f2 = sudo_force_value3[1];
		}
		else if (findex == 3){
			f1 = sudo_force_value4[0];
			f2 = sudo_force_value4[1];
		}
		/*double forceVec_x1 = sudo_force_value2[0];
		double forceVec_y1 = sudo_force_value2[1];*/
		for (int dof = 0; dof < dim; dof++){
			row = displaceInElem[node_c][dof];

			for (int dummy_V = 0; dummy_V < numP; dummy_V++){
				//K[row][dummy_V] = 0;
			}
			//K[row][row] = 1;
			if (dof == 0){
				f[row] += f1;
			}
			else if (dof == 1){
				f[row] += f2;
			}
			else if (dof == 2){
				// f[row] += forceVec_y1;
			}
		}
	}
#endif // 0




}

void Geometry::set_zero_nodes(int *points){
	//We have to make the corresponding L matrix and rhs vectors set to the correct values.


	for (int i = 0; i < numNodesZero; i++){
		vector_zero_nodes[i] = points[i];
	}


}

void Geometry::set_zero_AxB(void){
	int row1, row2;
#if 1
	for (int i = 0; i < numNodesZero; i++){
		row1 = displaceInElem[vector_zero_nodes[i]][0];
		row2 = displaceInElem[vector_zero_nodes[i]][1];
		for (int col = 0; col < Ncols; col++){

			L[IDX2C(col, row1, N)] = 0.0;
			L[IDX2C(col, row2, N)] = 0.0;

		}
		L[IDX2C(row1, row1, N)] = 1.0;
		L[IDX2C(row2, row2, N)] = 1.0;
		b_rhs[row1] = 0.0;
		b_rhs[row2] = 0.0;
	}

#else
	for (int i = 0; i < numNodesZero; i++){
		row1 = displaceInElem[vector_zero_nodes[i]][0];
		row2 = displaceInElem[vector_zero_nodes[i]][1];
		for (int col = 0; col < Ncols; col++){

			h_A_dense[IDX2C(col, row1, N)] = 0.0;
			h_A_dense[IDX2C(col, row2, N)] = 0.0;

		}
		h_A_dense[IDX2C(row1, row1, N)] = 1.0;
		h_A_dense[IDX2C(row2, row2, N)] = 1.0;
		/*f[row1] = 0.0;
		f[row2] = 0.0;*/
		b_rhs[row1] = 0.0;
		b_rhs[row2] = 0.0;
	}
#endif // 0

#if 0
	row1 = displaceInElem[sudo_force_index[0]][0];
	row2 = displaceInElem[sudo_force_index[0]][1];
	for (int col = 0; col < Ncols; col++){

		L[IDX2C(col, row1, N)] = 0.0;
		L[IDX2C(col, row2, N)] = 0.0;

	}
	L[IDX2C(row1, row1, N)] = 1.0;
	L[IDX2C(row2, row2, N)] = 1.0;
	b_rhs[row1] = (1.0 / beta_2)*((sudo_force_value1[0] - dt*u_dot[row1]) - (1.0 - beta_2)*u_doubledot_old[row1]);
	b_rhs[row2] = (1.0 / beta_2)*((sudo_force_value1[1] - dt*u_dot[row2]) - (1.0 - beta_2)*u_doubledot_old[row2]);;

#endif // 0


}

void Geometry::initialize_CUDA(void){
	Nrows = numNodes*dim;                        // --- Number of rows
	Ncols = numNodes*dim;                        // --- Number of columns
	N = Nrows;
	cusparseSafeCall(cusparseCreate(&handle));

	//h_A_dense = (float*)malloc(Nrows*Ncols*sizeof(*h_A_dense));
	cusparseSafeCall(cusparseCreateMatDescr(&descrA));
	cusparseSafeCall(cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL));
	cusparseSafeCall(cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ONE));
	nnz = 0;                                // --- Number of nonzero elements in dense matrix
	lda = Nrows;                      // --- Leading dimension of dense matrix
	gpuErrchk(cudaMalloc(&d_nnzPerVector, Nrows * sizeof(*d_nnzPerVector)));
	h_nnzPerVector = (int *)malloc(Nrows * sizeof(*h_nnzPerVector));


	//device side dense matrix
	gpuErrchk(cudaMalloc(&d_A_RowIndices, (Nrows + 1) * sizeof(*d_A_RowIndices)));

	cudaMalloc((void **)&dev_numNodes, sizeof(dev_numNodes));
	cudaMemcpy(dev_numNodes, &numNodes, sizeof(dev_numNodes), cudaMemcpyHostToDevice);

	//cudaMemcpy(&numNodes,dev_numNodes , sizeof(dev_numNodes), cudaMemcpyDeviceToHost);


	cusparseSafeCall(cusparseCreateMatDescr(&descr_L));
	cusparseSafeCall(cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ONE));
	cusparseSafeCall(cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL));
	cusparseSafeCall(cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER));
	cusparseSafeCall(cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_NON_UNIT));

	//emeory in cholesky
	cusparseSafeCall(cusparseCreateCsric02Info(&info_A));
	cusparseSafeCall(cusparseCreateCsrsv2Info(&info_L));
	cusparseSafeCall(cusparseCreateCsrsv2Info(&info_Lt));

}


int Geometry::tt()
{
	// --- Initialize cuSPARSE


	// --- Host side dense matrix

	double duration_K;


	// --- Column-major ordering
	/*h_A_dense[0] = 0.4612f;  h_A_dense[4] = -0.0006f;   h_A_dense[8] = 0.3566f; h_A_dense[12] = 0.0f;
	h_A_dense[1] = -0.0006f; h_A_dense[5] = 0.4640f;    h_A_dense[9] = -1000.0723f; h_A_dense[13] = 0.0f;
	h_A_dense[2] = 0.3566f;  h_A_dense[6] = 0.0723f;    h_A_dense[10] = 100.7543f; h_A_dense[14] = 0.0f;
	h_A_dense[3] = 0.f;      h_A_dense[7] = 0.0f;       h_A_dense[11] = 0.0f;    h_A_dense[15] = 0.1f;
	*/

	//for (int col = 0; col < Ncols; col++){
	//	for (int row = 0; row < Nrows; row++){

	//		h_A_dense[IDX2C(col, row, N)] = (float) h_A_dense_double[IDX2C(col, row, N)];
	//		//a[IDX2C(col, row, n)] = (float)ind++;
	//		//h_A_dense[IDX2C(col, row, N)] = 0;


	//	}

	//}
	if (!cuda_use){
		for (int col = 0; col < Ncols; col++){

			h_A_dense[IDX2C(col, 0, N)] = 0;
			h_A_dense[IDX2C(col, 1, N)] = 0;
			if (dim == 3){
				h_A_dense[IDX2C(col, 2, N)] = 0;
			}
		}
		h_A_dense[IDX2C(0, 0, N)] = 1.0;
		h_A_dense[IDX2C(1, 1, N)] = 1.0;
		if (dim == 3){
			h_A_dense[IDX2C(2, 2, N)] = 1.0;
		}
		set_zero_AxB();
		gpuErrchk(cudaMemcpy(d_A_dense, h_A_dense, Nrows * Ncols * sizeof(*d_A_dense), cudaMemcpyHostToDevice));
	}
#if 0
	std::ofstream writenodes("global_K.txt");

	for (int j = 0; j < N; j++){
		for (int i = 0; i < N; i++){
			writenodes << h_A_dense[IDX2C(j, i, N)] << " ";
		}
		writenodes << std::endl;
	}

	writenodes.close();
#endif // 0


	// --- Create device array and copy host array to it
	/*for (int j = 0; j < 20; j++){
		for (int i = 0; i < 20; i++){
		std::cout << h_A_dense[IDX2C(j, i, N)] << std::endl;
		}
		std::cout<<std::endl;
		}*/



	// --- Descriptor for sparse matrix A




	// --- Device side number of nonzero elements per row

	cusparseSafeCall(cusparseSnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, &nnz));
	// --- Host side number of nonzero elements per row

	gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));

	/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
	for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
	printf("\n");*/

	// --- Device side dense matrix
	gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
	gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));


	cusparseSafeCall(cusparseSdense2csr(handle, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));
	std::clock_t start_K;
	start_K = std::clock();
	// --- Host side dense matrix
	float *h_A = (float *)malloc(nnz * sizeof(*h_A));
	int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
	int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
	gpuErrchk(cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost));
	std::cout << nnz << std::endl;
	/*printf("\nOriginal matrix in CSR format\n\n");
	for (int i = 0; i < 10; ++i) printf("A[%i] = %.0f ", i, h_A[i]); printf("\n");

	printf("\n");
	for (int i = 0; i < (10 + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");

	for (int i = 0; i < 10; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);
	*/
	// --- Allocating and defining dense host and device data vectors

	float *h_x = (float *)malloc(Nrows * sizeof(float));
	/*h_x[0] = 100.0;  h_x[1] = 200.0; h_x[2] = 400.0; h_x[3] = 500.0;*/
	for (int i = 0; i < N; i++){
		h_x[i] = f[i];
	}
	if (dim == 3){
		h_x[0] = h_x[1] = h_x[2] = 0;
	}
	else {
		h_x[0] = h_x[1] = 0;
	}



	float *d_x;        gpuErrchk(cudaMalloc(&d_x, Nrows * sizeof(float)));
	gpuErrchk(cudaMemcpy(d_x, h_x, Nrows * sizeof(float), cudaMemcpyHostToDevice));




	/******************************************/
	/* STEP 1: CREATE DESCRIPTORS FOR L AND U */
	/******************************************/




	/********************************************************************************************************/
	/* STEP 2: QUERY HOW MUCH MEMORY USED IN CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
	/********************************************************************************************************/


	int pBufferSize_M, pBufferSize_L, pBufferSize_Lt;
	cusparseSafeCall(cusparseScsric02_bufferSize(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, &pBufferSize_M));
	cusparseSafeCall(cusparseScsrsv2_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, &pBufferSize_L));
	cusparseSafeCall(cusparseScsrsv2_bufferSize(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, &pBufferSize_Lt));

	int pBufferSize = max(pBufferSize_M, max(pBufferSize_L, pBufferSize_Lt));
	void *pBuffer = 0;  gpuErrchk(cudaMalloc((void**)&pBuffer, pBufferSize));


	/******************************************************************************************************/
	/* STEP 3: ANALYZE THE THREE PROBLEMS: CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
	/******************************************************************************************************/
	int structural_zero;

	cusparseSafeCall(cusparseScsric02_analysis(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));

	cusparseStatus_t status = cusparseXcsric02_zeroPivot(handle, info_A, &structural_zero);
	if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("A(%d,%d) is missing\n", structural_zero, structural_zero); }

	cusparseSafeCall(cusparseScsrsv2_analysis(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
	cusparseSafeCall(cusparseScsrsv2_analysis(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));

	/*************************************/
	/* STEP 4: FACTORIZATION: A = L * L' */
	/*************************************/
	int numerical_zero;

	cusparseSafeCall(cusparseScsric02(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
	status = cusparseXcsric02_zeroPivot(handle, info_A, &numerical_zero);
	/*if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("L(%d,%d) is zero\n", numerical_zero, numerical_zero); }
	*/

	gpuErrchk(cudaMemcpy(h_A, d_A, nnz * sizeof(float), cudaMemcpyDeviceToHost));
	/*printf("\nNon-zero elements in Cholesky matrix\n\n");
	for (int k = 0; k<10; k++) printf("%f\n", h_A[k]);*/

	cusparseSafeCall(cusparseScsr2dense(handle, Nrows, Ncols, descrA, d_A, d_A_RowIndices, d_A_ColIndices, d_A_dense, Nrows));

	/*printf("\nCholesky matrix\n\n");
	for (int i = 0; i < 10; i++) {
	std::cout << "[ ";
	for (int j = 0; j < 10; j++)
	std::cout << h_A_dense[i * Ncols + j] << " ";
	std::cout << "]\n";
	}*/

	/*********************/
	/* STEP 5: L * z = x */
	/*********************/
	// --- Allocating the intermediate result vector
	float *d_z;        gpuErrchk(cudaMalloc(&d_z, N * sizeof(float)));

	const float alpha = 1.;
	cusparseSafeCall(cusparseScsrsv2_solve(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, d_x, d_z, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));

	/**********************/
	/* STEP 5: L' * y = z */
	/**********************/
	// --- Allocating the host and device side result vector
	float *h_y = (float *)malloc(Ncols * sizeof(float));
	float *d_y;        gpuErrchk(cudaMalloc(&d_y, Ncols * sizeof(float)));

	cusparseSafeCall(cusparseScsrsv2_solve(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, d_z, d_y, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));

	cudaMemcpy(h_x, d_y, N * sizeof(float), cudaMemcpyDeviceToHost);
	printf("\n\nFinal result\n");
	/*for (int k = 0; k<20; k++) printf("dx[%i] = %f\n", k, h_x[k]);
	for (int k = 0; k<20; k++) printf("xs[%i] = %f\n", k, x[k]);*/


	for (int i = 0; i < numNodes; i++) {
		x[i] = x[i] + h_x[i * dim];
		y[i] = y[i] + h_x[i * dim + 1];
		if (dim == 3){
			z[i] = z[i] + h_x[i * dim + 2];
		}

	}

	free(h_A);
	free(h_A_RowIndices);
	free(h_A_ColIndices);
	//free(h_x);
	free(h_y);
	cudaFree(d_x);
	cudaFree(pBuffer);
	cudaFree(d_z);
	cudaFree(d_y);
	duration_K = (std::clock() - start_K) / (double)CLOCKS_PER_SEC;
	//std::cout << " change status : " << changeNode << std::endl;

	//std::cout << "FPS time: " <<1/duration_K << std::endl;

	//std::cout << "Duration: " << duration_K << std::endl;
	return 0;
}


//int Geometry::tt()
//{
//	// --- Initialize cuSPARSE
//	cusparseHandle_t handle;    cusparseSafeCall(cusparseCreate(&handle));
//
//	const int Nrows = numNodes*dim;                        // --- Number of rows
//	const int Ncols = numNodes*dim;                        // --- Number of columns
//	const int N = Nrows;
//
//	// --- Host side dense matrix
//	double *h_A_dense = (double*)malloc(Nrows*Ncols*sizeof(*h_A_dense));
//
//	// --- Column-major ordering
//	/*h_A_dense[0] = 0.4612f;  h_A_dense[4] = -0.0006f;   h_A_dense[8] = 0.3566f; h_A_dense[12] = 0.0f;
//	h_A_dense[1] = -0.0006f; h_A_dense[5] = 0.4640f;    h_A_dense[9] = -1000.0723f; h_A_dense[13] = 0.0f;
//	h_A_dense[2] = 0.3566f;  h_A_dense[6] = 0.0723f;    h_A_dense[10] = 100.7543f; h_A_dense[14] = 0.0f;
//	h_A_dense[3] = 0.f;      h_A_dense[7] = 0.0f;       h_A_dense[11] = 0.0f;    h_A_dense[15] = 0.1f;
//	*/
//	for (int col = 0; col < Ncols; col++){
//		for (int row = 0; row < Nrows; row++){
//
//			h_A_dense[IDX2C(col, row, N)] = K[col][row];
//			//a[IDX2C(col, row, n)] = (double)ind++;
//			//h_A_dense[IDX2C(col, row, N)] = 0;
//
//
//		}
//
//	}
//	for (int col = 0; col < Ncols; col++){
//
//		h_A_dense[IDX2C(col, 0, N)] = 0;
//		h_A_dense[IDX2C(col, 1, N)] = 0;
//	}
//	h_A_dense[IDX2C(0, 0, N)] = 1;
//	h_A_dense[IDX2C(1, 1, N)] = 1;
//
//
//	// --- Create device array and copy host array to it
//	double *d_A_dense;  gpuErrchk(cudaMalloc(&d_A_dense, Nrows * Ncols * sizeof(*d_A_dense)));
//	gpuErrchk(cudaMemcpy(d_A_dense, h_A_dense, Nrows * Ncols * sizeof(*d_A_dense), cudaMemcpyHostToDevice));
//
//	// --- Descriptor for sparse matrix A
//	cusparseMatDescr_t descrA;      cusparseSafeCall(cusparseCreateMatDescr(&descrA));
//	cusparseSafeCall(cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL));
//	cusparseSafeCall(cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ONE));
//
//	int nnz = 0;                                // --- Number of nonzero elements in dense matrix
//	const int lda = Nrows;                      // --- Leading dimension of dense matrix
//	// --- Device side number of nonzero elements per row
//	int *d_nnzPerVector;    gpuErrchk(cudaMalloc(&d_nnzPerVector, Nrows * sizeof(*d_nnzPerVector)));
//	cusparseSafeCall(cusparseDnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, &nnz));
//	// --- Host side number of nonzero elements per row
//	int *h_nnzPerVector = (int *)malloc(Nrows * sizeof(*h_nnzPerVector));
//	gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));
//
//	/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
//	for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
//	printf("\n");*/
//
//	// --- Device side dense matrix
//	double *d_A;            gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
//	int *d_A_RowIndices;    gpuErrchk(cudaMalloc(&d_A_RowIndices, (Nrows + 1) * sizeof(*d_A_RowIndices)));
//	int *d_A_ColIndices;    gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));
//
//	cusparseSafeCall(cusparseDdense2csr(handle, Nrows, Ncols, descrA, d_A_dense, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));
//
//	// --- Host side dense matrix
//	double *h_A = (double *)malloc(nnz * sizeof(*h_A));
//	int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
//	int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
//	gpuErrchk(cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost));
//	gpuErrchk(cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost));
//	gpuErrchk(cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost));
//
//	/*printf("\nOriginal matrix in CSR format\n\n");
//	for (int i = 0; i < 10; ++i) printf("A[%i] = %.0f ", i, h_A[i]); printf("\n");
//
//	printf("\n");
//	for (int i = 0; i < (10 + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");
//
//	for (int i = 0; i < 10; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);
//	*/
//	// --- Allocating and defining dense host and device data vectors
//	double *h_x = (double *)malloc(Nrows * sizeof(double));
//	/*h_x[0] = 100.0;  h_x[1] = 200.0; h_x[2] = 400.0; h_x[3] = 500.0;*/
//	for (int i = 0; i < N; i++){
//		h_x[i] = f[i];
//	}
//	h_x[0] = h_x[1] = 0;
//
//	double *d_x;        gpuErrchk(cudaMalloc(&d_x, Nrows * sizeof(double)));
//	gpuErrchk(cudaMemcpy(d_x, h_x, Nrows * sizeof(double), cudaMemcpyHostToDevice));
//
//	/******************************************/
//	/* STEP 1: CREATE DESCRIPTORS FOR L AND U */
//	/******************************************/
//	cusparseMatDescr_t      descr_L = 0;
//	cusparseSafeCall(cusparseCreateMatDescr(&descr_L));
//	cusparseSafeCall(cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ONE));
//	cusparseSafeCall(cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL));
//	cusparseSafeCall(cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER));
//	cusparseSafeCall(cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_NON_UNIT));
//
//	/********************************************************************************************************/
//	/* STEP 2: QUERY HOW MUCH MEMORY USED IN CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
//	/********************************************************************************************************/
//	csric02Info_t info_A = 0;  cusparseSafeCall(cusparseCreateCsric02Info(&info_A));
//	csrsv2Info_t  info_L = 0;  cusparseSafeCall(cusparseCreateCsrsv2Info(&info_L));
//	csrsv2Info_t  info_Lt = 0;  cusparseSafeCall(cusparseCreateCsrsv2Info(&info_Lt));
//
//	int pBufferSize_M, pBufferSize_L, pBufferSize_Lt;
//	cusparseSafeCall(cusparseDcsric02_bufferSize(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, &pBufferSize_M));
//	cusparseSafeCall(cusparseDcsrsv2_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, &pBufferSize_L));
//	cusparseSafeCall(cusparseDcsrsv2_bufferSize(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, &pBufferSize_Lt));
//
//	int pBufferSize = max(pBufferSize_M, max(pBufferSize_L, pBufferSize_Lt));
//	void *pBuffer = 0;  gpuErrchk(cudaMalloc((void**)&pBuffer, pBufferSize));
//
//	/******************************************************************************************************/
//	/* STEP 3: ANALYZE THE THREE PROBLEMS: CHOLESKY FACTORIZATION AND THE TWO FOLLOWING SYSTEM INVERSIONS */
//	/******************************************************************************************************/
//	int structural_zero;
//
//	cusparseSafeCall(cusparseDcsric02_analysis(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
//
//	cusparseStatus_t status = cusparseXcsric02_zeroPivot(handle, info_A, &structural_zero);
//	if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("A(%d,%d) is missing\n", structural_zero, structural_zero); }
//
//	cusparseSafeCall(cusparseDcsrsv2_analysis(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
//	cusparseSafeCall(cusparseDcsrsv2_analysis(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));
//
//	/*************************************/
//	/* STEP 4: FACTORIZATION: A = L * L' */
//	/*************************************/
//	int numerical_zero;
//
//	cusparseSafeCall(cusparseDcsric02(handle, N, nnz, descrA, d_A, d_A_RowIndices, d_A_ColIndices, info_A, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
//	status = cusparseXcsric02_zeroPivot(handle, info_A, &numerical_zero);
//	/*if (CUSPARSE_STATUS_ZERO_PIVOT == status){ printf("L(%d,%d) is zero\n", numerical_zero, numerical_zero); }
//	*/
//
//	gpuErrchk(cudaMemcpy(h_A, d_A, nnz * sizeof(double), cudaMemcpyDeviceToHost));
//	/*printf("\nNon-zero elements in Cholesky matrix\n\n");
//	for (int k = 0; k<10; k++) printf("%f\n", h_A[k]);*/
//
//	cusparseSafeCall(cusparseDcsr2dense(handle, Nrows, Ncols, descrA, d_A, d_A_RowIndices, d_A_ColIndices, d_A_dense, Nrows));
//
//	/*printf("\nCholesky matrix\n\n");
//	for (int i = 0; i < 10; i++) {
//	std::cout << "[ ";
//	for (int j = 0; j < 10; j++)
//	std::cout << h_A_dense[i * Ncols + j] << " ";
//	std::cout << "]\n";
//	}*/
//
//	/*********************/
//	/* STEP 5: L * z = x */
//	/*********************/
//	// --- Allocating the intermediate result vector
//	double *d_z;        gpuErrchk(cudaMalloc(&d_z, N * sizeof(double)));
//
//	const double alpha = 1.;
//	cusparseSafeCall(cusparseDcsrsv2_solve(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_L, d_x, d_z, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer));
//
//	/**********************/
//	/* STEP 5: L' * y = z */
//	/**********************/
//	// --- Allocating the host and device side result vector
//	double *h_y = (double *)malloc(Ncols * sizeof(double));
//	double *d_y;        gpuErrchk(cudaMalloc(&d_y, Ncols * sizeof(double)));
//
//	cusparseSafeCall(cusparseDcsrsv2_solve(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, d_z, d_y, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));
//
//	cudaMemcpy(h_x, d_y, N * sizeof(double), cudaMemcpyDeviceToHost);
//	/*printf("\n\nFinal result\n");
//	for (int k = 0; k<10; k++) printf("x[%i] = %f\n", k, h_x[k]);
//	*/
//	for (int i = 0; i < numNodes; i++) {
//		x[i] = x[i] + h_x[i * 2];
//		y[i] = y[i] + h_x[i * 2 + 1];
//	}
//	cudaFree(d_A_dense);
//	cudaFree(d_nnzPerVector);
//	cudaFree(d_A);
//	cudaFree(d_A_RowIndices);
//	cudaFree(d_A_ColIndices);
//	cudaFree(d_x);
//	cudaFree(pBuffer);
//	cudaFree(d_z);
//	cudaFree(d_y);
//
//	free(h_nnzPerVector);
//
//	free(h_A_dense);
//
//	free(h_A);
//	free(h_A_RowIndices);
//	free(h_A_ColIndices);
//	free(h_x);
//	free(h_y);
//
//	return 0;
//}#if 0

#endif // 0  





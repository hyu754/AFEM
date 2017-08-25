




#ifndef AFEM_LINEAR_ROUTINES_H
#define AFEM_LINEAR_ROUTINES_H

#include "cuda.h"
#include <cusolverDn.h>
#include <cusolverSp.h>
#include <cusparse.h>
#include <cuda_runtime.h> 
#include "cublas_v2.h" 
#include <cusolverDn.h>
#include <cusparse_v2.h>
#include "AFEM_geometry.hpp"
#include "glm\glm.hpp"


//Corotational dynamic GPU functions
//M u_dd(t) + C u_d(t) + K (x-x_0) = f(t)
__device__ void find_Jacobian_localK_localM_corotational(AFEM::element *in_element, AFEM::position_3D *in_pos,AFEM::position_3D *original_pos);
__global__ void gpu_make_K_corotational(AFEM::element *in_vec, AFEM::position_3D *in_pos, AFEM::position_3D *original_pos, int numElem, int numNodes, float *K_d, float *M_d, float *f_d, float *RK,  float E = 25000.0, float nu = 0.492);
__global__ void find_A_b_dynamic_corotational(float *K_in, float *dx_in, float *x_zero, float *x_current, float *u_dot, float *f_ext, float *RHS, float *M_in, float *LHS, float *RKx, int num_nodes, float dt, int dim);


//Corotational energy minimisation functions
//Min stretch and strain energy

__global__ void gpu_make_K_energy_corotational(AFEM::element *in_vec, AFEM::position_3D *in_pos, AFEM::position_3D *original_pos, int numElem, int numNodes, float *K_d, float *M_d, float *f_d, float *RKx_matrix_d);

__global__ void find_A_b_energy_minimisation_corotational(float *K_in, float *dx_in, float *u_dot, float *f_ext, float *RHS, float *M_in, float *LHS, int num_nodes, float dt, float alpha, float rk, int dim, AFEM::stationary *stationary, int num_station, AFEM::sudo_force_struct * sudo_force_vec, int *sudo_force_ind, int number_of_sudo_force);
/*
//Common functions used for CUDA and corotational formulation
*/
//Atomic add for global K matrix assembly
__device__ inline float atomicAdda(float* address, double value)

{

	float ret = atomicExch(address, 0.0f);

	float old = ret + (float)value;

	while ((old = atomicExch(address, old)) != 0.0f)

	{

		old = atomicExch(address, 0.0f) + old;

	}

	return ret;

};

//Finds the corotation matrix R[3][3]

__device__ void find_rotation_corotational(float R[3][3], AFEM::element *in_element, AFEM::position_3D *current_pos, AFEM::position_3D *original_pos);
#endif //AFEM_LINEAR_ROUTINES_H

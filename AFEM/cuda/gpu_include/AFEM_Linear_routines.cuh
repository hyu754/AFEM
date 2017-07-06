




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


//Corotational GPU functions
__device__ void find_Jacobian_localK_localM_corotational(AFEM::element *in_element, AFEM::position_3D *in_pos,AFEM::position_3D *original_pos);
__global__ void gpu_make_K_corotational(AFEM::element *in_vec, AFEM::position_3D *in_pos, AFEM::position_3D *original_pos, int numElem, int numNodes, float *K_d, float *M_d, float *f_d, float *RK);
__global__ void find_A_b_dynamic_corotational(float *K_in, float *dx_in, float *x_zero, float *x_current, float *u_dot, float *f_ext, float *RHS, float *M_in, float *LHS, float *RK, int num_nodes, float dt, int dim);




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



#endif //AFEM_LINEAR_ROUTINES_H

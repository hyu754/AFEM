

#include <stdio.h>
#include <math.h>
#include <iostream>
#include "AFEM_Linear_routines.cuh"

#include "cuda.h"
#include <cuda_runtime.h>
#include <fstream>
#include <glm/glm.hpp>


#define nodesinelemX(node,el,nodesPerElem) (node + nodesPerElem*el) //the first entry is the element # the second entry would be the element number and the last one is the number of nodes/element
#define threeD21D(row_d,col_d,el_d,width_d,depth_d) (row_d+width_d*(col_d+depth_d*el_d)) //
#define nodesDisplacementX(dof,node,dimension) (dof + node*dimension)
#define IDX2C(i,j,ld) (((j)*(ld))+( i )) 

//Define gravity
#define GRAVITY -9.81
__device__ void find_rotation_corotational(float R[3][3], AFEM::element *in_element, AFEM::position_3D *current_pos, AFEM::position_3D *original_pos){
	//Current nodal positions
	AFEM::position_3D q1, q2, q3, q4;
	q1 = current_pos[in_element->nodes_in_elem[0]];
	q2 = current_pos[in_element->nodes_in_elem[1]];
	q3 = current_pos[in_element->nodes_in_elem[2]];
	q4 = current_pos[in_element->nodes_in_elem[3]];

	//Original nodal positions
	AFEM::position_3D p1, p2, p3, p4;
	p1 = original_pos[in_element->nodes_in_elem[0]];
	p2 = original_pos[in_element->nodes_in_elem[1]];
	p3 = original_pos[in_element->nodes_in_elem[2]];
	p4 = original_pos[in_element->nodes_in_elem[3]];


	//omega = Q P.inv
	float omega[3][3];

	//omega[0][0] = (-1.0*p1.x*p2.z*q3.y + 1.0*p1.x*p2.z*q4.y + 1.0*p1.x*p3.z*q2.y - 1.0*p1.x*p3.z*q4.y - 1.0*p1.x*p4.z*q2.y + 1.0*p1.x*p4.z*q3.y + 1.0*p1.z*p2.x*q3.y - 1.0*p1.z*p2.x*q4.y - 1.0*p1.z*p3.x*q2.y + 1.0*p1.z*p3.x*q4.y + 1.0*p1.z*p4.x*q2.y - 1.0*p1.z*p4.x*q3.y - 1.0*p2.x*p3.z*q1.y + 1.0*p2.x*p3.z*q4.y + 1.0*p2.x*p4.z*q1.y - 1.0*p2.x*p4.z*q3.y + 1.0*p2.z*p3.x*q1.y - 1.0*p2.z*p3.x*q4.y - 1.0*p2.z*p4.x*q1.y + 1.0*p2.z*p4.x*q3.y - 1.0*p3.x*p4.z*q1.y + 1.0*p3.x*p4.z*q2.y + 1.0*p3.z*p4.x*q1.y - 1.0*p3.z*p4.x*q2.y) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[0][1] = 1.0*(p1.x*p2.y*q3.y - p1.x*p2.y*q4.y - p1.x*p3.y*q2.y + p1.x*p3.y*q4.y + p1.x*p4.y*q2.y - p1.x*p4.y*q3.y - p1.y*p2.x*q3.y + p1.y*p2.x*q4.y + p1.y*p3.x*q2.y - p1.y*p3.x*q4.y - p1.y*p4.x*q2.y + p1.y*p4.x*q3.y + p2.x*p3.y*q1.y - p2.x*p3.y*q4.y - p2.x*p4.y*q1.y + p2.x*p4.y*q3.y - p2.y*p3.x*q1.y + p2.y*p3.x*q4.y + p2.y*p4.x*q1.y - p2.y*p4.x*q3.y + p3.x*p4.y*q1.y - p3.x*p4.y*q2.y - p3.y*p4.x*q1.y + p3.y*p4.x*q2.y) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[0][2] = 1.0*(p1.x*p2.y*p3.z*q4.y - p1.x*p2.y*p4.z*q3.y - p1.x*p2.z*p3.y*q4.y + p1.x*p2.z*p4.y*q3.y + p1.x*p3.y*p4.z*q2.y - p1.x*p3.z*p4.y*q2.y - p1.y*p2.x*p3.z*q4.y + p1.y*p2.x*p4.z*q3.y + p1.y*p2.z*p3.x*q4.y - p1.y*p2.z*p4.x*q3.y - p1.y*p3.x*p4.z*q2.y + p1.y*p3.z*p4.x*q2.y + p1.z*p2.x*p3.y*q4.y - p1.z*p2.x*p4.y*q3.y - p1.z*p2.y*p3.x*q4.y + p1.z*p2.y*p4.x*q3.y + p1.z*p3.x*p4.y*q2.y - p1.z*p3.y*p4.x*q2.y - p2.x*p3.y*p4.z*q1.y + p2.x*p3.z*p4.y*q1.y + p2.y*p3.x*p4.z*q1.y - p2.y*p3.z*p4.x*q1.y - p2.z*p3.x*p4.y*q1.y + p2.z*p3.y*p4.x*q1.y) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[1][0] = (-1.0*p1.x*p2.z*q3.z + 1.0*p1.x*p2.z*q4.z + 1.0*p1.x*p3.z*q2.z - 1.0*p1.x*p3.z*q4.z - 1.0*p1.x*p4.z*q2.z + 1.0*p1.x*p4.z*q3.z + 1.0*p1.z*p2.x*q3.z - 1.0*p1.z*p2.x*q4.z - 1.0*p1.z*p3.x*q2.z + 1.0*p1.z*p3.x*q4.z + 1.0*p1.z*p4.x*q2.z - 1.0*p1.z*p4.x*q3.z - 1.0*p2.x*p3.z*q1.z + 1.0*p2.x*p3.z*q4.z + 1.0*p2.x*p4.z*q1.z - 1.0*p2.x*p4.z*q3.z + 1.0*p2.z*p3.x*q1.z - 1.0*p2.z*p3.x*q4.z - 1.0*p2.z*p4.x*q1.z + 1.0*p2.z*p4.x*q3.z - 1.0*p3.x*p4.z*q1.z + 1.0*p3.x*p4.z*q2.z + 1.0*p3.z*p4.x*q1.z - 1.0*p3.z*p4.x*q2.z) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[1][1] = 1.0*(p1.x*p2.y*q3.z - p1.x*p2.y*q4.z - p1.x*p3.y*q2.z + p1.x*p3.y*q4.z + p1.x*p4.y*q2.z - p1.x*p4.y*q3.z - p1.y*p2.x*q3.z + p1.y*p2.x*q4.z + p1.y*p3.x*q2.z - p1.y*p3.x*q4.z - p1.y*p4.x*q2.z + p1.y*p4.x*q3.z + p2.x*p3.y*q1.z - p2.x*p3.y*q4.z - p2.x*p4.y*q1.z + p2.x*p4.y*q3.z - p2.y*p3.x*q1.z + p2.y*p3.x*q4.z + p2.y*p4.x*q1.z - p2.y*p4.x*q3.z + p3.x*p4.y*q1.z - p3.x*p4.y*q2.z - p3.y*p4.x*q1.z + p3.y*p4.x*q2.z) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[1][2] = 1.0*(p1.x*p2.y*p3.z*q4.z - p1.x*p2.y*p4.z*q3.z - p1.x*p2.z*p3.y*q4.z + p1.x*p2.z*p4.y*q3.z + p1.x*p3.y*p4.z*q2.z - p1.x*p3.z*p4.y*q2.z - p1.y*p2.x*p3.z*q4.z + p1.y*p2.x*p4.z*q3.z + p1.y*p2.z*p3.x*q4.z - p1.y*p2.z*p4.x*q3.z - p1.y*p3.x*p4.z*q2.z + p1.y*p3.z*p4.x*q2.z + p1.z*p2.x*p3.y*q4.z - p1.z*p2.x*p4.y*q3.z - p1.z*p2.y*p3.x*q4.z + p1.z*p2.y*p4.x*q3.z + p1.z*p3.x*p4.y*q2.z - p1.z*p3.y*p4.x*q2.z - p2.x*p3.y*p4.z*q1.z + p2.x*p3.z*p4.y*q1.z + p2.y*p3.x*p4.z*q1.z - p2.y*p3.z*p4.x*q1.z - p2.z*p3.x*p4.y*q1.z + p2.z*p3.y*p4.x*q1.z) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	//omega[2][0] = 0;
	//omega[2][1] = 0;
	//omega[2][2] = 1.00000000000000;
	omega[0][0] = (p1.y*p2.z*q3.x - p1.y*p2.z*q4.x - p1.y*p3.z*q2.x + p1.y*p3.z*q4.x + p1.y*p4.z*q2.x - p1.y*p4.z*q3.x - p1.z*p2.y*q3.x + p1.z*p2.y*q4.x + p1.z*p3.y*q2.x - p1.z*p3.y*q4.x - p1.z*p4.y*q2.x + p1.z*p4.y*q3.x + p2.y*p3.z*q1.x - p2.y*p3.z*q4.x - p2.y*p4.z*q1.x + p2.y*p4.z*q3.x - p2.z*p3.y*q1.x + p2.z*p3.y*q4.x + p2.z*p4.y*q1.x - p2.z*p4.y*q3.x + p3.y*p4.z*q1.x - p3.y*p4.z*q2.x - p3.z*p4.y*q1.x + p3.z*p4.y*q2.x) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[0][1] = (-p1.x*p2.z*q3.x + p1.x*p2.z*q4.x + p1.x*p3.z*q2.x - p1.x*p3.z*q4.x - p1.x*p4.z*q2.x + p1.x*p4.z*q3.x + p1.z*p2.x*q3.x - p1.z*p2.x*q4.x - p1.z*p3.x*q2.x + p1.z*p3.x*q4.x + p1.z*p4.x*q2.x - p1.z*p4.x*q3.x - p2.x*p3.z*q1.x + p2.x*p3.z*q4.x + p2.x*p4.z*q1.x - p2.x*p4.z*q3.x + p2.z*p3.x*q1.x - p2.z*p3.x*q4.x - p2.z*p4.x*q1.x + p2.z*p4.x*q3.x - p3.x*p4.z*q1.x + p3.x*p4.z*q2.x + p3.z*p4.x*q1.x - p3.z*p4.x*q2.x) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[0][2] = (-(p1.x - p4.x)*((q2.x - q4.x)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (q3.x - q4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x))) + (q1.x - q4.x)*((p2.x - p4.x)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)))) / (-(p1.x - p4.x)*(p3.z - p4.z)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + (p1.z - p4.z)*(p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + ((p1.x - p4.x)*(p2.z - p4.z) - (p1.z - p4.z)*(p2.x - p4.x))*((p1.x - p4.x)*(p3.y - p4.y) - (p1.y - p4.y)*(p3.x - p4.x)));
	omega[1][0] = (p1.y*p2.z*q3.y - p1.y*p2.z*q4.y - p1.y*p3.z*q2.y + p1.y*p3.z*q4.y + p1.y*p4.z*q2.y - p1.y*p4.z*q3.y - p1.z*p2.y*q3.y + p1.z*p2.y*q4.y + p1.z*p3.y*q2.y - p1.z*p3.y*q4.y - p1.z*p4.y*q2.y + p1.z*p4.y*q3.y + p2.y*p3.z*q1.y - p2.y*p3.z*q4.y - p2.y*p4.z*q1.y + p2.y*p4.z*q3.y - p2.z*p3.y*q1.y + p2.z*p3.y*q4.y + p2.z*p4.y*q1.y - p2.z*p4.y*q3.y + p3.y*p4.z*q1.y - p3.y*p4.z*q2.y - p3.z*p4.y*q1.y + p3.z*p4.y*q2.y) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[1][1] = (-p1.x*p2.z*q3.y + p1.x*p2.z*q4.y + p1.x*p3.z*q2.y - p1.x*p3.z*q4.y - p1.x*p4.z*q2.y + p1.x*p4.z*q3.y + p1.z*p2.x*q3.y - p1.z*p2.x*q4.y - p1.z*p3.x*q2.y + p1.z*p3.x*q4.y + p1.z*p4.x*q2.y - p1.z*p4.x*q3.y - p2.x*p3.z*q1.y + p2.x*p3.z*q4.y + p2.x*p4.z*q1.y - p2.x*p4.z*q3.y + p2.z*p3.x*q1.y - p2.z*p3.x*q4.y - p2.z*p4.x*q1.y + p2.z*p4.x*q3.y - p3.x*p4.z*q1.y + p3.x*p4.z*q2.y + p3.z*p4.x*q1.y - p3.z*p4.x*q2.y) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[1][2] = (-(p1.x - p4.x)*((q2.y - q4.y)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (q3.y - q4.y)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x))) + (q1.y - q4.y)*((p2.x - p4.x)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)))) / (-(p1.x - p4.x)*(p3.z - p4.z)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + (p1.z - p4.z)*(p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + ((p1.x - p4.x)*(p2.z - p4.z) - (p1.z - p4.z)*(p2.x - p4.x))*((p1.x - p4.x)*(p3.y - p4.y) - (p1.y - p4.y)*(p3.x - p4.x)));
	omega[2][0] = (p1.y*p2.z*q3.z - p1.y*p2.z*q4.z - p1.y*p3.z*q2.z + p1.y*p3.z*q4.z + p1.y*p4.z*q2.z - p1.y*p4.z*q3.z - p1.z*p2.y*q3.z + p1.z*p2.y*q4.z + p1.z*p3.y*q2.z - p1.z*p3.y*q4.z - p1.z*p4.y*q2.z + p1.z*p4.y*q3.z + p2.y*p3.z*q1.z - p2.y*p3.z*q4.z - p2.y*p4.z*q1.z + p2.y*p4.z*q3.z - p2.z*p3.y*q1.z + p2.z*p3.y*q4.z + p2.z*p4.y*q1.z - p2.z*p4.y*q3.z + p3.y*p4.z*q1.z - p3.y*p4.z*q2.z - p3.z*p4.y*q1.z + p3.z*p4.y*q2.z) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[2][1] = (-p1.x*p2.z*q3.z + p1.x*p2.z*q4.z + p1.x*p3.z*q2.z - p1.x*p3.z*q4.z - p1.x*p4.z*q2.z + p1.x*p4.z*q3.z + p1.z*p2.x*q3.z - p1.z*p2.x*q4.z - p1.z*p3.x*q2.z + p1.z*p3.x*q4.z + p1.z*p4.x*q2.z - p1.z*p4.x*q3.z - p2.x*p3.z*q1.z + p2.x*p3.z*q4.z + p2.x*p4.z*q1.z - p2.x*p4.z*q3.z + p2.z*p3.x*q1.z - p2.z*p3.x*q4.z - p2.z*p4.x*q1.z + p2.z*p4.x*q3.z - p3.x*p4.z*q1.z + p3.x*p4.z*q2.z + p3.z*p4.x*q1.z - p3.z*p4.x*q2.z) / (p1.x*p2.y*p3.z - p1.x*p2.y*p4.z - p1.x*p2.z*p3.y + p1.x*p2.z*p4.y + p1.x*p3.y*p4.z - p1.x*p3.z*p4.y - p1.y*p2.x*p3.z + p1.y*p2.x*p4.z + p1.y*p2.z*p3.x - p1.y*p2.z*p4.x - p1.y*p3.x*p4.z + p1.y*p3.z*p4.x + p1.z*p2.x*p3.y - p1.z*p2.x*p4.y - p1.z*p2.y*p3.x + p1.z*p2.y*p4.x + p1.z*p3.x*p4.y - p1.z*p3.y*p4.x - p2.x*p3.y*p4.z + p2.x*p3.z*p4.y + p2.y*p3.x*p4.z - p2.y*p3.z*p4.x - p2.z*p3.x*p4.y + p2.z*p3.y*p4.x);
	omega[2][2] = (-(p1.x - p4.x)*((q2.z - q4.z)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (q3.z - q4.z)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x))) + (q1.z - q4.z)*((p2.x - p4.x)*((p1.x - p4.x)*(-p3.y + p4.y) + (p1.y - p4.y)*(p3.x - p4.x)) + (p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)))) / (-(p1.x - p4.x)*(p3.z - p4.z)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + (p1.z - p4.z)*(p3.x - p4.x)*((p1.x - p4.x)*(p2.y - p4.y) - (p1.y - p4.y)*(p2.x - p4.x)) + ((p1.x - p4.x)*(p2.z - p4.z) - (p1.z - p4.z)*(p2.x - p4.x))*((p1.x - p4.x)*(p3.y - p4.y) - (p1.y - p4.y)*(p3.x - p4.x))); 
	//perform gram schmit

	glm::vec3 a0(omega[0][0], omega[1][0], omega[2][0]);
	glm::vec3 a1(omega[0][1], omega[1][1], omega[2][1]);
	glm::vec3 a2(omega[0][2], omega[1][2], omega[2][2]);

	//glm::vec3 a0(omega[0][0], omega[0][1], omega[0][2]);
	//glm::vec3 a1(omega[1][0], omega[1][1], omega[1][2]);
	//glm::vec3 a2(omega[2][0], omega[2][1], omega[2][2]);
	//

	glm::vec3 r0 = a0 / glm::length(a0);
	glm::vec3 r1 = (a1 - r0*glm::dot(r0, a1));
	r1 = r1 / glm::length(r1);
	glm::vec3 r2 = (a2 - r1*glm::dot(r1, a2));
	r2 = r2 / glm::length(r2);
	r2 = glm::cross(r0, r1);
	//printf("%f %f %f \n ", r1.x, r1.y, r1.z);
	
	//assign above to R
	//col0
	R[0][0] = r0[0];
	R[1][0] = r0[1];
	R[2][0] = r0[2];

	//col1
	R[0][1] = r1[0];
	R[1][1] = r1[1];
	R[2][1] = r1[2];

	//col2 
	R[0][2] = r2[0];
	R[1][2] = r2[1];
	R[2][2] = r2[2];

	//////col0
	//R[0][0] = r0[0];
	//R[0][1] = r0[1];
	//R[0][2] = r0[2];

	////col1
	//R[1][0] = r1[0];
	//R[1][1] = r1[1];
	//R[1][2] = r1[2];

	////col2 
	//R[2][0] = r2[0];
	//R[2][1] = r2[1];
	//R[2][2] = r2[2];
	


	


}
__device__ void find_Jacobian_localK_localM_corotational(AFEM::element *in_element, AFEM::position_3D *in_pos, AFEM::position_3D *original_pos){

	/*float x14 = in_element->position_info[0].x - in_element->position_info[3].x;
	float x24 = in_element->position_info[1].x - in_element->position_info[3].x;
	float x34 = in_element->position_info[2].x - in_element->position_info[3].x;*/
	//Original nodal positions
	//Original nodal positions
	AFEM::position_3D p1, p2, p3, p4;
	p1 = original_pos[in_element->nodes_in_elem[0]];
	p2 = original_pos[in_element->nodes_in_elem[1]];
	p3 = original_pos[in_element->nodes_in_elem[2]];
	p4 = original_pos[in_element->nodes_in_elem[3]];

	//four node positions
	AFEM::position_3D n1, n2, n3, n4;
	n1 = in_pos[in_element->nodes_in_elem[0]];
	n2 = in_pos[in_element->nodes_in_elem[1]];
	n3 = in_pos[in_element->nodes_in_elem[2]];
	n4 = in_pos[in_element->nodes_in_elem[3]];


	float x14 = n1.x - n4.x;
	float x24 = n2.x - n4.x;
	float x34 = n3.x - n4.x;

	float y14 = n1.y - n4.y;
	float y24 = n2.y - n4.y;
	float y34 = n3.y - n4.y;


	float z14 = n1.z - n4.z;
	float z24 = n2.z - n4.z;
	float z34 = n3.z - n4.z;



	//std::cout << x14*(y24*z34 - y34*z24) - y14*(x24*z34 - z24 * 34) + z14*(x24*y34 - y24*x34) << std::endl;
	float det_J = (x14*(y24*z34 - y34*z24) - y14*(x24*z34 - z24 * x34) + z14*(x24*y34 - y24*x34));
	float J_bar11 = (y24*z34 - z24*y34) / det_J;
	float J_bar12 = (z14*y34 - y14*z34) / det_J;
	float J_bar13 = (y14*z24 - z14*y24) / det_J;

	float J_bar21 = (z24*x34 - x24*z34) / det_J;
	float J_bar22 = (x14*z34 - z14*x34) / det_J;
	float J_bar23 = (z14*x24 - x14*z24) / det_J;

	float J_bar31 = (x24*y34 - y24*x34) / det_J;
	float J_bar32 = (y14*x34 - x14*y34) / det_J;
	float J_bar33 = (x14*y24 - y14*x24) / det_J;

	float J_star1 = -(J_bar11 + J_bar12 + J_bar13);
	float J_star2 = -(J_bar21 + J_bar22 + J_bar23);
	float J_star3 = -(J_bar31 + J_bar32 + J_bar33);


	//Initi
	float R[3][3];
	find_rotation_corotational(R, in_element, in_pos, original_pos);
	
	//float R11(R[0][0]), R12(R[1][0]), R13(R[2][0]), R21(R[0][1]), R22(R[1][1]), R23(R[2][1]), R31(R[0][2]), R32(R[1][2]), R33(R[2][2]);
	//float R11(1.0), R12(0.0), R13(0.0), R21(0.0), R22(1.0), R23(0.0), R31(0.0), R32(0.0), R33(1.0);



	float rho = 1000.0;

	float B[6][12];


	//B transpose
	float BT[12][6];




	B[0][0] = J_bar11;
	B[0][1] = 0;
	B[0][2] = 0;
	B[0][3] = J_bar12;
	B[0][4] = 0;
	B[0][5] = 0;
	B[0][6] = J_bar13;
	B[0][7] = 0;
	B[0][8] = 0;
	B[0][9] = J_star1;
	B[0][10] = 0;
	B[0][11] = 0;
	B[1][0] = 0;
	B[1][1] = J_bar21;
	B[1][2] = 0;
	B[1][3] = 0;
	B[1][4] = J_bar22;
	B[1][5] = 0;
	B[1][6] = 0;
	B[1][7] = J_bar23;
	B[1][8] = 0;
	B[1][9] = 0;
	B[1][10] = J_star2;
	B[1][11] = 0;
	B[2][0] = 0;
	B[2][1] = 0;
	B[2][2] = J_bar31;
	B[2][3] = 0;
	B[2][4] = 0;
	B[2][5] = J_bar32;
	B[2][6] = 0;
	B[2][7] = 0;
	B[2][8] = J_bar33;
	B[2][9] = 0;
	B[2][10] = 0;
	B[2][11] = J_star3;
	B[3][0] = J_bar21;
	B[3][1] = J_bar11;
	B[3][2] = 0;
	B[3][3] = J_bar22;
	B[3][4] = J_bar12;
	B[3][5] = 0;
	B[3][6] = J_bar23;
	B[3][7] = J_bar13;
	B[3][8] = 0;
	B[3][9] = J_star2;
	B[3][10] = J_star1;
	B[3][11] = 0;
	B[4][0] = 0;
	B[4][1] = J_bar31;
	B[4][2] = J_bar21;
	B[4][3] = 0;
	B[4][4] = J_bar32;
	B[4][5] = J_bar22;
	B[4][6] = 0;
	B[4][7] = J_bar33;
	B[4][8] = J_bar23;
	B[4][9] = 0;
	B[4][10] = J_star3;
	B[4][11] = J_star2;
	B[5][0] = J_bar31;
	B[5][1] = 0;
	B[5][2] = J_bar11;
	B[5][3] = J_bar32;
	B[5][4] = 0;
	B[5][5] = J_bar12;
	B[5][6] = J_bar33;
	B[5][7] = 0;
	B[5][8] = J_bar13;
	B[5][9] = J_star3;
	B[5][10] = 0;
	B[5][11] = J_star1;

	BT[0][0] = J_bar11;
	BT[0][1] = 0;
	BT[0][2] = 0;
	BT[0][3] = J_bar21;
	BT[0][4] = 0;
	BT[0][5] = J_bar31;
	BT[1][0] = 0;
	BT[1][1] = J_bar21;
	BT[1][2] = 0;
	BT[1][3] = J_bar11;
	BT[1][4] = J_bar31;
	BT[1][5] = 0;
	BT[2][0] = 0;
	BT[2][1] = 0;
	BT[2][2] = J_bar31;
	BT[2][3] = 0;
	BT[2][4] = J_bar21;
	BT[2][5] = J_bar11;
	BT[3][0] = J_bar12;
	BT[3][1] = 0;
	BT[3][2] = 0;
	BT[3][3] = J_bar22;
	BT[3][4] = 0;
	BT[3][5] = J_bar32;
	BT[4][0] = 0;
	BT[4][1] = J_bar22;
	BT[4][2] = 0;
	BT[4][3] = J_bar12;
	BT[4][4] = J_bar32;
	BT[4][5] = 0;
	BT[5][0] = 0;
	BT[5][1] = 0;
	BT[5][2] = J_bar32;
	BT[5][3] = 0;
	BT[5][4] = J_bar22;
	BT[5][5] = J_bar12;
	BT[6][0] = J_bar13;
	BT[6][1] = 0;
	BT[6][2] = 0;
	BT[6][3] = J_bar23;
	BT[6][4] = 0;
	BT[6][5] = J_bar33;
	BT[7][0] = 0;
	BT[7][1] = J_bar23;
	BT[7][2] = 0;
	BT[7][3] = J_bar13;
	BT[7][4] = J_bar33;
	BT[7][5] = 0;
	BT[8][0] = 0;
	BT[8][1] = 0;
	BT[8][2] = J_bar33;
	BT[8][3] = 0;
	BT[8][4] = J_bar23;
	BT[8][5] = J_bar13;
	BT[9][0] = J_star1;
	BT[9][1] = 0;
	BT[9][2] = 0;
	BT[9][3] = J_star2;
	BT[9][4] = 0;
	BT[9][5] = J_star3;
	BT[10][0] = 0;
	BT[10][1] = J_star2;
	BT[10][2] = 0;
	BT[10][3] = J_star1;
	BT[10][4] = J_star3;
	BT[10][5] = 0;
	BT[11][0] = 0;
	BT[11][1] = 0;
	BT[11][2] = J_star3;
	BT[11][3] = 0;
	BT[11][4] = J_star2;
	BT[11][5] = J_star1;









	in_element->Jacobian = det_J;

	in_element->volume = det_J / 6.0;
	//printf("element volume : %f \n", in_element->volume);
	float E =55000.0;
	float nu = 0.493;

	float D[6][6];

	for (int i = 0; i < 6; i++){

		for (int j = 0; j < 6; j++){

			D[i][j] = 0.0;
		}
	}
	D[0][0] = D[1][1] = D[2][2] = (1.0 - nu);
	D[0][1] = D[1][0] = D[0][2] = D[2][0] = D[1][2] = D[2][1] = nu;
	D[3][3] = D[4][4] = D[5][5] = (1.0 - 2.0 * nu) / 2.0;
	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 6; j++){
			D[i][j] = D[i][j] * (E / (1.0 - nu - 2.0 * nu*nu));
		}
	}


	//First find D*B
	float DB[6][12];
	for (int row = 0; row < 6; row++){
		for (int col = 0; col < 12; col++){
			DB[row][col] = 0.0;
		}
	}

	float testingvalue = 0;
	for (int row = 0; row < 6; row++){
		for (int col = 0; col < 12; col++){
			float sum = 0.0;
			for (int j = 0; j < 6; j++){
				sum += D[row][j] * B[j][col];
				//testingvalue += D[row][j];
			}
			DB[row][col] = sum;
		}
	}
	//Now find and assign element K matrix
	//We will find BT * DB
	float BTDB[12][12];
	for (int row = 0; row < 12; row++){
		for (int col = 0; col < 12; col++){
			BTDB[row][col] = 0.0;
		}
	}


	float volume = fabsf(det_J) / 6.0f;


	for (int row = 0; row < 12; row++){

		for (int col = 0; col < 12; col++){
			float sum = 0.0;
			for (int j = 0; j < 6; j++){
				sum += BT[row][j] * DB[j][col] * volume;


			}

			BTDB[row][col] = sum;

			



			//printf("%f ", in_element->local_K[row + col * 12]);
		}

		//printf("\n");


	}

	//	int position_counter = 0;
	//in_element->local_K[position_counter] = BTDB[row][col];
	//position_counter++;
	//Perform corotation
	float R_large[12][12],Rinv_large[12][12];
	R_large[0][0] = R[0][ 0]; R_large[0][1] = R[0][ 1]; R_large[0][2] = R[0][ 2]; R_large[0][3] = 0; R_large[0][4] = 0; R_large[0][5] = 0; R_large[0][6] = 0; R_large[0][7] = 0; R_large[0][8] = 0; R_large[0][9] = 0; R_large[0][10] = 0; R_large[0][11] = 0; R_large[1][0] = R[1][ 0]; R_large[1][1] = R[1][ 1]; R_large[1][2] = R[1][ 2]; R_large[1][3] = 0; R_large[1][4] = 0; R_large[1][5] = 0; R_large[1][6] = 0; R_large[1][7] = 0; R_large[1][8] = 0; R_large[1][9] = 0; R_large[1][10] = 0; R_large[1][11] = 0; R_large[2][0] = R[2][ 0]; R_large[2][1] = R[2][ 1]; R_large[2][2] = R[2][ 2]; R_large[2][3] = 0; R_large[2][4] = 0; R_large[2][5] = 0; R_large[2][6] = 0; R_large[2][7] = 0; R_large[2][8] = 0; R_large[2][9] = 0; R_large[2][10] = 0; R_large[2][11] = 0; R_large[3][0] = 0; R_large[3][1] = 0; R_large[3][2] = 0; R_large[3][3] = R[0][ 0]; R_large[3][4] = R[0][ 1]; R_large[3][5] = R[0][ 2]; R_large[3][6] = 0; R_large[3][7] = 0; R_large[3][8] = 0; R_large[3][9] = 0; R_large[3][10] = 0; R_large[3][11] = 0; R_large[4][0] = 0; R_large[4][1] = 0; R_large[4][2] = 0; R_large[4][3] = R[1][ 0]; R_large[4][4] = R[1][ 1]; R_large[4][5] = R[1][ 2]; R_large[4][6] = 0; R_large[4][7] = 0; R_large[4][8] = 0; R_large[4][9] = 0; R_large[4][10] = 0; R_large[4][11] = 0; R_large[5][0] = 0; R_large[5][1] = 0; R_large[5][2] = 0; R_large[5][3] = R[2][ 0]; R_large[5][4] = R[2][ 1]; R_large[5][5] = R[2][ 2]; R_large[5][6] = 0; R_large[5][7] = 0; R_large[5][8] = 0; R_large[5][9] = 0; R_large[5][10] = 0; R_large[5][11] = 0; R_large[6][0] = 0; R_large[6][1] = 0; R_large[6][2] = 0; R_large[6][3] = 0; R_large[6][4] = 0; R_large[6][5] = 0; R_large[6][6] = R[0][ 0]; R_large[6][7] = R[0][ 1]; R_large[6][8] = R[0][ 2]; R_large[6][9] = 0; R_large[6][10] = 0; R_large[6][11] = 0; R_large[7][0] = 0; R_large[7][1] = 0; R_large[7][2] = 0; R_large[7][3] = 0; R_large[7][4] = 0; R_large[7][5] = 0; R_large[7][6] = R[1][ 0]; R_large[7][7] = R[1][ 1]; R_large[7][8] = R[1][ 2]; R_large[7][9] = 0; R_large[7][10] = 0; R_large[7][11] = 0; R_large[8][0] = 0; R_large[8][1] = 0; R_large[8][2] = 0; R_large[8][3] = 0; R_large[8][4] = 0; R_large[8][5] = 0; R_large[8][6] = R[2][ 0]; R_large[8][7] = R[2][ 1]; R_large[8][8] = R[2][ 2]; R_large[8][9] = 0; R_large[8][10] = 0; R_large[8][11] = 0; R_large[9][0] = 0; R_large[9][1] = 0; R_large[9][2] = 0; R_large[9][3] = 0; R_large[9][4] = 0; R_large[9][5] = 0; R_large[9][6] = 0; R_large[9][7] = 0; R_large[9][8] = 0; R_large[9][9] = R[0][ 0]; R_large[9][10] = R[0][ 1]; R_large[9][11] = R[0][ 2]; R_large[10][0] = 0; R_large[10][1] = 0; R_large[10][2] = 0; R_large[10][3] = 0; R_large[10][4] = 0; R_large[10][5] = 0; R_large[10][6] = 0; R_large[10][7] = 0; R_large[10][8] = 0; R_large[10][9] = R[1][ 0]; R_large[10][10] = R[1][ 1]; R_large[10][11] = R[1][ 2]; R_large[11][0] = 0; R_large[11][1] = 0; R_large[11][2] = 0; R_large[11][3] = 0; R_large[11][4] = 0; R_large[11][5] = 0; R_large[11][6] = 0; R_large[11][7] = 0; R_large[11][8] = 0; R_large[11][9] = R[2][ 0]; R_large[11][10] = R[2][ 1]; R_large[11][11] = R[2][ 2];
	Rinv_large[0][0] = R[0][ 0]; Rinv_large[0][1] = R[1][ 0]; Rinv_large[0][2] = R[2][ 0]; Rinv_large[0][3] = 0; Rinv_large[0][4] = 0; Rinv_large[0][5] = 0; Rinv_large[0][6] = 0; Rinv_large[0][7] = 0; Rinv_large[0][8] = 0; Rinv_large[0][9] = 0; Rinv_large[0][10] = 0; Rinv_large[0][11] = 0; Rinv_large[1][0] = R[0][ 1]; Rinv_large[1][1] = R[1][ 1]; Rinv_large[1][2] = R[2][ 1]; Rinv_large[1][3] = 0; Rinv_large[1][4] = 0; Rinv_large[1][5] = 0; Rinv_large[1][6] = 0; Rinv_large[1][7] = 0; Rinv_large[1][8] = 0; Rinv_large[1][9] = 0; Rinv_large[1][10] = 0; Rinv_large[1][11] = 0; Rinv_large[2][0] = R[0][ 2]; Rinv_large[2][1] = R[1][ 2]; Rinv_large[2][2] = R[2][ 2]; Rinv_large[2][3] = 0; Rinv_large[2][4] = 0; Rinv_large[2][5] = 0; Rinv_large[2][6] = 0; Rinv_large[2][7] = 0; Rinv_large[2][8] = 0; Rinv_large[2][9] = 0; Rinv_large[2][10] = 0; Rinv_large[2][11] = 0; Rinv_large[3][0] = 0; Rinv_large[3][1] = 0; Rinv_large[3][2] = 0; Rinv_large[3][3] = R[0][ 0]; Rinv_large[3][4] = R[1][ 0]; Rinv_large[3][5] = R[2][ 0]; Rinv_large[3][6] = 0; Rinv_large[3][7] = 0; Rinv_large[3][8] = 0; Rinv_large[3][9] = 0; Rinv_large[3][10] = 0; Rinv_large[3][11] = 0; Rinv_large[4][0] = 0; Rinv_large[4][1] = 0; Rinv_large[4][2] = 0; Rinv_large[4][3] = R[0][ 1]; Rinv_large[4][4] = R[1][ 1]; Rinv_large[4][5] = R[2][ 1]; Rinv_large[4][6] = 0; Rinv_large[4][7] = 0; Rinv_large[4][8] = 0; Rinv_large[4][9] = 0; Rinv_large[4][10] = 0; Rinv_large[4][11] = 0; Rinv_large[5][0] = 0; Rinv_large[5][1] = 0; Rinv_large[5][2] = 0; Rinv_large[5][3] = R[0][ 2]; Rinv_large[5][4] = R[1][ 2]; Rinv_large[5][5] = R[2][ 2]; Rinv_large[5][6] = 0; Rinv_large[5][7] = 0; Rinv_large[5][8] = 0; Rinv_large[5][9] = 0; Rinv_large[5][10] = 0; Rinv_large[5][11] = 0; Rinv_large[6][0] = 0; Rinv_large[6][1] = 0; Rinv_large[6][2] = 0; Rinv_large[6][3] = 0; Rinv_large[6][4] = 0; Rinv_large[6][5] = 0; Rinv_large[6][6] = R[0][ 0]; Rinv_large[6][7] = R[1][ 0]; Rinv_large[6][8] = R[2][ 0]; Rinv_large[6][9] = 0; Rinv_large[6][10] = 0; Rinv_large[6][11] = 0; Rinv_large[7][0] = 0; Rinv_large[7][1] = 0; Rinv_large[7][2] = 0; Rinv_large[7][3] = 0; Rinv_large[7][4] = 0; Rinv_large[7][5] = 0; Rinv_large[7][6] = R[0][ 1]; Rinv_large[7][7] = R[1][ 1]; Rinv_large[7][8] = R[2][ 1]; Rinv_large[7][9] = 0; Rinv_large[7][10] = 0; Rinv_large[7][11] = 0; Rinv_large[8][0] = 0; Rinv_large[8][1] = 0; Rinv_large[8][2] = 0; Rinv_large[8][3] = 0; Rinv_large[8][4] = 0; Rinv_large[8][5] = 0; Rinv_large[8][6] = R[0][ 2]; Rinv_large[8][7] = R[1][ 2]; Rinv_large[8][8] = R[2][ 2]; Rinv_large[8][9] = 0; Rinv_large[8][10] = 0; Rinv_large[8][11] = 0; Rinv_large[9][0] = 0; Rinv_large[9][1] = 0; Rinv_large[9][2] = 0; Rinv_large[9][3] = 0; Rinv_large[9][4] = 0; Rinv_large[9][5] = 0; Rinv_large[9][6] = 0; Rinv_large[9][7] = 0; Rinv_large[9][8] = 0; Rinv_large[9][9] = R[0][ 0]; Rinv_large[9][10] = R[1][ 0]; Rinv_large[9][11] = R[2][ 0]; Rinv_large[10][0] = 0; Rinv_large[10][1] = 0; Rinv_large[10][2] = 0; Rinv_large[10][3] = 0; Rinv_large[10][4] = 0; Rinv_large[10][5] = 0; Rinv_large[10][6] = 0; Rinv_large[10][7] = 0; Rinv_large[10][8] = 0; Rinv_large[10][9] = R[0][ 1]; Rinv_large[10][10] = R[1][ 1]; Rinv_large[10][11] = R[2][ 1]; Rinv_large[11][0] = 0; Rinv_large[11][1] = 0; Rinv_large[11][2] = 0; Rinv_large[11][3] = 0; Rinv_large[11][4] = 0; Rinv_large[11][5] = 0; Rinv_large[11][6] = 0; Rinv_large[11][7] = 0; Rinv_large[11][8] = 0; Rinv_large[11][9] = R[0][ 2]; Rinv_large[11][10] = R[1][ 2]; Rinv_large[11][11] = R[2][ 2];


	//First find R*BTDB, or R*K
	float RBTDB[12][12];
	for (int row = 0; row < 12; row++){
		for (int col = 0; col < 12; col++){
			float sum = 0.0;
			for (int j = 0; j < 12; j++){
				sum += R_large[row][j] * BTDB[j][col];
			}
			RBTDB[row][col] = sum;
		}
	}

	//Then find RBTDB*R.T
	//	
	//in_element->local_K[position_counter] = BTDB[row][col];
	//position_counter++;
	//Perform corotation
	int position_counter = 0;
	float K_corotation[12][12];
	for (int row = 0; row < 12; row++){
		for (int col = 0; col < 12; col++){
			float sum = 0.0;
			for (int j = 0; j < 12; j++){
				sum += RBTDB[row][j]*Rinv_large[j][col];
			}
			K_corotation[row][col] = sum;
			in_element->local_K[position_counter] = K_corotation[row][col];
			position_counter++;
		}
	}

	//current vector
	float x0[12] = {
		p1.x,p1.y,p1.z,
		p2.x,p2.y,p2.z,
		p3.x,p3.y,p3.z,
		p4.x,p4.y,p4.z
	};
	
	for (int row = 0; row < 12; row++){
		float sum = 0.0;
		for (int j = 0; j < 12; j++){
			sum += RBTDB[row][j] * x0[j];
		}
		in_element->local_RKx[row] = sum;
	}



	in_element->local_M[0] = volume*6.0*rho / 24.0;
	in_element->local_M[1] = 0;
	in_element->local_M[2] = 0;
	in_element->local_M[3] = 0;
	in_element->local_M[4] = 0;
	in_element->local_M[5] = 0;
	in_element->local_M[6] = 0;
	in_element->local_M[7] = 0;
	in_element->local_M[8] = 0;
	in_element->local_M[9] = 0;
	in_element->local_M[10] = 0;
	in_element->local_M[11] = 0;
	in_element->local_M[12] = 0;
	in_element->local_M[13] = volume*6.0*rho / 24.0;
	in_element->local_M[14] = 0;
	in_element->local_M[15] = 0;
	in_element->local_M[16] = 0;
	in_element->local_M[17] = 0;
	in_element->local_M[18] = 0;
	in_element->local_M[19] = 0;
	in_element->local_M[20] = 0;
	in_element->local_M[21] = 0;
	in_element->local_M[22] = 0;
	in_element->local_M[23] = 0;
	in_element->local_M[24] = 0;
	in_element->local_M[25] = 0;
	in_element->local_M[26] = volume*6.0*rho / 24.0;
	in_element->local_M[27] = 0;
	in_element->local_M[28] = 0;
	in_element->local_M[29] = 0;
	in_element->local_M[30] = 0;
	in_element->local_M[31] = 0;
	in_element->local_M[32] = 0;
	in_element->local_M[33] = 0;
	in_element->local_M[34] = 0;
	in_element->local_M[35] = 0;
	in_element->local_M[36] = 0;
	in_element->local_M[37] = 0;
	in_element->local_M[38] = 0;
	in_element->local_M[39] = volume*6.0*rho / 24.0;
	in_element->local_M[40] = 0;
	in_element->local_M[41] = 0;
	in_element->local_M[42] = 0;
	in_element->local_M[43] = 0;
	in_element->local_M[44] = 0;
	in_element->local_M[45] = 0;
	in_element->local_M[46] = 0;
	in_element->local_M[47] = 0;
	in_element->local_M[48] = 0;
	in_element->local_M[49] = 0;
	in_element->local_M[50] = 0;
	in_element->local_M[51] = 0;
	in_element->local_M[52] = volume*6.0*rho / 24.0;
	in_element->local_M[53] = 0;
	in_element->local_M[54] = 0;
	in_element->local_M[55] = 0;
	in_element->local_M[56] = 0;
	in_element->local_M[57] = 0;
	in_element->local_M[58] = 0;
	in_element->local_M[59] = 0;
	in_element->local_M[60] = 0;
	in_element->local_M[61] = 0;
	in_element->local_M[62] = 0;
	in_element->local_M[63] = 0;
	in_element->local_M[64] = 0;
	in_element->local_M[65] = volume*6.0*rho / 24.0;
	in_element->local_M[66] = 0;
	in_element->local_M[67] = 0;
	in_element->local_M[68] = 0;
	in_element->local_M[69] = 0;
	in_element->local_M[70] = 0;
	in_element->local_M[71] = 0;
	in_element->local_M[72] = 0;
	in_element->local_M[73] = 0;
	in_element->local_M[74] = 0;
	in_element->local_M[75] = 0;
	in_element->local_M[76] = 0;
	in_element->local_M[77] = 0;
	in_element->local_M[78] = volume*6.0*rho / 24.0;
	in_element->local_M[79] = 0;
	in_element->local_M[80] = 0;
	in_element->local_M[81] = 0;
	in_element->local_M[82] = 0;
	in_element->local_M[83] = 0;
	in_element->local_M[84] = 0;
	in_element->local_M[85] = 0;
	in_element->local_M[86] = 0;
	in_element->local_M[87] = 0;
	in_element->local_M[88] = 0;
	in_element->local_M[89] = 0;
	in_element->local_M[90] = 0;
	in_element->local_M[91] = volume*6.0*rho / 24.0;
	in_element->local_M[92] = 0;
	in_element->local_M[93] = 0;
	in_element->local_M[94] = 0;
	in_element->local_M[95] = 0;
	in_element->local_M[96] = 0;
	in_element->local_M[97] = 0;
	in_element->local_M[98] = 0;
	in_element->local_M[99] = 0;
	in_element->local_M[100] = 0;
	in_element->local_M[101] = 0;
	in_element->local_M[102] = 0;
	in_element->local_M[103] = 0;
	in_element->local_M[104] = volume*6.0*rho / 24.0;
	in_element->local_M[105] = 0;
	in_element->local_M[106] = 0;
	in_element->local_M[107] = 0;
	in_element->local_M[108] = 0;
	in_element->local_M[109] = 0;
	in_element->local_M[110] = 0;
	in_element->local_M[111] = 0;
	in_element->local_M[112] = 0;
	in_element->local_M[113] = 0;
	in_element->local_M[114] = 0;
	in_element->local_M[115] = 0;
	in_element->local_M[116] = 0;
	in_element->local_M[117] = volume*6.0*rho / 24.0;
	in_element->local_M[118] = 0;
	in_element->local_M[119] = 0;
	in_element->local_M[120] = 0;
	in_element->local_M[121] = 0;
	in_element->local_M[122] = 0;
	in_element->local_M[123] = 0;
	in_element->local_M[124] = 0;
	in_element->local_M[125] = 0;
	in_element->local_M[126] = 0;
	in_element->local_M[127] = 0;
	in_element->local_M[128] = 0;
	in_element->local_M[129] = 0;
	in_element->local_M[130] = volume*6.0*rho / 24.0;
	in_element->local_M[131] = 0;
	in_element->local_M[132] = 0;
	in_element->local_M[133] = 0;
	in_element->local_M[134] = 0;
	in_element->local_M[135] = 0;
	in_element->local_M[136] = 0;
	in_element->local_M[137] = 0;
	in_element->local_M[138] = 0;
	in_element->local_M[139] = 0;
	in_element->local_M[140] = 0;
	in_element->local_M[141] = 0;
	in_element->local_M[142] = 0;
	in_element->local_M[143] = volume*6.0*rho / 24.0;


	float b1 = 0.0;
	float b2 = (-9.81 *rho);// *(det_J / 6) / 4.0;
	float b3 = 0.0;

	in_element->f_body[0] = b1*volume / 12.0;
	in_element->f_body[1] = b2*volume / 12.0;
	in_element->f_body[2] = b3*volume / 12.0;
	in_element->f_body[3] = b1*volume / 12.0;
	in_element->f_body[4] = b2*volume / 12.0;
	in_element->f_body[5] = b3*volume / 12.0;
	in_element->f_body[6] = b1*volume / 12.0;
	in_element->f_body[7] = b2*volume / 12.0;
	in_element->f_body[8] = b3*volume / 12.0;
	in_element->f_body[9] = b1*volume / 12.0;
	in_element->f_body[10] = b2*volume / 12.0;
	in_element->f_body[11] = b3*volume / 12.0;
}



__global__ void gpu_make_K_corotational(AFEM::element *in_vec, AFEM::position_3D *in_pos, AFEM::position_3D *original_pos, int numElem, int numNodes, float *K_d, float *M_d, float *f_d,float *RKx_matrix_d)
{

	int x = threadIdx.x + blockIdx.x * blockDim.x;
	if (x < numElem){
		find_Jacobian_localK_localM_corotational(&in_vec[x], in_pos,original_pos);
		//find_localM(&in_vec[x]);
		//K_d[x] = (in_vec[x]).local_K[0];
		int DOF[12];
		int counter = 0;
		//The two loops are responsible for finding the DOF (or q_i) for each element
		for (int npe = 0; npe < 4; npe++){
			//dummy_node = nodesInElem[nodesinelemX(npe, offset, 4)]; // The row of the matrix we looking at will be k_th element and npe (nodes per element) 	
			for (int dof = 0; dof < 3; dof++){

				DOF[counter] = in_vec[x].position_info[npe].displacement_index[dof];
				counter++;

			}
		}

		for (int c = 0; c < 12; c++){
			for (int r = 0; r < 12; r++){

				//d_A_dense[IDX2C(DOF[c], DOF[r], 3000)] = d_A_dense[IDX2C(DOF[c], DOF[r], 3000)] + E_vector[offset * 144 + c*12+r];
				atomicAdda(&(K_d[IDX2C(DOF[c], DOF[r], 3 * (numNodes))]), in_vec[x].local_K[c * 12 + r]);


				//atomicAdda(&(M_d[IDX2C(DOF[c], DOF[r], 3 * (numNodes))]), in_vec[x].local_M[c * 12 + r]);
				
				atomicAdda(&(M_d[IDX2C(DOF[c], DOF[r], 3 * (numNodes))]), in_vec[x].local_M[c * 12 + r]);
				//atomicAdda(&(RK_matrix_d[IDX2C(DOF[c], DOF[r], 3 * (numNodes))]), in_vec[x].local_RK[c * 12 + r]);

				//atomicAdda(&(M_d[IDX2C(DOF[c], DOF[r], 3 * (numNodes))]), in_vec[x].local_M[c * 12 + r]);
				//IDX2C(DOF[c], DOF[r], 3000)
				//K[IDX2C(DOF[r], DOF[c], numP*dim)] = K[IDX2C(DOF[r], DOF[c], numP*dim)] + E[k][r][c];

			}
			//atomicAdda(&(f_d[DOF[c]]), in_vec[x].f_body[c]);
			/*if (x == 800){
				atomicAdda(&(f_d[DOF[c]]), in_vec[x].f_body[c]);
				}*/
			atomicAdda(&(f_d[DOF[c]]), in_vec[x].f_body[c]);
			atomicAdda(&(RKx_matrix_d[DOF[c]]), in_vec[x].local_RKx[c]);
		}
		//printf("hi");
	}

}








//find the vector value of dt*f_ext - dt*K*(u(t)-u(0))+dt*dt K*u_dot(t)
//In the code I have set:
//a= dt*f_ext
//b= dt*K*(x(t)-x(0))
//c= dt*dt K*u_dot(t)
//so RHS = a-b+c
//And LHS: = M-dt*dt* K
__global__ void find_A_b_dynamic_corotational(float *K_in, float *dx_in, float *x_zero, float *x_current,float *u_dot, float *f_ext, float *RHS, float *M_in, float *LHS, float *RKx,int num_nodes, float dt, int dim){
	int x = threadIdx.x + blockIdx.x *blockDim.x;

	if (x < num_nodes*dim){
		float a, b, c;
		float b1, b2;
		b = c = 0;
		b1 = b2 = 0;
		a = f_ext[x];
		for (int i = 0; i < num_nodes*dim; i++){
			/*if (i == x){
				M_in[IDX2C(i, x, 3 * (num_nodes))] =
				}*/
			//+ RK[IDX2C(x,i, 3 * (num_nodes))] * x_zero[i];
			b2 = b2 + K_in[IDX2C(i, x, 3 * (num_nodes))] * x_current[i];

			//= b1 - b2;
			//b = b + K_in[IDX2C(i, x, 3 * (num_nodes))] * dx_in[i];
			//c = c + K_in[IDX2C(i, x, 3 * (num_nodes))] * u_dot[i]; 
			//origional
			//float c1 = dt*rm*M_in[IDX2C(i, x, 3 * num_nodes)] + (dt *rk-dt*dt)*K_in[IDX2C(i, x, 3 * num_nodes)];
			float c1 = M_in[IDX2C(i, x, 3 * num_nodes)];
			c += c1*u_dot[i];

			//Origional
			//LHS[IDX2C(i, x, 3 * (num_nodes))] = (1.0-dt*rm)*M_in[IDX2C(i, x, 3 * (num_nodes))] - (dt*rk+dt*dt)*K_in[IDX2C(i, x, 3 * (num_nodes))];
			LHS[IDX2C(i, x, 3 * (num_nodes))] = (1.0+1.0*dt)*M_in[IDX2C(i, x, 3 * (num_nodes))] + (dt*dt)*K_in[IDX2C(i, x, 3 * (num_nodes))];
			/*if (i == x){

			}
			else{
			LHS[IDX2C(i, x, 3 * (num_nodes))] = (dt*dt)*K_in[IDX2C(i, x, 3 * (num_nodes))];
			}*/

		}
		b = RKx[x] - b2;
		a = a*dt;
		b = b*dt;


		RHS[x] = a+b + c;
		//RHS[x] = f_ext[x];
	}
}





























#if 0

__global__ void vecAdd(double *a, double *b, double *c, int n)
{
	// Get our global thread ID
	int id = blockIdx.x*blockDim.x + threadIdx.x;

	// Make sure we do not go out of bounds
	if (id < n)
		c[id] = a[id] + b[id];
}

void hello(){
	// Size of vectors
	int n = 100000;

	// Host input vectors
	double *h_a;
	double *h_b;
	//Host output vector
	double *h_c;

	// Device input vectors
	double *d_a;
	double *d_b;
	//Device output vector
	double *d_c;

	// Size, in bytes, of each vector
	size_t bytes = n*sizeof(double);

	// Allocate memory for each vector on host
	h_a = (double*)malloc(bytes);
	h_b = (double*)malloc(bytes);
	h_c = (double*)malloc(bytes);

	// Allocate memory for each vector on GPU
	cudaMalloc(&d_a, bytes);
	cudaMalloc(&d_b, bytes);
	cudaMalloc(&d_c, bytes);

	int i;
	// Initialize vectors on host
	for (i = 0; i < n; i++) {
		h_a[i] = sin(i)*sin(i);
		h_b[i] = cos(i)*cos(i);
	}

	// Copy host vectors to device
	cudaMemcpy(d_a, h_a, bytes, cudaMemcpyHostToDevice);
	cudaMemcpy(d_b, h_b, bytes, cudaMemcpyHostToDevice);

	int blockSize, gridSize;

	// Number of threads in each thread block
	blockSize = 1024;

	// Number of thread blocks in grid
	gridSize = (int)ceil((float)n / blockSize);

	// Execute the kernel
	vecAdd << <gridSize, blockSize >> >(d_a, d_b, d_c, n);

	// Copy array back to host
	cudaMemcpy(h_c, d_c, bytes, cudaMemcpyDeviceToHost);

	// Sum up vector c and print result divided by n, this should equal 1 within error
	double sum = 0;
	for (i = 0; i < n; i++)
		sum += h_c[i];
	printf("final result: %f\n", sum / n);

	// Release device memory
	cudaFree(d_a);
	cudaFree(d_b);
	cudaFree(d_c);

	// Release host memory
	free(h_a);
	free(h_b);
	free(h_c);

}

//__global__ void make_K_cuda3d(double *E_vector, int *nodesInElem_device, double *x_vector, double *y_vector, double *z_vector, int *displaceInElem_device, float *d_A_dense, int *numnodes);//3D
//__global__ void make_K_cuda2d(double *K, int *nodesInElem, double *x_vector, double *y_vector, int *displaceInElem_device, float *d_A_dense, int numnodes, double thickness,double young_E,double nu,double alpha,double beta1,double beta2, double rho, double dt,double c_xi,int numE);//2D
//__global__ void make_global_K(void); 






#endif // 0

#include "AFEM_cuda.cuh"

#include <Eigen\Eigen>
#include <Eigen\IterativeLinearSolvers>


#define IDX2C(i,j,ld) (((j)*(ld))+( i )) 
void cuda_tools::cg_cpu(){




	/*
	If we are using the Eigen CPU solver
	*/

	float *LHS_host, *RHS_host;

	LHS_host = (float *)malloc(sizeof(*LHS_host) * 3 * Nnodes * 3 * Nnodes);
	RHS_host = (float *)malloc(sizeof(*RHS_host) * 3 * Nnodes);


	cudaMemcpy(LHS_host, LHS, sizeof(*LHS_host) * 3 * Nnodes * 3 * Nnodes, cudaMemcpyDeviceToHost);
	cudaMemcpy(RHS_host, RHS, sizeof(*RHS_host) * 3 * Nnodes, cudaMemcpyDeviceToHost);

	int n = 3 * Nnodes;
	Eigen::VectorXd x(n), b(n);
	Eigen::SparseMatrix<double> A(n, n);
	//Eigen::MatrixXd A(n, n);
	// fill A and b
	for (int i = 0; i < 3 * Nnodes; i++){

		for (int j = 0; j < 3 * Nnodes; j++){
			A.insert(i, j) = LHS_host[IDX2C(i, j, 3 * (Nnodes))];
		}
		b(i) = RHS_host[i];
	}



	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
	cg.compute(A);
	x = cg.solve(b);
	std::cout << cg.error() << std::endl;
	float *temp = (float *)malloc(sizeof(*RHS_host) * 3 * Nnodes);
	for (int i = 0; i < 3 * Nnodes; i++){
		temp[i] = x(i);
	}
	
	cudaMemcpy(RHS, temp, sizeof(*RHS) * 3 * Nnodes, cudaMemcpyHostToDevice);

	update_geometry(RHS);

		
	//std::cout << "#iterations:     " << cg.iterations() << std::endl;
	//std::cout << "estimated error: " << cg.error() << std::endl;
	//// update b, and solve again
	//x = cg.solve(b);
	free(LHS_host);
	free(RHS_host);
	free(temp);
	//delete temp;



}
















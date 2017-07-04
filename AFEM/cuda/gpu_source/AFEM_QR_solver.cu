

#include <stdio.h>
#include <math.h>
#include <iostream>
#include "AFEM_cuda.cuh"
#include "Utilities.cuh"
#include <fstream>
#include <ctime>

#define IDX2C(i,j,ld) (((j)*(ld))+( i )) 
void printMatrix(int m, int n, const double*A, int lda, const char* name) { for (int row = 0; row < m; row++){ for (int col = 0; col < n; col++){ double Areg = A[row + col*lda]; printf("%s(%d,%d) = %f\n", name, row + 1, col + 1, Areg); } } }

void cuda_tools::initialize_cholesky_variables(int numNodes, int numElem, int dim){
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

void cuda_tools::cholesky()
{
	
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

#if 0
	float *LHSOUT = (float *)malloc(Ncols * Ncols*sizeof(float));
	cudaMemcpy(LHSOUT, LHS, Ncols * Ncols*sizeof(float), cudaMemcpyDeviceToHost);


	std::ofstream output;
	output.open("LHS.txt");

	for (int i = 0; i < Ncols; i++){
		for (int j = 0; j < Ncols; j++){
			//std::cout << LHSOUT[IDX2C(j, i, Ncols)] << " ";
			output << LHSOUT[IDX2C(j, i, Ncols)] << " ";
		}
		//std::cout << std::endl;
		output << "\n";
	}

	output.close();
	free(LHSOUT);


	float *RHSOUT = (float *)malloc( Ncols*sizeof(float));
	cudaMemcpy(RHSOUT, RHS,  Ncols*sizeof(float), cudaMemcpyDeviceToHost);


	std::ofstream RHSOUT_;
	RHSOUT_.open("RHS.txt");

	for (int i = 0; i < Ncols; i++){
		
			//std::cout << RHSOUT[IDX2C(j, i, Ncols)] << " ";
			RHSOUT_ << RHSOUT[i] << " ";
		
		//std::cout << std::endl;
		RHSOUT_ << std::endl;
	}

	RHSOUT_.close();
	free(RHSOUT);


#endif // 0

	

	// --- Device side number of nonzero elements per row

	cusparseSafeCall(cusparseSnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, LHS, lda, d_nnzPerVector, &nnz));
	// --- Host side number of nonzero elements per row


	gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));

	/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
	for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
	printf("\n");*/

	// --- Device side dense matrix
	gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
	gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));

	
	cusparseSafeCall(cusparseSdense2csr(handle, Nrows, Ncols, descrA, LHS, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));
	// --- Host side dense matrix

	float *h_A = (float *)malloc(nnz * sizeof(*h_A));
	int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
	int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
	gpuErrchk(cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost));
	gpuErrchk(cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost));



	
//	std::cout << nnz << std::endl;

	/*printf("\nOriginal matrix in CSR format\n\n");
	for (int i = 0; i < 10; ++i) printf("A[%i] = %.0f ", i, h_A[i]); printf("\n");

	printf("\n");
	for (int i = 0; i < (10 + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");

	for (int i = 0; i < 10; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);
	*/
	// --- Allocating and defining dense host and device data vectors

	//float *h_x = (float *)malloc(Nrows * sizeof(float));
	///*h_x[0] = 100.0;  h_x[1] = 200.0; h_x[2] = 400.0; h_x[3] = 500.0;*/
	//for (int i = 0; i < N; i++){
	//	h_x[i] = 0.00001;
	//}


	float *d_x;        gpuErrchk(cudaMalloc(&d_x, Nrows * sizeof(float)));
	gpuErrchk(cudaMemcpy(d_x, RHS, Nrows * sizeof(float), cudaMemcpyDeviceToDevice));



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


	cusparseSafeCall(cusparseScsr2dense(handle, Nrows, Ncols, descrA, d_A, d_A_RowIndices, d_A_ColIndices, LHS, Nrows));


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
	
	float *d_y;        gpuErrchk(cudaMalloc(&d_y, Ncols * sizeof(float)));

	cusparseSafeCall(cusparseScsrsv2_solve(handle, CUSPARSE_OPERATION_TRANSPOSE, N, nnz, &alpha, descr_L, d_A, d_A_RowIndices, d_A_ColIndices, info_Lt, d_z, d_y, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer));



	//cudaMemcpy(h_x, d_y, N * sizeof(float), cudaMemcpyDeviceToHost);
	///*for (int k = 0; k<20; k++) printf("dx[%i] = %f\n", k, h_x[k]);
	//for (int k = 0; k<20; k++) printf("xs[%i] = %f\n", k, x[k]);*/
	//

	//std::cout << std::endl;
	//

	

	//float *sln_ptr = (float *)malloc(Ncols*sizeof(float));
	//cudaMemcpy(sln_ptr, d_y, Ncols*sizeof(float), cudaMemcpyDeviceToHost);


	//std::ofstream outputSLN;
	//outputSLN.open("SLN.txt");

	//for (int i = 0; i < Ncols; i++){

	//	outputSLN << sln_ptr[i];

	//	//std::cout << std::endl;
	//	outputSLN << std::endl;
	//}

	//outputSLN.close();
	//free(sln_ptr); 


	//outputSLN.close();


	update_geometry(d_y);



	cudaFree(d_A);
	cudaFree(d_A_ColIndices);
	cudaFree(pBuffer);

	cudaFree(d_z);
	cudaFree(d_y);
	//free(h_y);
	//free(h_x);


#if 0

	free(h_A);
	free(h_A_RowIndices);
	free(h_A_ColIndices);
	free(h_x);
	free(h_y);
	cudaFree(d_x);
	cudaFree(pBuffer);
	cudaFree(d_z);
	cudaFree(d_y);


	for (int i = 0; i < numNodes; i++) {
		x[i] = x[i] + h_x[i * dim];
		y[i] = y[i] + h_x[i * dim + 1];
		if (dim == 3){
			z[i] = z[i] + h_x[i * dim + 2];
		}

	}

	
	duration_K = (std::clock() - start_K) / (double)CLOCKS_PER_SEC;
	//std::cout << " change status : " << changeNode << std::endl;

	//std::cout << "FPS time: " <<1/duration_K << std::endl;

	//std::cout << "Duration: " << duration_K << std::endl;
	return 0;
#endif // 0

}


void cuda_tools::cg(void){


		/*M = N = 10480076;
		nz = (N - 2) * 3 + 4;
		I = (int *)malloc(sizeof(int)*(N + 1));
		J = (int *)malloc(sizeof(int)*nz);
		val = (float *)malloc(sizeof(float)*nz);


		x = (float *)malloc(sizeof(float)*N);
		rhs = (float *)malloc(sizeof(float)*N);

		for (int i = 0; i < N; i++)
		{
			rhs[i] = 1.0;
			x[i] = 0.0;
		}*/
	N = Ncols;
	/*x = (float *)malloc(sizeof(float)*N);
	rhs = (float *)malloc(sizeof(float)*N);

	for (int i = 0; i < N; i++)
	{
		rhs[i] = 1.0;
		x[i] = 0.0;
	}*/
	
	x = (float *)malloc(sizeof(float)*N);
	for (int i = 0; i < N; i++)
	{
		
		x[i] =0.0;
	}
		/* Get handle to the CUBLAS context */
		cublasHandle_t cublasHandle = 0;
		cublasStatus_t cublasStatus;
		cublasStatus = cublasCreate(&cublasHandle);



		/* Get handle to the CUSPARSE context */
		cusparseHandle_t cusparseHandle = 0;
		cusparseStatus_t cusparseStatus;
		cusparseStatus = cusparseCreate(&cusparseHandle);


		cusparseMatDescr_t descr = 0;
		cusparseStatus = cusparseCreateMatDescr(&descr);

		cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
		cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);

		cusparseSafeCall(cusparseSnnz(cusparseHandle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descr, LHS, lda, d_nnzPerVector, &nnz));
		// --- Host side number of nonzero elements per row


		gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));

		/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
		for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
		printf("\n");*/

		// --- Device side dense matrix
		gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
		gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));
	
		//cudaMalloc((void **)&d_Ax, N*sizeof(float));
		cudaMalloc(&d_A_RowIndices, (Nrows + 1) * sizeof(*d_A_RowIndices));
		cusparseSafeCall(cusparseSdense2csr(cusparseHandle, Nrows, Ncols, descr, LHS, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));

		//float *LHSOUT = (float *)malloc(Ncols *sizeof(float));
		//cudaMemcpy(LHSOUT, RHS, Ncols *sizeof(float), cudaMemcpyDeviceToHost);


		//std::ofstream output;
		//output.open("LHS.txt");

		//for (int i = 0; i < Ncols; i++){
		//	
		//		//std::cout << LHSOUT[IDX2C(j, i, Ncols)] << " ";
		//		output << LHSOUT[i] << " ";
		//	
		//	//std::cout << std::endl;
		//	output << "\n";
		//}

		//output.close();
		//free(LHSOUT);

#if 0

		float *LHSOUT = (float *)malloc(Ncols * Ncols*sizeof(float));
		cudaMemcpy(LHSOUT, LHS, Ncols * Ncols*sizeof(float), cudaMemcpyDeviceToHost);


		std::ofstream output;
		output.open("LHS.txt");

		for (int i = 0; i < Ncols; i++){
			for (int j = 0; j < Ncols; j++){
				//std::cout << LHSOUT[IDX2C(j, i, Ncols)] << " ";
				output << LHSOUT[IDX2C(j, i, Ncols)] << " ";
			}
			//std::cout << std::endl;
			output << "\n";
		}

		output.close();
		free(LHSOUT);
#endif // 0




	
		(cudaMalloc((void **)&d_x, N*sizeof(float)));
		(cudaMalloc((void **)&d_r, N*sizeof(float)));
		(cudaMalloc((void **)&d_p, N*sizeof(float)));
		(cudaMalloc((void **)&d_Ax, N*sizeof(float)));

		alpha = 1.0;
		alpham1 = -1.0;
		beta = 0.0;
		r0 = 0.0;
		cudaMemcpy(d_x, RHS, N*sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(d_r, RHS, N*sizeof(float), cudaMemcpyDeviceToDevice);
	
		cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, Ncols, Ncols, nnz, &alpha, descr, d_A, d_A_RowIndices, d_A_ColIndices, d_x, &beta, d_Ax);

		cublasSaxpy(cublasHandle, N, &alpham1, d_Ax, 1, d_r, 1);
		cublasStatus = cublasSdot(cublasHandle, N, d_r, 1, d_r, 1, &r1);
		//cudaMalloc((void **)&d_p, N*sizeof(float));
		k = 1;
		double start_time = std::clock();
		while (r1 >tol*tol && k <= max_iter)
		{
			if (k > 1)
			{
				b = r1 / r0;
				cublasStatus = cublasSscal(cublasHandle, N, &b, d_p, 1);
				cublasStatus = cublasSaxpy(cublasHandle, N, &alpha, d_r, 1, d_p, 1);
				//stationary_BC_f(d_p);
			}
			else
			{
				cublasStatus = cublasScopy(cublasHandle, N, d_r, 1, d_p, 1);
			}

			cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, N, nnz, &alpha, descr, d_A, d_A_RowIndices, d_A_ColIndices, d_p, &beta, d_Ax);
			//stationary_BC_f(d_p);
			cublasStatus = cublasSdot(cublasHandle, N, d_p, 1, d_Ax, 1, &dot);
			
			a = r1 / dot;

			
			cublasStatus = cublasSaxpy(cublasHandle, N, &a, d_p, 1, d_x, 1);
			//stationary_BC_f(d_x);
		//	stationary_BC_f(d_Ax);

			/*stationary_BC_f(d_r);
						*/
						
						//stationary_BC_f(d_Ax);
						//stationary_BC_f(d_r);
			na = -a;
			cublasStatus = cublasSaxpy(cublasHandle, N, &na, d_Ax, 1, d_r, 1);
		
			
			r0 = r1;
			cublasStatus = cublasSdot(cublasHandle, N, d_r, 1, d_r, 1, &r1);
			
		//	cudaThreadSynchronize();
			//printf("iteration = %3d, residual = %e\n", k, sqrt(r1));
			k++;
		}
		std::cout << "iteration time"<< (std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
		//cudaMemcpy(x, d_x, N*sizeof(float), cudaMemcpyDeviceToHost);

		/*float rsum, diff, err = 0.0;

		for (int i = 0; i < N; i++)
		{
			rsum = 0.0;

			for (int j = I[i]; j < I[i + 1]; j++)
			{
				rsum += val[j] * x[J[j]];
			}

			diff = fabs(rsum - rhs[i]);

			if (diff > err)
			{
				err = diff;
			}
		}*/
		update_geometry(d_x);
		cusparseDestroy(cusparseHandle);
		cublasDestroy(cublasHandle);
		
		/*free(I);
		free(J);*/
		free(val);
		//free(x);
		//free(rhs);
		cudaFree(d_col);
		cudaFree(d_row);
		cudaFree(d_val);
		cudaFree(d_x);
		cudaFree(d_r);
		cudaFree(d_p);
		cudaFree(d_Ax);
	}






	void cuda_tools::cg_precond(void){


		/*M = N = 10480076;
		nz = (N - 2) * 3 + 4;
		I = (int *)malloc(sizeof(int)*(N + 1));
		J = (int *)malloc(sizeof(int)*nz);
		val = (float *)malloc(sizeof(float)*nz);


		x = (float *)malloc(sizeof(float)*N);
		rhs = (float *)malloc(sizeof(float)*N);

		for (int i = 0; i < N; i++)
		{
		rhs[i] = 1.0;
		x[i] = 0.0;
		}*/
		N = Ncols;
		/*x = (float *)malloc(sizeof(float)*N);
		rhs = (float *)malloc(sizeof(float)*N);

		for (int i = 0; i < N; i++)
		{
		rhs[i] = 1.0;
		x[i] = 0.0;
		}*/

		x = (float *)malloc(sizeof(float)*N);
		for (int i = 0; i < N; i++)
		{

			x[i] = 0.0;
		}
		/* Get handle to the CUBLAS context */
		cublasHandle_t cublasHandle = 0;
		cublasStatus_t cublasStatus;
		cublasStatus = cublasCreate(&cublasHandle);

		
		/* Get handle to the CUSPARSE context */
		cusparseHandle_t cusparseHandle = 0;
		cusparseStatus_t cusparseStatus;
		cusparseStatus = cusparseCreate(&cusparseHandle);


		cusparseMatDescr_t descr = 0;
		cusparseStatus = cusparseCreateMatDescr(&descr);

		cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
		cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);

		cusparseSafeCall(cusparseSnnz(cusparseHandle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descr, LHS, lda, d_nnzPerVector, &nnz));
		// --- Host side number of nonzero elements per row


		gpuErrchk(cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost));

		/*printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
		for (int i = 0; i < 10; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
		printf("\n");*/

		// --- Device side dense matrix
		gpuErrchk(cudaMalloc(&d_A, nnz * sizeof(*d_A)));
		gpuErrchk(cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices)));

		//cudaMalloc((void **)&d_Ax, N*sizeof(float));
		cudaMalloc(&d_A_RowIndices, (Nrows + 1) * sizeof(*d_A_RowIndices));
		cusparseSafeCall(cusparseSdense2csr(cusparseHandle, Nrows, Ncols, descr, LHS, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices));

		//float *LHSOUT = (float *)malloc(Ncols *sizeof(float));
		//cudaMemcpy(LHSOUT, RHS, Ncols *sizeof(float), cudaMemcpyDeviceToHost);


		//std::ofstream output;
		//output.open("LHS.txt");

		//for (int i = 0; i < Ncols; i++){
		//	
		//		//std::cout << LHSOUT[IDX2C(j, i, Ncols)] << " ";
		//		output << LHSOUT[i] << " ";
		//	
		//	//std::cout << std::endl;
		//	output << "\n";
		//}

		//output.close();
		//free(LHSOUT);
		float *valsILU0;
		float *d_valsILU0;
		int nzILU0 = 2 * N - 1;
		valsILU0 = (float *)malloc(nnz*sizeof(float));
		float *d_zm1, *d_zm2, *d_rm2;



		(cudaMalloc((void **)&d_valsILU0, nnz*sizeof(float)));
		(cudaMalloc((void **)&d_zm1, (N)*sizeof(float)));
		(cudaMalloc((void **)&d_zm2, (N)*sizeof(float)));
		(cudaMalloc((void **)&d_rm2, (N)*sizeof(float)));




		/* create the analysis info object for the A matrix */
		cusparseSolveAnalysisInfo_t infoA = 0;
		cusparseStatus = cusparseCreateSolveAnalysisInfo(&infoA);
		cusparseStatus = cusparseScsrsv_analysis(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE,	N, nnz, descr, d_A, d_A_RowIndices, d_A_ColIndices, infoA);
		cudaMemcpy(d_valsILU0, d_A, nnz*sizeof(float), cudaMemcpyDeviceToDevice);
		cusparseStatus = cusparseScsrilu0(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, descr, d_valsILU0, d_A_RowIndices, d_A_ColIndices, infoA);



		/* Create info objects for the ILU0 preconditioner */
		cusparseSolveAnalysisInfo_t info_u;
		cusparseCreateSolveAnalysisInfo(&info_u);

		cusparseMatDescr_t descrL = 0;
		cusparseStatus = cusparseCreateMatDescr(&descrL);
		cusparseSetMatType(descrL, CUSPARSE_MATRIX_TYPE_GENERAL);
		cusparseSetMatIndexBase(descrL, CUSPARSE_INDEX_BASE_ZERO);
		cusparseSetMatFillMode(descrL, CUSPARSE_FILL_MODE_LOWER);
		cusparseSetMatDiagType(descrL, CUSPARSE_DIAG_TYPE_UNIT);

		cusparseMatDescr_t descrU = 0;
		cusparseStatus = cusparseCreateMatDescr(&descrU);
		cusparseSetMatType(descrU, CUSPARSE_MATRIX_TYPE_GENERAL);
		cusparseSetMatIndexBase(descrU, CUSPARSE_INDEX_BASE_ZERO);
		cusparseSetMatFillMode(descrU, CUSPARSE_FILL_MODE_UPPER);
		cusparseSetMatDiagType(descrU, CUSPARSE_DIAG_TYPE_NON_UNIT);
		cusparseStatus = cusparseScsrsv_analysis(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descrU, d_A, d_A_RowIndices, d_A_ColIndices, info_u);


		(cudaMalloc((void **)&d_x, N*sizeof(float)));
		(cudaMalloc((void **)&d_r, N*sizeof(float)));
		(cudaMalloc((void **)&d_p, N*sizeof(float)));
		(cudaMalloc((void **)&d_Ax, N*sizeof(float)));
		float *d_y;
		(cudaMalloc((void **)&d_y, N*sizeof(float)));
		alpha = 1.0;
		alpham1 = -1.0;
		beta = 0.0;
		r0 = 0.;
		//cudaMemcpy(d_x, RHS, N*sizeof(float), cudaMemcpyDeviceToDevice);
		cudaMemcpy(d_r, RHS, N*sizeof(float), cudaMemcpyDeviceToDevice);

		cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, Ncols, Ncols, nnz, &alpha, descr, d_A, d_A_RowIndices, d_A_ColIndices, d_x, &beta, d_Ax);

		const float floatone = 1.0;
		const float floatzero = 1.0;
		cublasSaxpy(cublasHandle, N, &alpham1, d_Ax, 1, d_r, 1);
		cublasStatus = cublasSdot(cublasHandle, N, d_r, 1, d_r, 1, &r1);
		//cudaMalloc((void **)&d_p, N*sizeof(float));
		k = 0;
		double start_time = std::clock();
		float dot, numerator, denominator, nalpha;

		float *d_omega;
		(cudaMalloc((void **)&d_omega, N*sizeof(float)));
		while (r1 > tol*tol && k <= max_iter)
		{
			// Forward Solve, we can re-use infoA since the sparsity pattern of A matches that of L
			cusparseStatus = cusparseScsrsv_solve(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, &floatone, descrL,
				d_valsILU0, d_A_RowIndices, d_A_ColIndices, infoA, d_r, d_y);
			

			// Back Substitution
			cusparseStatus = cusparseScsrsv_solve(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, &floatone, descrU,
				d_valsILU0, d_A_RowIndices, d_A_ColIndices, info_u, d_y, d_zm1);
			

			k++;

			if (k == 1)
			{
				cublasScopy(cublasHandle, N, d_zm1, 1, d_p, 1);
			}
			else
			{
				cublasSdot(cublasHandle, N, d_r, 1, d_zm1, 1, &numerator);
				cublasSdot(cublasHandle, N, d_rm2, 1, d_zm2, 1, &denominator);
				beta = numerator / denominator;
				cublasSscal(cublasHandle, N, &beta, d_p, 1);
				cublasSaxpy(cublasHandle, N, &floatone, d_zm1, 1, d_p, 1);
			}

			cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, N, nzILU0, &floatone, descrU, d_val, d_A_RowIndices, d_A_ColIndices, d_p, &floatzero, d_omega);
			cublasSdot(cublasHandle, N, d_r, 1, d_zm1, 1, &numerator);
			cublasSdot(cublasHandle, N, d_p, 1, d_omega, 1, &denominator);
			alpha = numerator / denominator;
			cublasSaxpy(cublasHandle, N, &alpha, d_p, 1, d_x, 1);
			cublasScopy(cublasHandle, N, d_r, 1, d_rm2, 1);
			cublasScopy(cublasHandle, N, d_zm1, 1, d_zm2, 1);
			nalpha = -alpha;
			cublasSaxpy(cublasHandle, N, &nalpha, d_omega, 1, d_r, 1);
			cublasSdot(cublasHandle, N, d_r, 1, d_r, 1, &r1);
		}
		std::cout << (std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
		//cudaMemcpy(x, d_x, N*sizeof(float), cudaMemcpyDeviceToHost);

		/*float rsum, diff, err = 0.0;

		for (int i = 0; i < N; i++)
		{
		rsum = 0.0;

		for (int j = I[i]; j < I[i + 1]; j++)
		{
		rsum += val[j] * x[J[j]];
		}

		diff = fabs(rsum - rhs[i]);

		if (diff > err)
		{
		err = diff;
		}
		}*/
		update_geometry(d_x);
		//cusparseDestroy(cusparseHandle);
		//cublasDestroy(cublasHandle);
		///* Destroy parameters */
		//cusparseDestroySolveAnalysisInfo(infoA);
		//cusparseDestroySolveAnalysisInfo(info_u);

		//cusparseDestroyMatDescr(descr);
		/* Free device memory */
		
	//	free(val);
		//free(x);
		//free(rhs);
		free(valsILU0);
		
	
		cudaFree(d_y);
		cudaFree(d_r);
		cudaFree(d_p);
		cudaFree(d_omega);
		cudaFree(d_valsILU0);
		cudaFree(d_zm1);
		cudaFree(d_zm2);
		cudaFree(d_rm2);
		/*free(I);
		free(J);*/
		free(val);
		//free(x);
		//free(rhs);
		cudaFree(d_col);
		cudaFree(d_row);
		cudaFree(d_val);
		cudaFree(d_x);
		cudaFree(d_r);
		cudaFree(d_p);
		cudaFree(d_Ax);
	}

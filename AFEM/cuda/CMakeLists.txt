file(GLOB CUDA_SOURCE gpu_source/*.cu)

CUDA_ADD_LIBRARY(AFEM_cuda_lib ${CUDA_SOURCE})
target_link_libraries( AFEM_cuda_lib cudart cublas cusparse)
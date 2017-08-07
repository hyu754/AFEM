file(GLOB CUDA_SOURCE gpu_source/*.cu)
file(GLOB CUDA_HEADER gpu_include/*.cuh)
source_group("Headers" FILES ${CUDA_HEADER})
CUDA_ADD_LIBRARY(AFEM_cuda_lib ${CUDA_SOURCE}  ${CUDA_HEADER})


target_link_libraries( AFEM_cuda_lib cudart cublas cusparse)
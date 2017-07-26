file(GLOB SOURCE source/*.cpp)
file(GLOB HEADER include/*.h)
source_group("Headers" FILES ${HEADER})
ADD_LIBRARY(FEM_lib ${SOURCE}  ${HEADER})

target_link_libraries( FEM_lib cudart cublas cusparse)
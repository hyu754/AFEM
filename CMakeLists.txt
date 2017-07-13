cmake_minimum_required(VERSION 2.8)
project(AFEM)

#Additional libraries
include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/AFEM/include/)
include_directories(${CMAKE_SOURCE_DIR}/AFEM/cuda/gpu_include/)
include_directories(${CMAKE_SOURCE_DIR}/Geometry)
link_directories(${CMAKE_SOURCE_DIR}/lib)
#link_libraries()

#cuda
find_package(CUDA REQUIRED)
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include/)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib)
#link_libraries(${CUDA_LIBRARIES})

#add_executable(OpenCloth "OpenCloth/OpenCloth_CoRotated_Linear_FEM/main.cpp")

#include Eigen headers
include_directories(${CMAKE_SOURCE_DIR}/include/Eigen/)


#include AFEM source
file(GLOB SOURCES AFEM/source/*.cpp )
#list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-std=c++11;-O2;-DVERBOSE")


set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_20)
add_subdirectory(AFEM/cuda)
ADD_EXECUTABLE(AFEM ${SOURCES})


#target_link_libraries(OpenCloth freeglutd glew32d)
#target_link_libraries( AFEM cudart cublas cuda cusparse)
target_link_libraries( AFEM AFEM_cuda_lib cudart freeglutd glew32d)
#Adding dll to the debug (assuming debug) folder
if(1)
add_custom_target(dll_includes

  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/freeglutd.dll            ${CMAKE_BINARY_DIR}/Debug/
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/glew32d.dll          ${CMAKE_BINARY_DIR}/Debug/
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/msvcp120.dll            ${CMAKE_BINARY_DIR}/Debug/
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/msvcr120.dll          ${CMAKE_BINARY_DIR}/Debug/
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/Geometry/FEM_Nodes.txt          ${CMAKE_BINARY_DIR}/Debug/
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/Geometry/FEM_Nodes.txt          ${CMAKE_BINARY_DIR}/
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/Geometry/FEM_Elem.txt          ${CMAKE_BINARY_DIR}/Debug/
 COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/Geometry/FEM_Elem.txt          ${CMAKE_BINARY_DIR}/
 
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/Geometry/FEM_Stationary.txt          ${CMAKE_BINARY_DIR}/

)
#add_dependencies(OpenCloth  dll_includes)
add_dependencies(AFEM        dll_includes)
endif(1)





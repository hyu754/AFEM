cmake_minimum_required(VERSION 2.8)
project(AFEM)

#Additional libraries
include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/AFEM/FEM/include/)
include_directories(${CMAKE_SOURCE_DIR}/AFEM/cuda/gpu_include/)
include_directories(${CMAKE_SOURCE_DIR}/Geometry)
link_directories(${CMAKE_SOURCE_DIR}/lib)
#link_libraries()

#cuda
find_package(CUDA REQUIRED)
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include/)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib)
#link_libraries(${CUDA_LIBRARIES})

#vtk required for opencv
find_package( OpenCV REQUIRED )
set(VTK_DIR "D:/Downloads/extracted/VTK-6.3.0/build")
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/cv_wrappers/include/)
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(cv_wrappers/)
#Add precompile header so we use AFEM
target_compile_definitions(CV_CUSTOM_WRAPPER PRIVATE AFEM_USE)
#add_executable(OpenCloth "OpenCloth/OpenCloth_CoRotated_Linear_FEM/main.cpp")

#include Eigen headers
include_directories(${CMAKE_SOURCE_DIR}/include/Eigen/)


#list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-std=c++11;-O2;-DVERBOSE")


#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_50)
add_subdirectory(AFEM/cuda)
add_subdirectory(AFEM/FEM)
ADD_EXECUTABLE(FEM_TEST tests/test_fem.cpp)
ADD_EXECUTABLE(CV_TEST tests/test_cv.cpp)

#Experiments
ADD_EXECUTABLE(PHANTOM_EXPERIMENT experiment/experiment_abdominal_phantom.cpp)


#target_link_libraries(OpenCloth freeglutd glew32d)
#target_link_libraries( AFEM cudart cublas cuda cusparse)
target_link_libraries( FEM_TEST AFEM_cuda_lib FEM_lib cudart freeglutd glew32d)
target_link_libraries(CV_TEST ${OpenCV_LIBS} AFEM_cuda_lib FEM_lib CV_CUSTOM_WRAPPER ${VTK_LIBRARIES})
target_link_libraries(PHANTOM_EXPERIMENT ovrvision64d ${OpenCV_LIBS} AFEM_cuda_lib FEM_lib CV_CUSTOM_WRAPPER ${VTK_LIBRARIES})
target_compile_definitions(PHANTOM_EXPERIMENT PRIVATE AFEM_USE)
target_compile_definitions(CV_TEST PRIVATE AFEM_USE)
#Adding dll and other necessary information to the debug (assuming debug) folder
if(1)
file(GLOB dlls_extra ${CMAKE_SOURCE_DIR}/bin/*.DLL)
file(GLOB dlls_vtk ${VTK_DIR}/bin/debug/*.dll)
file(GLOB dlls_cv ${OpenCV_DIR}/bin/debug/*.dll)
file(GLOB FEM_files ${CMAKE_SOURCE_DIR}/Geometry/*.txt)
file(GLOB cam_prop ${CMAKE_SOURCE_DIR}/camera_info/*.xml)

add_custom_target(dll_includes
COMMAND ${CMAKE_COMMAND} -E copy ${dlls_extra}            ${CMAKE_BINARY_DIR}/Debug/
)

add_custom_target(fem_info  COMMAND ${CMAKE_COMMAND} -E copy ${FEM_files}          ${CMAKE_BINARY_DIR}/)
add_custom_target(cam_info 
	COMMAND ${CMAKE_COMMAND} -E copy ${cam_prop} 			${CMAKE_BINARY_DIR}/
	COMMAND ${CMAKE_COMMAND} -E copy ${cam_prop} 			${CMAKE_BINARY_DIR}/Debug/
)
add_custom_target(dll_cv_includes
COMMAND ${CMAKE_COMMAND} -E copy ${dlls_vtk}     ${CMAKE_BINARY_DIR}/Debug/
COMMAND ${CMAKE_COMMAND} -E copy ${dlls_cv}     ${CMAKE_BINARY_DIR}/Debug/

)
add_dependencies(FEM_TEST   dll_includes)
add_dependencies(CV_TEST dll_cv_includes fem_info)
add_dependencies(PHANTOM_EXPERIMENT dll_cv_includes fem_info cam_info)


endif(1)





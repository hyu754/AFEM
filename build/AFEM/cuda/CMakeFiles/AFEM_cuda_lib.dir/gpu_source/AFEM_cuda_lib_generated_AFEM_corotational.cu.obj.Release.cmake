#  James Bigler, NVIDIA Corp (nvidia.com - jbigler)
#
#  Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
#
#  This code is licensed under the MIT License.  See the FindCUDA.cmake script
#  for the text of the license.

# The MIT License
#
# License for the specific language governing rights and limitations under
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.


##########################################################################
# This file runs the nvcc commands to produce the desired output file along with
# the dependency file needed by CMake to compute dependencies.  In addition the
# file checks the output of each command and if the command fails it deletes the
# output files.

# Input variables
#
# verbose:BOOL=<>          OFF: Be as quiet as possible (default)
#                          ON : Describe each step
#
# build_configuration:STRING=<> Typically one of Debug, MinSizeRel, Release, or
#                               RelWithDebInfo, but it should match one of the
#                               entries in CUDA_HOST_FLAGS. This is the build
#                               configuration used when compiling the code.  If
#                               blank or unspecified Debug is assumed as this is
#                               what CMake does.
#
# generated_file:STRING=<> File to generate.  This argument must be passed in.
#
# generated_cubin_file:STRING=<> File to generate.  This argument must be passed
#                                                   in if build_cubin is true.

if(NOT generated_file)
  message(FATAL_ERROR "You must specify generated_file on the command line")
endif()

# Set these up as variables to make reading the generated file easier
set(CMAKE_COMMAND "D:/Program Files/CMake/bin/cmake.exe") # path
set(source_file "D:/GitHub/AFEM/AFEM/cuda/gpu_source/AFEM_corotational.cu") # path
set(NVCC_generated_dependency_file "D:/GitHub/AFEM/build/AFEM/cuda/CMakeFiles/AFEM_cuda_lib.dir/gpu_source/AFEM_cuda_lib_generated_AFEM_corotational.cu.obj.NVCC-depend") # path
set(cmake_dependency_file "D:/GitHub/AFEM/build/AFEM/cuda/CMakeFiles/AFEM_cuda_lib.dir/gpu_source/AFEM_cuda_lib_generated_AFEM_corotational.cu.obj.depend") # path
set(CUDA_make2cmake "D:/Program Files/CMake/share/cmake-3.7/Modules/FindCUDA/make2cmake.cmake") # path
set(CUDA_parse_cubin "D:/Program Files/CMake/share/cmake-3.7/Modules/FindCUDA/parse_cubin.cmake") # path
set(build_cubin OFF) # bool
set(CUDA_HOST_COMPILER "$(VCInstallDir)bin") # path
# We won't actually use these variables for now, but we need to set this, in
# order to force this file to be run again if it changes.
set(generated_file_path "D:/GitHub/AFEM/build/AFEM/cuda/CMakeFiles/AFEM_cuda_lib.dir/gpu_source/$(Configuration)") # path
set(generated_file_internal "D:/GitHub/AFEM/build/AFEM/cuda/CMakeFiles/AFEM_cuda_lib.dir/gpu_source/$(Configuration)/AFEM_cuda_lib_generated_AFEM_corotational.cu.obj") # path
set(generated_cubin_file_internal "D:/GitHub/AFEM/build/AFEM/cuda/CMakeFiles/AFEM_cuda_lib.dir/gpu_source/$(Configuration)/AFEM_cuda_lib_generated_AFEM_corotational.cu.obj.cubin.txt") # path

set(CUDA_NVCC_EXECUTABLE "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/bin/nvcc.exe") # path
set(CUDA_NVCC_FLAGS  ;; ) # list
# Build specific configuration flags
set(CUDA_NVCC_FLAGS_DEBUG  ; )
set(CUDA_NVCC_FLAGS_RELEASE  ; )
set(CUDA_NVCC_FLAGS_MINSIZEREL  ; )
set(CUDA_NVCC_FLAGS_RELWITHDEBINFO  ; )
set(nvcc_flags -m64) # list
set(CUDA_NVCC_INCLUDE_DIRS "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include;D:/GitHub/AFEM/include;D:/GitHub/AFEM/AFEM/FEM/include;D:/GitHub/AFEM/AFEM/cuda/gpu_include;D:/GitHub/AFEM/Geometry;C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include;D:/Downloads/extracted/VTK-6.3.0/build/Charts/Core;D:/Downloads/extracted/VTK-6.3.0/Charts/Core;D:/Downloads/extracted/VTK-6.3.0/build/Common/Color;D:/Downloads/extracted/VTK-6.3.0/Common/Color;D:/Downloads/extracted/VTK-6.3.0/build/Common/DataModel;D:/Downloads/extracted/VTK-6.3.0/Common/DataModel;D:/Downloads/extracted/VTK-6.3.0/build/Common/Math;D:/Downloads/extracted/VTK-6.3.0/Common/Math;D:/Downloads/extracted/VTK-6.3.0/build/Common/Core;D:/Downloads/extracted/VTK-6.3.0/Common/Core;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/KWSys;D:/Downloads/extracted/VTK-6.3.0/Utilities/KWSys;D:/Downloads/extracted/VTK-6.3.0/build/Common/Misc;D:/Downloads/extracted/VTK-6.3.0/Common/Misc;D:/Downloads/extracted/VTK-6.3.0/build/Common/System;D:/Downloads/extracted/VTK-6.3.0/Common/System;D:/Downloads/extracted/VTK-6.3.0/build/Common/Transforms;D:/Downloads/extracted/VTK-6.3.0/Common/Transforms;D:/Downloads/extracted/VTK-6.3.0/build/Infovis/Core;D:/Downloads/extracted/VTK-6.3.0/Infovis/Core;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Extraction;D:/Downloads/extracted/VTK-6.3.0/Filters/Extraction;D:/Downloads/extracted/VTK-6.3.0/build/Common/ExecutionModel;D:/Downloads/extracted/VTK-6.3.0/Common/ExecutionModel;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Core;D:/Downloads/extracted/VTK-6.3.0/Filters/Core;D:/Downloads/extracted/VTK-6.3.0/build/Filters/General;D:/Downloads/extracted/VTK-6.3.0/Filters/General;D:/Downloads/extracted/VTK-6.3.0/build/Common/ComputationalGeometry;D:/Downloads/extracted/VTK-6.3.0/Common/ComputationalGeometry;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Statistics;D:/Downloads/extracted/VTK-6.3.0/Filters/Statistics;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Fourier;D:/Downloads/extracted/VTK-6.3.0/Imaging/Fourier;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Core;D:/Downloads/extracted/VTK-6.3.0/Imaging/Core;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/alglib;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/alglib;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Context2D;D:/Downloads/extracted/VTK-6.3.0/Rendering/Context2D;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Core;D:/Downloads/extracted/VTK-6.3.0/Rendering/Core;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Geometry;D:/Downloads/extracted/VTK-6.3.0/Filters/Geometry;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Sources;D:/Downloads/extracted/VTK-6.3.0/Filters/Sources;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/FreeType;D:/Downloads/extracted/VTK-6.3.0/Rendering/FreeType;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/freetype;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/freetype;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/zlib;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/zlib;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/ftgl/src;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/ftgl;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/ftgl;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/DICOMParser;D:/Downloads/extracted/VTK-6.3.0/Utilities/DICOMParser;D:/Downloads/extracted/VTK-6.3.0/build/Domains/Chemistry;D:/Downloads/extracted/VTK-6.3.0/Domains/Chemistry;D:/Downloads/extracted/VTK-6.3.0/build/IO/XML;D:/Downloads/extracted/VTK-6.3.0/IO/XML;D:/Downloads/extracted/VTK-6.3.0/build/IO/Geometry;D:/Downloads/extracted/VTK-6.3.0/IO/Geometry;D:/Downloads/extracted/VTK-6.3.0/build/IO/Core;D:/Downloads/extracted/VTK-6.3.0/IO/Core;D:/Downloads/extracted/VTK-6.3.0/build/IO/XMLParser;D:/Downloads/extracted/VTK-6.3.0/IO/XMLParser;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/expat;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/expat;D:/Downloads/extracted/VTK-6.3.0/build/Filters/AMR;D:/Downloads/extracted/VTK-6.3.0/Filters/AMR;D:/Downloads/extracted/VTK-6.3.0/build/Parallel/Core;D:/Downloads/extracted/VTK-6.3.0/Parallel/Core;D:/Downloads/extracted/VTK-6.3.0/build/IO/Legacy;D:/Downloads/extracted/VTK-6.3.0/IO/Legacy;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/HashSource;D:/Downloads/extracted/VTK-6.3.0/Utilities/HashSource;D:/Downloads/extracted/VTK-6.3.0/build/Filters/FlowPaths;D:/Downloads/extracted/VTK-6.3.0/Filters/FlowPaths;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Generic;D:/Downloads/extracted/VTK-6.3.0/Filters/Generic;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Hybrid;D:/Downloads/extracted/VTK-6.3.0/Filters/Hybrid;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Sources;D:/Downloads/extracted/VTK-6.3.0/Imaging/Sources;D:/Downloads/extracted/VTK-6.3.0/build/Filters/HyperTree;D:/Downloads/extracted/VTK-6.3.0/Filters/HyperTree;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Imaging;D:/Downloads/extracted/VTK-6.3.0/Filters/Imaging;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/General;D:/Downloads/extracted/VTK-6.3.0/Imaging/General;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Modeling;D:/Downloads/extracted/VTK-6.3.0/Filters/Modeling;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Parallel;D:/Downloads/extracted/VTK-6.3.0/Filters/Parallel;D:/Downloads/extracted/VTK-6.3.0/build/Filters/ParallelImaging;D:/Downloads/extracted/VTK-6.3.0/Filters/ParallelImaging;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Programmable;D:/Downloads/extracted/VTK-6.3.0/Filters/Programmable;D:/Downloads/extracted/VTK-6.3.0/build/Filters/SMP;D:/Downloads/extracted/VTK-6.3.0/Filters/SMP;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Selection;D:/Downloads/extracted/VTK-6.3.0/Filters/Selection;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Texture;D:/Downloads/extracted/VTK-6.3.0/Filters/Texture;D:/Downloads/extracted/VTK-6.3.0/build/Filters/Verdict;D:/Downloads/extracted/VTK-6.3.0/Filters/Verdict;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/verdict;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/verdict;D:/Downloads/extracted/VTK-6.3.0/build/Geovis/Core;D:/Downloads/extracted/VTK-6.3.0/Geovis/Core;D:/Downloads/extracted/VTK-6.3.0/build/Infovis/Layout;D:/Downloads/extracted/VTK-6.3.0/Infovis/Layout;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Hybrid;D:/Downloads/extracted/VTK-6.3.0/Imaging/Hybrid;D:/Downloads/extracted/VTK-6.3.0/build/IO/Image;D:/Downloads/extracted/VTK-6.3.0/IO/Image;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/MetaIO/vtkmetaio;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/MetaIO;D:/Downloads/extracted/VTK-6.3.0/Utilities/MetaIO;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/jpeg;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/jpeg;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/png;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/png;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/tiff;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/tiff;D:/Downloads/extracted/VTK-6.3.0/build/Interaction/Style;D:/Downloads/extracted/VTK-6.3.0/Interaction/Style;D:/Downloads/extracted/VTK-6.3.0/build/Interaction/Widgets;D:/Downloads/extracted/VTK-6.3.0/Interaction/Widgets;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Annotation;D:/Downloads/extracted/VTK-6.3.0/Rendering/Annotation;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Color;D:/Downloads/extracted/VTK-6.3.0/Imaging/Color;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Volume;D:/Downloads/extracted/VTK-6.3.0/Rendering/Volume;D:/Downloads/extracted/VTK-6.3.0/build/Views/Core;D:/Downloads/extracted/VTK-6.3.0/Views/Core;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/libproj4/vtklibproj4;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/libproj4/vtklibproj4;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/libproj4;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/libproj4;D:/Downloads/extracted/VTK-6.3.0/build/IO/AMR;D:/Downloads/extracted/VTK-6.3.0/IO/AMR;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/hdf5/vtkhdf5;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/hdf5/vtkhdf5/hl/src;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/hdf5/vtkhdf5/src;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/hdf5;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/hdf5;D:/Downloads/extracted/VTK-6.3.0/build/IO/EnSight;D:/Downloads/extracted/VTK-6.3.0/IO/EnSight;D:/Downloads/extracted/VTK-6.3.0/build/IO/Exodus;D:/Downloads/extracted/VTK-6.3.0/IO/Exodus;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/exodusII;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/exodusII;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/netcdf/vtknetcdf/include;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/netcdf/vtknetcdf;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/netcdf;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/netcdf;D:/Downloads/extracted/VTK-6.3.0/build/IO/Export;D:/Downloads/extracted/VTK-6.3.0/IO/Export;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/GL2PS;D:/Downloads/extracted/VTK-6.3.0/Rendering/GL2PS;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/ContextOpenGL;D:/Downloads/extracted/VTK-6.3.0/Rendering/ContextOpenGL;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/OpenGL;D:/Downloads/extracted/VTK-6.3.0/Rendering/OpenGL;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/ParseOGLExt;D:/Downloads/extracted/VTK-6.3.0/Utilities/ParseOGLExt;D:/Downloads/extracted/VTK-6.3.0/build/Utilities/EncodeString;D:/Downloads/extracted/VTK-6.3.0/Utilities/EncodeString;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/gl2ps;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/gl2ps;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Label;D:/Downloads/extracted/VTK-6.3.0/Rendering/Label;D:/Downloads/extracted/VTK-6.3.0/build/IO/Import;D:/Downloads/extracted/VTK-6.3.0/IO/Import;D:/Downloads/extracted/VTK-6.3.0/build/IO/Infovis;D:/Downloads/extracted/VTK-6.3.0/IO/Infovis;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/libxml2/vtklibxml2;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/libxml2;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/libxml2;D:/Downloads/extracted/VTK-6.3.0/build/IO/LSDyna;D:/Downloads/extracted/VTK-6.3.0/IO/LSDyna;D:/Downloads/extracted/VTK-6.3.0/build/IO/MINC;D:/Downloads/extracted/VTK-6.3.0/IO/MINC;D:/Downloads/extracted/VTK-6.3.0/build/IO/Movie;D:/Downloads/extracted/VTK-6.3.0/IO/Movie;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/oggtheora;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/oggtheora;D:/Downloads/extracted/VTK-6.3.0/build/IO/NetCDF;D:/Downloads/extracted/VTK-6.3.0/IO/NetCDF;D:/Downloads/extracted/VTK-6.3.0/build/IO/PLY;D:/Downloads/extracted/VTK-6.3.0/IO/PLY;D:/Downloads/extracted/VTK-6.3.0/build/IO/Parallel;D:/Downloads/extracted/VTK-6.3.0/IO/Parallel;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/jsoncpp;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/jsoncpp;D:/Downloads/extracted/VTK-6.3.0/build/IO/ParallelXML;D:/Downloads/extracted/VTK-6.3.0/IO/ParallelXML;D:/Downloads/extracted/VTK-6.3.0/build/IO/SQL;D:/Downloads/extracted/VTK-6.3.0/IO/SQL;D:/Downloads/extracted/VTK-6.3.0/build/ThirdParty/sqlite;D:/Downloads/extracted/VTK-6.3.0/ThirdParty/sqlite;D:/Downloads/extracted/VTK-6.3.0/build/IO/Video;D:/Downloads/extracted/VTK-6.3.0/IO/Video;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Math;D:/Downloads/extracted/VTK-6.3.0/Imaging/Math;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Morphological;D:/Downloads/extracted/VTK-6.3.0/Imaging/Morphological;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Statistics;D:/Downloads/extracted/VTK-6.3.0/Imaging/Statistics;D:/Downloads/extracted/VTK-6.3.0/build/Imaging/Stencil;D:/Downloads/extracted/VTK-6.3.0/Imaging/Stencil;D:/Downloads/extracted/VTK-6.3.0/build/Interaction/Image;D:/Downloads/extracted/VTK-6.3.0/Interaction/Image;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/Image;D:/Downloads/extracted/VTK-6.3.0/Rendering/Image;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/LIC;D:/Downloads/extracted/VTK-6.3.0/Rendering/LIC;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/LOD;D:/Downloads/extracted/VTK-6.3.0/Rendering/LOD;D:/Downloads/extracted/VTK-6.3.0/build/Rendering/VolumeOpenGL;D:/Downloads/extracted/VTK-6.3.0/Rendering/VolumeOpenGL;D:/Downloads/extracted/VTK-6.3.0/build/Views/Context2D;D:/Downloads/extracted/VTK-6.3.0/Views/Context2D;D:/Downloads/extracted/VTK-6.3.0/build/Views/Infovis;D:/Downloads/extracted/VTK-6.3.0/Views/Infovis;D:/GitHub/AFEM/cv_wrappers/include;D:/GitHub/AFEM/include/Eigen") # list (needs to be in quotes to handle spaces properly).
set(CUDA_NVCC_COMPILE_DEFINITIONS "") # list (needs to be in quotes to handle spaces properly).
set(format_flag "-c") # string
set(cuda_language_flag ) # list

# Clean up list of include directories and add -I flags
list(REMOVE_DUPLICATES CUDA_NVCC_INCLUDE_DIRS)
set(CUDA_NVCC_INCLUDE_ARGS)
foreach(dir ${CUDA_NVCC_INCLUDE_DIRS})
  # Extra quotes are added around each flag to help nvcc parse out flags with spaces.
  list(APPEND CUDA_NVCC_INCLUDE_ARGS "-I${dir}")
endforeach()

# Clean up list of compile definitions, add -D flags, and append to nvcc_flags
list(REMOVE_DUPLICATES CUDA_NVCC_COMPILE_DEFINITIONS)
foreach(def ${CUDA_NVCC_COMPILE_DEFINITIONS})
  list(APPEND nvcc_flags "-D${def}")
endforeach()

if(build_cubin AND NOT generated_cubin_file)
  message(FATAL_ERROR "You must specify generated_cubin_file on the command line")
endif()

# This is the list of host compilation flags.  It C or CXX should already have
# been chosen by FindCUDA.cmake.
set(CMAKE_HOST_FLAGS /DWIN32 /D_WINDOWS /W3 /GR /EHsc )
set(CMAKE_HOST_FLAGS_DEBUG /D_DEBUG /MDd /Zi /Ob0 /Od /RTC1)
set(CMAKE_HOST_FLAGS_RELEASE /MD /O2 /Ob2 /DNDEBUG)
set(CMAKE_HOST_FLAGS_MINSIZEREL /MD /O1 /Ob1 /DNDEBUG)
set(CMAKE_HOST_FLAGS_RELWITHDEBINFO /MD /Zi /O2 /Ob1 /DNDEBUG)

# Take the compiler flags and package them up to be sent to the compiler via -Xcompiler
set(nvcc_host_compiler_flags "")
# If we weren't given a build_configuration, use Debug.
if(NOT build_configuration)
  set(build_configuration Debug)
endif()
string(TOUPPER "${build_configuration}" build_configuration)
#message("CUDA_NVCC_HOST_COMPILER_FLAGS = ${CUDA_NVCC_HOST_COMPILER_FLAGS}")
foreach(flag ${CMAKE_HOST_FLAGS} ${CMAKE_HOST_FLAGS_${build_configuration}})
  # Extra quotes are added around each flag to help nvcc parse out flags with spaces.
  string(APPEND nvcc_host_compiler_flags ",\"${flag}\"")
endforeach()
if (nvcc_host_compiler_flags)
  set(nvcc_host_compiler_flags "-Xcompiler" ${nvcc_host_compiler_flags})
endif()
#message("nvcc_host_compiler_flags = \"${nvcc_host_compiler_flags}\"")
# Add the build specific configuration flags
list(APPEND CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS_${build_configuration}})

# Any -ccbin existing in CUDA_NVCC_FLAGS gets highest priority
list( FIND CUDA_NVCC_FLAGS "-ccbin" ccbin_found0 )
list( FIND CUDA_NVCC_FLAGS "--compiler-bindir" ccbin_found1 )
if( ccbin_found0 LESS 0 AND ccbin_found1 LESS 0 AND CUDA_HOST_COMPILER )
  if (CUDA_HOST_COMPILER STREQUAL "$(VCInstallDir)bin" AND DEFINED CCBIN)
    set(CCBIN -ccbin "${CCBIN}")
  else()
    set(CCBIN -ccbin "${CUDA_HOST_COMPILER}")
  endif()
endif()

# cuda_execute_process - Executes a command with optional command echo and status message.
#
#   status  - Status message to print if verbose is true
#   command - COMMAND argument from the usual execute_process argument structure
#   ARGN    - Remaining arguments are the command with arguments
#
#   CUDA_result - return value from running the command
#
# Make this a macro instead of a function, so that things like RESULT_VARIABLE
# and other return variables are present after executing the process.
macro(cuda_execute_process status command)
  set(_command ${command})
  if(NOT "x${_command}" STREQUAL "xCOMMAND")
    message(FATAL_ERROR "Malformed call to cuda_execute_process.  Missing COMMAND as second argument. (command = ${command})")
  endif()
  if(verbose)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E echo -- ${status})
    # Now we need to build up our command string.  We are accounting for quotes
    # and spaces, anything else is left up to the user to fix if they want to
    # copy and paste a runnable command line.
    set(cuda_execute_process_string)
    foreach(arg ${ARGN})
      # If there are quotes, excape them, so they come through.
      string(REPLACE "\"" "\\\"" arg ${arg})
      # Args with spaces need quotes around them to get them to be parsed as a single argument.
      if(arg MATCHES " ")
        list(APPEND cuda_execute_process_string "\"${arg}\"")
      else()
        list(APPEND cuda_execute_process_string ${arg})
      endif()
    endforeach()
    # Echo the command
    execute_process(COMMAND ${CMAKE_COMMAND} -E echo ${cuda_execute_process_string})
  endif()
  # Run the command
  execute_process(COMMAND ${ARGN} RESULT_VARIABLE CUDA_result )
endmacro()

# Delete the target file
cuda_execute_process(
  "Removing ${generated_file}"
  COMMAND "${CMAKE_COMMAND}" -E remove "${generated_file}"
  )

# For CUDA 2.3 and below, -G -M doesn't work, so remove the -G flag
# for dependency generation and hope for the best.
set(depends_CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}")
set(CUDA_VERSION 7.5)
if(CUDA_VERSION VERSION_LESS "3.0")
  cmake_policy(PUSH)
  # CMake policy 0007 NEW states that empty list elements are not
  # ignored.  I'm just setting it to avoid the warning that's printed.
  cmake_policy(SET CMP0007 NEW)
  # Note that this will remove all occurances of -G.
  list(REMOVE_ITEM depends_CUDA_NVCC_FLAGS "-G")
  cmake_policy(POP)
endif()

# nvcc doesn't define __CUDACC__ for some reason when generating dependency files.  This
# can cause incorrect dependencies when #including files based on this macro which is
# defined in the generating passes of nvcc invokation.  We will go ahead and manually
# define this for now until a future version fixes this bug.
set(CUDACC_DEFINE -D__CUDACC__)

# Generate the dependency file
cuda_execute_process(
  "Generating dependency file: ${NVCC_generated_dependency_file}"
  COMMAND "${CUDA_NVCC_EXECUTABLE}"
  -M
  ${CUDACC_DEFINE}
  "${source_file}"
  -o "${NVCC_generated_dependency_file}"
  ${CCBIN}
  ${nvcc_flags}
  ${nvcc_host_compiler_flags}
  ${depends_CUDA_NVCC_FLAGS}
  -DNVCC
  ${CUDA_NVCC_INCLUDE_ARGS}
  )

if(CUDA_result)
  message(FATAL_ERROR "Error generating ${generated_file}")
endif()

# Generate the cmake readable dependency file to a temp file.  Don't put the
# quotes just around the filenames for the input_file and output_file variables.
# CMake will pass the quotes through and not be able to find the file.
cuda_execute_process(
  "Generating temporary cmake readable file: ${cmake_dependency_file}.tmp"
  COMMAND "${CMAKE_COMMAND}"
  -D "input_file:FILEPATH=${NVCC_generated_dependency_file}"
  -D "output_file:FILEPATH=${cmake_dependency_file}.tmp"
  -D "verbose=${verbose}"
  -P "${CUDA_make2cmake}"
  )

if(CUDA_result)
  message(FATAL_ERROR "Error generating ${generated_file}")
endif()

# Copy the file if it is different
cuda_execute_process(
  "Copy if different ${cmake_dependency_file}.tmp to ${cmake_dependency_file}"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${cmake_dependency_file}.tmp" "${cmake_dependency_file}"
  )

if(CUDA_result)
  message(FATAL_ERROR "Error generating ${generated_file}")
endif()

# Delete the temporary file
cuda_execute_process(
  "Removing ${cmake_dependency_file}.tmp and ${NVCC_generated_dependency_file}"
  COMMAND "${CMAKE_COMMAND}" -E remove "${cmake_dependency_file}.tmp" "${NVCC_generated_dependency_file}"
  )

if(CUDA_result)
  message(FATAL_ERROR "Error generating ${generated_file}")
endif()

# Generate the code
cuda_execute_process(
  "Generating ${generated_file}"
  COMMAND "${CUDA_NVCC_EXECUTABLE}"
  "${source_file}"
  ${cuda_language_flag}
  ${format_flag} -o "${generated_file}"
  ${CCBIN}
  ${nvcc_flags}
  ${nvcc_host_compiler_flags}
  ${CUDA_NVCC_FLAGS}
  -DNVCC
  ${CUDA_NVCC_INCLUDE_ARGS}
  )

if(CUDA_result)
  # Since nvcc can sometimes leave half done files make sure that we delete the output file.
  cuda_execute_process(
    "Removing ${generated_file}"
    COMMAND "${CMAKE_COMMAND}" -E remove "${generated_file}"
    )
  message(FATAL_ERROR "Error generating file ${generated_file}")
else()
  if(verbose)
    message("Generated ${generated_file} successfully.")
  endif()
endif()

# Cubin resource report commands.
if( build_cubin )
  # Run with -cubin to produce resource usage report.
  cuda_execute_process(
    "Generating ${generated_cubin_file}"
    COMMAND "${CUDA_NVCC_EXECUTABLE}"
    "${source_file}"
    ${CUDA_NVCC_FLAGS}
    ${nvcc_flags}
    ${CCBIN}
    ${nvcc_host_compiler_flags}
    -DNVCC
    -cubin
    -o "${generated_cubin_file}"
    ${CUDA_NVCC_INCLUDE_ARGS}
    )

  # Execute the parser script.
  cuda_execute_process(
    "Executing the parser script"
    COMMAND  "${CMAKE_COMMAND}"
    -D "input_file:STRING=${generated_cubin_file}"
    -P "${CUDA_parse_cubin}"
    )

endif()

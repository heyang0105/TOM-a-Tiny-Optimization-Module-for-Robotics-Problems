cmake_minimum_required(VERSION 2.8.3)
project(Optim_Module)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

#------------OPTIM Module (Independent Lib)
set(OPTIMIZER_FILES
        optim/optim.h
	optim/core/eigen_types.h	
	optim/core/edge.h
	optim/core/edge.cc
	optim/core/loss_function.cc
	optim/core/loss_function.h
	optim/core/problem_base.cc
	optim/core/problem_base.h
	optim/core/vertex.h
	optim/core/linear_solver.h
	optim/core/linear_solver.cc 
	#
	optim/vertex/vertex_name.cc
	optim/vertex/vertex_inverse_depth.h
	optim/vertex/vertex_point_xyz.h
	optim/vertex/vertex_pose.h
	optim/vertex/vertex_pose.cc
	#
	#source/optim/edge_imu.cc
	#source/optim/edge_imu.h
	#source/optim/edge_prior.cpp
	#source/optim/edge_prior.h
	optim/edge/edge_name.cc
	optim/edge/edge_reprojection.cc
	optim/edge/edge_reprojection.h
	optim/edge/edge_direct_method.h
	optim/edge/edge_direct_method.cc

	optim/sophus/se3.hpp
	optim/sophus/so3.hpp
	optim/sophus/sophus.hpp
	#
	optim/problem/problem_dense.h
	optim/problem/problem_dense.cc
	
)

add_library(${PROJECT_NAME} SHARED
    ${OPTIMIZER_FILES})


add_executable(mono_BA_test test/mono_BA_test.cc)
target_link_libraries(mono_BA_test ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})
add_executable(PNP_test test/PNP_test.cc)
target_link_libraries(PNP_test ${PROJECT_NAME}  ${OpenCV_LIBS})

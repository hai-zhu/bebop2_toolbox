cmake_minimum_required(VERSION 2.8.3)
project(bebop2_nonlinear_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES bebop2_nonlinear_mpc
    # CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include/${PROJECT_NAME}
)

## Declare a C++ library
add_library(single_mpc_lib STATIC
    qpoases/SRC/Bounds.cpp
    qpoases/SRC/Constraints.cpp
    qpoases/SRC/CyclingManager.cpp
    qpoases/SRC/Indexlist.cpp
    qpoases/SRC/MessageHandling.cpp
    qpoases/SRC/QProblem.cpp
    qpoases/SRC/QProblemB.cpp
    qpoases/SRC/SubjectTo.cpp
    qpoases/SRC/Utils.cpp
    qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    single_mpc_export/acado_auxiliary_functions.c
    single_mpc_export/acado_auxiliary_functions.h 
    single_mpc_export/acado_common.h
    single_mpc_export/acado_integrator.c
    single_mpc_export/acado_qpoases_interface.hpp
    single_mpc_export/acado_qpoases_interface.cpp
    single_mpc_export/acado_solver.c
)
target_include_directories(single_mpc_lib PUBLIC
    qpoases
    qpoases/EXTRAS
    qpoases/INCLUDE
    qpoases/SRC
    single_mpc_export
)


cmake_minimum_required(VERSION 2.8.3)
project(bebop2_nonlinear_mpc)

add_compile_options(-std=c++17)

## Set cmake type
# set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf
)

catkin_python_setup()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES bebop2_nonlinear_mpc
    # CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include/${PROJECT_NAME}
)



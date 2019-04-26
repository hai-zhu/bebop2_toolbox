# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )

# Name of this package .
project( bebop2_linear_mpc )

# Find the catkin build system , and any other packages on
# which we depend .
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs tf)
find_package(Eigen3 REQUIRED)

# Declare our catkin package .
catkin_package( )

# Specify locations of header files .
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include/${PROJECT_NAME}
)

include_directories(exportFolder)
link_directories(exportFolder)
add_subdirectory(ThirdParty/exportFolder)

include_directories(qpoases)
link_directories(qpoases)
add_subdirectory(ThirdParty/qpoases)
# Declare the executable , along with its sourcefiles . If
# there are multiple executables ,use multiple copies of
# this line .

add_executable( bebop2_linear_mpc_node bebop2_linear_mpc_node.cpp)
target_link_libraries( bebop2_linear_mpc_node ${catkin_LIBRARIES} qpoases exportFolder)

# Specify libraries against which to link. Again , this
# line should be copied for each distinct executable in
# the package .
#target_link_libraries( bebop_mpc_node ${catkin_LIBRARIES} Eigen3::Eigen)

 

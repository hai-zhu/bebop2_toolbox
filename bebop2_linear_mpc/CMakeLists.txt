cmake_minimum_required(VERSION 2.8.3 )

project( bebop2_linear_mpc )

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs tf)
find_package(Eigen3 REQUIRED)

catkin_package( )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(exportFolder)
include_directories(qpoases)

link_directories(exportFolder)
add_subdirectory(ThirdParty/exportFolder)

link_directories(qpoases)
add_subdirectory(ThirdParty/qpoases)

add_executable( linear_mpc_node linear_mpc_node.cpp)
target_link_libraries( linear_mpc_node ${catkin_LIBRARIES} qpoases exportFolder)

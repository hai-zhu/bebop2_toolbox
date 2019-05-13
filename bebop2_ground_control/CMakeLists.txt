# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )
project( bebop2_ground_control )

find_package(
    catkin REQUIRED 
    COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs
)

catkin_package( )

include_directories( 
    include 
    ${catkin_INCLUDE_DIRS} 
    include/${PROJECT_NAME}
)

add_executable( ground_control_node nodes/ground_control_node.cpp )
target_link_libraries( ground_control_node ${catkin_LIBRARIES} )

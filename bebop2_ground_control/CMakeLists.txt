# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )

# Name of this package .
project( bebop2_ground_control )

# Find the catkin build system , and any other packages on
# which we depend .
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs)


# Declare our catkin package .
catkin_package( )

# Specify locations of header files .
include_directories( 
    include 
    ${catkin_INCLUDE_DIRS} 
    include/${PROJECT_NAME}
)

add_executable( ground_control_node nodes/ground_control_node.cpp )
target_link_libraries( ground_control_node ${catkin_LIBRARIES} )

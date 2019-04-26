# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )

# Name of this package .
project(bebop2_sphinx)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)

# Declare our catkin package .
catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(sphinx_data_logger nodes/sphinx_data_logger.cpp)
target_link_libraries(sphinx_data_logger ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )
project(bebop2_sphinx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)

catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(sphinx_data_logger_node nodes/sphinx_data_logger_node.cpp)
target_link_libraries(sphinx_data_logger_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

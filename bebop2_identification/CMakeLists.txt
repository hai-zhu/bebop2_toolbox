# What version of CMake is needed ?
cmake_minimum_required(VERSION 2.8.3 )

# Name of this package .
project( bebop2_identification )

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

add_executable(bebop_identifier_pitch ./nodes/bebop_identifier_pitch.cpp)
target_link_libraries(bebop_identifier_pitch ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_roll ./nodes/bebop_identifier_roll.cpp)
target_link_libraries(bebop_identifier_roll ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_vertical_speed ./nodes/bebop_identifier_vertical_speed.cpp)
target_link_libraries(bebop_identifier_vertical_speed ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_yaw_speed ./nodes/bebop_identifier_yaw_speed.cpp)
target_link_libraries(bebop_identifier_yaw_speed ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_airdrag_x ./nodes/bebop_identifier_airdrag_x.cpp)
target_link_libraries(bebop_identifier_airdrag_x ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_airdrag_y ./nodes/bebop_identifier_airdrag_y.cpp)
target_link_libraries(bebop_identifier_airdrag_y ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

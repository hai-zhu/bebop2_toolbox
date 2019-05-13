cmake_minimum_required(VERSION 2.8.3)
project(bebop2_identification)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(bebop_identifier_pitch_node nodes/bebop_identifier_pitch_node.cpp)
target_link_libraries(bebop_identifier_pitch_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_roll_node nodes/bebop_identifier_roll_node.cpp)
target_link_libraries(bebop_identifier_roll_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_yaw_node nodes/bebop_identifier_yaw_node.cpp)
target_link_libraries(bebop_identifier_yaw_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bebop_identifier_vz_node nodes/bebop_identifier_vz_node.cpp)
target_link_libraries(bebop_identifier_vz_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

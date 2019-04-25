cmake_minimum_required(VERSION 2.8.3)
project(bebop2_state_estimator)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        message_generation
        bebop2_msgs
)


find_package(Eigen3 REQUIRED)


# generate_messages(
#         DEPENDENCIES
#         std_msgs
#         geometry_msgs
#         nav_msgs
# )


catkin_package(
        CATKIN_DEPENDS 
        roscpp rospy 
        std_msgs 
        geometry_msgs 
        message_runtime
        bebop2_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include/${PROJECT_NAME}
)


add_executable(full_state_kalman_filter_node nodes/full_state_kalman_filter_node.cpp src/full_state_kalman_filter.cpp)
target_link_libraries(full_state_kalman_filter_node ${catkin_LIBRARIES})

add_executable(position_velocity_orientation_filter_node nodes/position_velocity_orientation_filter_node.cpp src/position_velocity_orientation_filter.cpp)
target_link_libraries(position_velocity_orientation_filter_node ${catkin_LIBRARIES})


## For debugging
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)

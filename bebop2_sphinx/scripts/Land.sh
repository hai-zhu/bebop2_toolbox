#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub --once /bebop2/land std_msgs/Empty

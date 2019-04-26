#!/bin/bash

sudo ./Land.sh

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub --once /simulator/reset std_msgs/Bool '{data: true}'

sudo ./Takeoff.sh


#!/bin/bash

# start Sphinx
gnome-terminal --tab "sphinx" -x bash -c "sudo systemctl start firmwared.service;GAZEBO_PLUGIN_PATH=`pwd`/build:$GAZEBO_PLUGIN_PATH sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/worlds/empty.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_eth0.drone;exec bash;"
sleep 20s


# start data logger
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "data logger" -x bash -c "rosrun bebop2_sphinx sphinx_data_logger;exec bash;"
sleep 1s

# start data logger
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab "pub_15_hz" -x bash -c "rosrun topic_tools throttle messages /simulator/odometry 15 /bebop2/odometry;exec bash;"
sleep 1s



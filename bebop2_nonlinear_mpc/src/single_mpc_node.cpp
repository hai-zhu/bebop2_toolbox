/*
 * @Description: Node source file of the simple single drone mpc controller
 * @Author: Hai Zhu
 * @Affiliation: Delft University of Technology
 * @Date: 2019-06-04 12:42:16
 * @LastEditTime: 2019-06-06 12:33:31
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

// #include <acado_common.h>
// #include <acado_auxiliary_functions.h>

#include "bebop2_nonlinear_mpc/single_mpc.h"

int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "nonlinear_mpc_node");
    ros::NodeHandle nh;

    // Create controller object
    Single_MPC mpcNode(nh);

    ros::spin();

    return 0;

    
}

/*
 * @Description: Class for single drone mpc controller
 * @Author: Hai Zhu
 * @Affiliation: Delft University of Technology
 * @Date: 2019-06-04 12:50:16
 * @LastEditTime: 2019-06-05 18:21:34
 */

#ifndef BEBOP2_NONLINEAR_MPC_SINGLE_MPC_H
#define BEBOP2_NONLINEAR_MPC_SINGLE_MPC_H

// General include
#include <math.h>
#include <vector>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Ros include
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

// Acado include
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// Acado macro
#define NX          ACADO_NX	    // number of differential states
#define NXA         ACADO_NXA	    // number of algebraic stats
#define NU          ACADO_NU	    // number of control inputs
#define N          	ACADO_N		    // number of control intervals
#define NY			ACADO_NY	    // number of references, nodes 0..N - 1
#define NYN			ACADO_NYN
#define VERBOSE     1			    // show iterations: 1, silent: 0
#define PI          3.14159265358979323846

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// Bebop control commands limit
const double    maxRoll = 12.0*PI / 180.0;
const double   maxPitch = 12.0*PI / 180.0;
const double      maxVz = 1.0;
const double maxYawRate = 90.0*PI / 180.0;

class Single_MPC
{
public:
    //! Constructor, "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    Single_MPC(ros::NodeHandle nh);
    virtual ~Single_MPC();

private:
    //! Ros node handle
    ros::NodeHandle     nh_;                    // to pass between "main" and constructor

    //! Some objects to support subscriber, service, and publisher
    ros::Subscriber     ego_odom_sub_;          // mav ego state subscriber
    ros::Publisher      cmd_vel_pub_;           // mav control input publisher
    ros::Publisher      path_plan_pub_;         // mav planned path publisher

    //! Ego-agent state
    Eigen::Matrix<double, 3, 1>     ego_pos_;   // x, y, z [m]
    Eigen::Matrix<double, 3, 1>     ego_vel_;   // vx, vy, vz, [m/s]
    Eigen::Matrix<double, 3, 1>     ego_euler_; // roll, pitch, yaw [rad]

    //! Optimization output
    Eigen::Matrix<double, 4, 1>     control_optimal_;
    Eigen::Matrix<double, 6, N>     path_plan_;

    //! Subscriber callback, solving the optimization problem
    void mpc_solve(const nav_msgs::Odometry& msg);

};



#endif  // BEBOP2_NONLINEAR_MPC_SINGLE_MPC_H

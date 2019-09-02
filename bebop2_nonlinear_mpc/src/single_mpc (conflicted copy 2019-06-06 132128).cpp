/*
 * @Description: Source file of the single_mpc class
 * @Author: Hai Zhu
 * @Affiliation: Delft University of Technology
 * @Date: 2019-06-04 12:50:30
 * @LastEditTime: 2019-06-06 13:21:27
 */

#include "bebop2_nonlinear_mpc/single_mpc.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//! Constructor
Single_MPC::Single_MPC(ros::NodeHandle nh) : nh_(nh)
{
    // Initialize subscriber
    ego_odom_sub_ = nh_.subscribe("/bebop/odometry", 1, &Single_MPC::mpc_solve, this);
    
    // Initialize publisher
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/bebop_auto/cmd_vel", 1);
    path_plan_pub_ = nh_.advertise<nav_msgs::Path>("/bebop/path_plan", 1);

    // Variable initialization
    ego_pos_.setZero();
    ego_vel_.setZero();
    ego_euler_.setZero();
    control_optimal_.setZero();
    path_plan_.setZero();

    // Reset all acado solver memory
	// memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	// memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the acado solver
    acado_initializeSolver();

}

Single_MPC::~Single_MPC()
{
    
}

//! Callback function, solving the optimization problem
void Single_MPC::mpc_solve(const nav_msgs::Odometry& msg)
{
    // Start clock
    clock_t t_start = clock();

    // Read ego-agent odometry information
    ego_pos_(0) = msg.pose.pose.position.x;         // current position
    ego_pos_(1) = msg.pose.pose.position.y;
    ego_pos_(2) = msg.pose.pose.position.z;
    ego_vel_(0) = msg.twist.twist.linear.x;         // current velocity
    ego_vel_(1) = msg.twist.twist.linear.y;
    ego_vel_(2) = msg.twist.twist.linear.z;
    tf::Quaternion ego_quat;                        // convert current quaternion into euler angles
    tf::quaternionMsgToTF(msg.pose.pose.orientation, ego_quat);
    tf::Matrix3x3(ego_quat).getRPY(ego_euler_(0), ego_euler_(1), ego_euler_(2));

    // Current state feedback to the solver
    acadoVariables.x0[ 0 ] = ego_pos_(0);           // x
    acadoVariables.x0[ 1 ] = ego_pos_(1);           // y
    acadoVariables.x0[ 2 ] = ego_pos_(2);           // z
    acadoVariables.x0[ 3 ] = ego_vel_(0);           // vx
    acadoVariables.x0[ 4 ] = ego_vel_(1);           // vy
    acadoVariables.x0[ 5 ] = ego_vel_(2);           // vz
    acadoVariables.x0[ 6 ] = ego_euler_(0);         // roll
    acadoVariables.x0[ 7 ] = ego_euler_(1);         // pitch
    acadoVariables.x0[ 8 ] = ego_euler_(2);         // yaw

    acado_preparationStep();

    // Call the solver
    int solver_status;
    solver_status = acado_feedbackStep();
    ROS_INFO_STREAM("SOLVER STATUS: " << solver_status);

    // Obtain optimized control input and planned path
    if ( solver_status == 0)                        // if feasible solution is obtained
    {
        // optimized control
        control_optimal_(0) = acadoVariables.u[ 0 ];// commanded roll
        control_optimal_(1) = acadoVariables.u[ 1 ];// commanded pitch
        control_optimal_(2) = acadoVariables.u[ 2 ];// commanded vertical velocity
        control_optimal_(3) = acadoVariables.u[ 3 ];// commanded yaw rate
        // planned path
        for (int i = 0; i < N; i++)
        {
            path_plan_(0, i) = acadoVariables.x[ i*NX + 0 ];    // x
            path_plan_(1, i) = acadoVariables.x[ i*NX + 1 ];    // y
            path_plan_(2, i) = acadoVariables.x[ i*NX + 2 ];    // z
            path_plan_(3, i) = acadoVariables.x[ i*NX + 6 ];    // roll
            path_plan_(4, i) = acadoVariables.x[ i*NX + 7 ];    // pitch
            path_plan_(5, i) = acadoVariables.x[ i*NX + 8 ];    // yaw
        }
    }
    else
    {
        control_optimal_.setZero();                 // if infeasible, set control input to be zero
        path_plan_.setZero();
    }
    
    // Publish the control input command
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control_optimal_(1) / maxPitch;      // along x direction, pitch
    cmd_vel.linear.y = -control_optimal_(0) / maxRoll;      // along negative y direction, roll
    cmd_vel.linear.y = control_optimal_(2) / maxVz;         // along z direction, vertical velocity
    cmd_vel.angular.z = control_optimal_(3) / maxYawRate;   // commanded yaw rate
    cmd_vel_pub_.publish(cmd_vel);
    
    // Publish planned path
    nav_msgs::Path path_plan;
    for (int i = 0; i < N; i++)
    {
        geometry_msgs::PoseStamped pose_plan;
        // position
        pose_plan.pose.position.x = path_plan_(0, i);
        pose_plan.pose.position.y = path_plan_(1, i);
        pose_plan.pose.position.z = path_plan_(2, i);
        // convert euler angles to quaternion
        pose_plan.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(path_plan_(3, i), path_plan_(4, i), path_plan_(5, 1));
        // push into the path
        path_plan.poses.push_back(pose_plan);
    }
    path_plan_pub_.publish(path_plan);

    // End clock
    clock_t t_end = clock();
	std::cout << "[INFO] solving time: " << ((float)(t_end - t_start))/CLOCKS_PER_SEC << std::endl;

}

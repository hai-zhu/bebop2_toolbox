//
// Created by hai on 4/24/19.
//

#ifndef BEBOP2_STATE_ESTIMATOR_POSITION_VELOCITY_ORIENTATION_FILTER_H
#define BEBOP2_STATE_ESTIMATOR_POSITION_VELOCITY_ORIENTATION_FILTER_H

// General include
#include <math.h>
#include <vector>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Ros include
#include <ros/ros.h>

// Message types
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

// Custom message includes. Auto-generated from msg/ directory.


// Define a class, including a constructor, member variables and member functions
class Position_Velocity_Orientation_Filter
{
public:
    //! Constructor, "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    explicit Position_Velocity_Orientation_Filter(ros::NodeHandle nh);

private:
    //! Ros node handle
    ros::NodeHandle     nh_;        // we will need this, to pass between "main" and constructor

    //! Some objects to support subscriber, service, and publisher
    ros::Subscriber     sub_;
    ros::Publisher      pub_;

    //! Bebop2 measurement
    Eigen::Vector3d     pos_measured_;          // measured position information

    //! Time information for filter
    ros::Time           time_stamp_;            // time stamp of current measurement
    ros::Time           time_stamp_previous_;   // time stamp of last measurement
    double              dt_;                    // time difference between two measurements

    //! State estimation
    Eigen::Matrix<double, 6, 1> pos_vel_estimated_;     // estimated state (pos & vel)
    Eigen::Matrix<double, 6, 6> pos_vel_cov_estimated_; // estimated covariance matrix

    //! Initializations
    void initializeSubscribers();
    void initializePublishers();

    //! Subscriber callback
    void subscriberCallback(const geometry_msgs::PoseStamped &msg);

};



#endif //BEBOP2_STATE_ESTIMATOR_POSITION_VELOCITY_ORIENTATION_FILTER_H

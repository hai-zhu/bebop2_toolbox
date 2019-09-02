//
// Created by hai on 4/24/19.
//

#include <bebop2_state_estimator/full_state_kalman_filter.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "full_state_kalman_filter_node");     // node name
    ros::NodeHandle nh;                                         // create a node handle

    // Initialize a class object and pass node handle for constructor
    Full_State_Kalman_Filter full_state_kalman_filter(nh);

    // not specify the publishing rate
    ros::spin();


    return 0;
}
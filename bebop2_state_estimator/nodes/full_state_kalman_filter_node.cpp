//
// Created by hai on 4/24/19.
//

#include <bebop2_state_estimator/full_state_kalman_filter.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "full_state_kalman_filter_node");     // node name
    ros::NodeHandle nh;                                         // create a node handle

    // Set parameters
    double node_rate;                               // node rate
    if (!nh.getParam(ros::this_node::getName()+"/node_rate", node_rate))
    {
        ROS_ERROR_STREAM("full_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/node_rate not set");
        return 0;
    }

    std::string sub_topic;                          // subscriber topic name
    if (!nh.getParam(ros::this_node::getName()+"/bebop2/pose_measured", sub_topic))
    {
        ROS_ERROR_STREAM("full_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/pose_measured not set");
        return 0;
    }

    std::string pub_topic;                          // publisher topic name
    if (!nh.getParam(ros::this_node::getName()+"/bebop2/full_state_estimation", pub_topic))
    {
        ROS_ERROR_STREAM("full_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/full_state_estimation not set");
        return 0;
    }

    // Initialize a class object and pass node handle for constructor
    Full_State_Kalman_Filter full_state_kalman_filter(nh, sub_topic, pub_topic, node_rate);

    // Let ROS handle all callbacks
    // specify the publishing rate
    // ros::Rate loop_rate(node_rate);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // not specify the publishing rate
    ros::spin();


    return 0;
}
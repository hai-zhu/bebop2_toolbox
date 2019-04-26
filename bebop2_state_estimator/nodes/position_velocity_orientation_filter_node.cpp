//
// Created by hai on 4/24/19.
//

#include <bebop2_state_estimator/position_velocity_orientation_filter.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "position_velocity_orientation_filter_node");     // node name
    ros::NodeHandle nh;                                         // create a node handle

    // Set parameters
    double node_rate;                               // node rate
    if (!nh.getParam(ros::this_node::getName()+"/node_rate", node_rate))
    {
        ROS_ERROR_STREAM("position_velocity_orientation_filter_node Parameter " << ros::this_node::getName()+"/node_rate not set");
        return 0;
    }

    std::string sub_topic;                          // subscriber topic name
    if (!nh.getParam(ros::this_node::getName()+"/bebop2/pose_measured", sub_topic))
    {
        ROS_ERROR_STREAM("position_velocity_orientation_filter_node Parameter " << ros::this_node::getName()+"/pose_measured not set");
        return 0;
    }

    std::string pub_topic;                          // publisher topic name
    if (!nh.getParam(ros::this_node::getName()+"/bebop2/position_velocity_orientation_estimation", pub_topic))
    {
        ROS_ERROR_STREAM("position_velocity_orientation_filter_node Parameter " << ros::this_node::getName()+"/position_velocity_orientation_estimation not set");
        return 0;
    }

    // Initialize a class object and pass node handle for constructor
    Position_Velocity_Orientation_Filter position_velocity_orientation_filter(nh, sub_topic, pub_topic, node_rate);

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
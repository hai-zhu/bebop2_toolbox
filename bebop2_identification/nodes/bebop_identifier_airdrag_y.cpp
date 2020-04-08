#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <math.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bebop_identifier_pitch");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);  // in Hz

	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/bebop2/cmd_vel",1);
	ros::Publisher cmd_log_pub = nh.advertise<geometry_msgs::TwistStamped>("/data_logger/cmd_vel",1);

	// start time
	ros::Time startTime = ros::Time::now();
	// differential time
	ros::Duration diffTime = ros::Time::now() - startTime;
	double dt = diffTime.toSec();
	while (ros::ok() && dt <= 50.0)
	{
		double cmd = (int(dt/3) % 4 - 1==2)?0:int(dt/3) % 4 - 1;
		geometry_msgs::Twist msg;
		msg.linear.x = 0; 
		msg.linear.y = cmd;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0; // 90 rad/s
		cmd_pub.publish(msg);
		// publish message with stamp
		geometry_msgs::TwistStamped msgStamped;
		msgStamped.header.stamp = ros::Time::now();
		msgStamped.twist.linear.x = 0; 
		msgStamped.twist.linear.y = cmd;
		msgStamped.twist.linear.z = 0;
		msgStamped.twist.angular.x = 0;
		msgStamped.twist.angular.y = 0;
		msgStamped.twist.angular.z = 0; // 90 rad/s
		cmd_log_pub.publish(msgStamped);
		ros::spinOnce();
		loop_rate.sleep();
		diffTime = ros::Time::now() - startTime;
		dt = diffTime.toSec();
	}
	cout << "[INFO]DONE!!!" << endl;
	return 0;
}
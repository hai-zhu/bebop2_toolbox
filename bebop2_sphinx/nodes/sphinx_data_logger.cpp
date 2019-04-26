#include "ros/ros.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <math.h>

using namespace std;

class data_logger
{
	public:
		gazebo::transport::SubscriberPtr odom_gazebo_sub;
		ros::Subscriber reset_sub;
		ros::Subscriber random_reset_sub;
		gazebo::transport::PublisherPtr reset_pub;
		ros::Publisher odom_pub;

		bool isInited = false;
		nav_msgs::Odometry odom_temp;

	data_logger(){}
	void read_pose(ConstPosesStampedPtr &msg);
	void reset(const std_msgs::Bool& msg);
	void random_reset(const std_msgs::Bool& msg);
};

void data_logger::reset(const std_msgs::Bool& msg)
{
	if(msg.data)
	{
		gazebo::msgs::Model msg_model;
		gazebo::msgs::Pose *msg_pose = new gazebo::msgs::Pose();
		gazebo::msgs::Quaternion *msg_quaternion = new gazebo::msgs::Quaternion();
		gazebo::msgs::Vector3d *msg_vector = new gazebo::msgs::Vector3d();
		// set rotation
		msg_quaternion->set_w(1);
		msg_quaternion->set_x(0);
		msg_quaternion->set_y(0);
		msg_quaternion->set_z(0);
		// set position
		msg_vector->set_x(0);
		msg_vector->set_y(0);
		msg_vector->set_z(0);
		// set pose
		msg_pose->set_allocated_orientation(msg_quaternion);
    	msg_pose->set_allocated_position(msg_vector);
		// set model
		msg_model.set_allocated_pose(msg_pose);
    	msg_model.set_name("bebop2");
		this->reset_pub->Publish(msg_model,true);
	}
}

void data_logger::random_reset(const std_msgs::Bool& msg)
{
	if(msg.data)
	{
		gazebo::msgs::Model msg_model;
		gazebo::msgs::Pose *msg_pose = new gazebo::msgs::Pose();
		gazebo::msgs::Quaternion *msg_quaternion = new gazebo::msgs::Quaternion();
		gazebo::msgs::Vector3d *msg_vector = new gazebo::msgs::Vector3d();
		// set rotation
		double w = (rand() % 2000 -1000)/1000.0;
		double z = (rand() % 2 == 0)?sqrtf(1 - powf(w,2)):-sqrtf(1 - powf(w,2));
		msg_quaternion->set_w(w);
		msg_quaternion->set_x(0);
		msg_quaternion->set_y(0);
		msg_quaternion->set_z(z);
		// set position
		msg_vector->set_x((rand() % 1000 -500)/100.0);
		msg_vector->set_y((rand() % 1000 -500)/100.0);
		msg_vector->set_z(1.0);
		// set pose
		msg_pose->set_allocated_orientation(msg_quaternion);
    	msg_pose->set_allocated_position(msg_vector);
		// set model
		msg_model.set_allocated_pose(msg_pose);
    	msg_model.set_name("bebop2");
		this->reset_pub->Publish(msg_model,true);
	}
}

// Reading Follower pose from Gazebo
void data_logger::read_pose(ConstPosesStampedPtr &msg)
{
	for (int i =0; i < msg->pose_size(); ++i)
    	{
		const ::gazebo::msgs::Pose &pose = msg->pose(i);
		if (pose.name() == "bebop2")
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion q = pose.orientation();
			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "world";
			odom.pose.pose.position.x = position.x();
			odom.pose.pose.position.y = position.y();
			odom.pose.pose.position.z = position.z();
			odom.pose.pose.orientation.x = q.x();
			odom.pose.pose.orientation.y = q.y();
			odom.pose.pose.orientation.z = q.z();
			odom.pose.pose.orientation.w = q.w();
			if(this->isInited)
			{
				ros::Duration diff = odom.header.stamp - this->odom_temp.header.stamp;
				double dt = diff.toSec();
				odom.twist.twist.linear.x = (odom.pose.pose.position.x - odom_temp.pose.pose.position.x) / dt;
				odom.twist.twist.linear.y = (odom.pose.pose.position.y - odom_temp.pose.pose.position.y) / dt;
				odom.twist.twist.linear.z = (odom.pose.pose.position.z - odom_temp.pose.pose.position.z) / dt;
				// differential
			}
			else
			{
				this->isInited = true;
			}
			this->odom_pub.publish(odom);
			this->odom_temp = odom;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sphinx_data_logger");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);  // in Hz

	data_logger my_logger;

	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	my_logger.reset_sub = nh.subscribe("/simulator/reset", 1, &data_logger::reset, &my_logger);
	my_logger.random_reset_sub = nh.subscribe("/simulator/random_reset", 1, &data_logger::random_reset, &my_logger);
	my_logger.odom_gazebo_sub = node->Subscribe("/gazebo/default/pose/info", &data_logger::read_pose, &my_logger);
    my_logger.reset_pub = node->Advertise<gazebo::msgs::Model>("/gazebo/default/model/modify",1);
	my_logger.odom_pub = nh.advertise<nav_msgs::Odometry>("/simulator/odometry",1);
	my_logger.reset_pub->WaitForConnection();

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}
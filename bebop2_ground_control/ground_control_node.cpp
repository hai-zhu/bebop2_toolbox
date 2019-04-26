#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class ground_controller
{
    public:
        // member
        ros::NodeHandle nh;   
        ros::Subscriber joy_sub;
        ros::Subscriber auto_sub;
        ros::Publisher cmd_pub;
        ros::Publisher takeoff_pub;
        ros::Publisher landing_pub;
        bool auto_navigation;

    ground_controller()
    {
        bool auto_navigation = false;
        joy_sub = nh.subscribe("/joy", 1, &ground_controller::from_joystick, this);
        auto_sub = nh.subscribe("/bebop2_auto/cmd_vel", 1, &ground_controller::auto_nav, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/bebop2/cmd_vel", 1);
        takeoff_pub = nh.advertise<std_msgs::Empty>("/bebop2/takeoff", 1);
        landing_pub = nh.advertise<std_msgs::Empty>("/bebop2/land", 1);
    }

    void from_joystick(const sensor_msgs::Joy& msg)
    {
        // axes 0 -> yawrate(left)
        // axes 1 -> vertical velocity(up)
        // axes 3 -> roll(left)
        // axes 4 -> pitch(forward)
        // buttons 0 -> A
        // buttons 1 -> B
        // buttons 2 -> X
        // buttons 3 -> Y
        // buttons 6 -> back
        // buttons 7 -> start

        ROS_INFO_STREAM("AUTO MODE: " << this->auto_navigation);

        // piloting
        if(!this->auto_navigation)
        {
            geometry_msgs::Twist cmd;
            cmd.linear.x = msg.axes[4];
            cmd.linear.y = msg.axes[3];
            cmd.linear.z = msg.axes[1];
            cmd.angular.z = msg.axes[0];
            cmd_pub.publish(cmd);
        }

        // take off
        if(msg.buttons[0])
        {
            // switch to manual control after takeoff
            this->auto_navigation = false;
            std_msgs::Empty takeoff_msg;
            takeoff_pub.publish(takeoff_msg);
        }

        // landing
        if(msg.buttons[1])
        {
            std_msgs::Empty landing_msg;
            landing_pub.publish(landing_msg);
        }

        // enable auto-navigation
        if(msg.buttons[7])
        {
            this->auto_navigation = true;
        }

        // disable auto-navigation
        if(msg.buttons[6])
        {
            this->auto_navigation = false;
        }
    }

    void auto_nav(const geometry_msgs::Twist& msg)
    {
        if(this->auto_navigation)
        {
            cmd_pub.publish(msg);
        }
    }
};

int main(int argc, char **argv)  
{  
	//Initiate ROS  
	ros::init(argc, argv, "ground_controller");  

	//Create an object of class SubscribeAndPublish that will take care of everything  
	ground_controller object; 

	ros::spin();  
	return 0;  
}  
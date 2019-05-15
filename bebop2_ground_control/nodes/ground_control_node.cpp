#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class Ground_Controller
{
    public:
        //! Members
        ros::NodeHandle     nh_;                // ros handle
        ros::Subscriber     joy_sub_;           // subscriber to the joy stick commands
        ros::Subscriber     auto_sub_;          // subscriber to the auto navigation commands
        ros::Publisher      cmd_pub_;           // control commands sent to the quadrotor for execution
        ros::Publisher      takeoff_pub_;       // take off message publisher
        ros::Publisher      landing_pub_;       // landing message publisher
        ros::Publisher      cmd_stamped_pub_;   // time stamped control commands for recording
        bool                auto_navigation_;   // if in auto mode
        bool                landing_;           // if needs to land
        int                 node_rate_;         // control node rate

    public:
        //! Constructor
        explicit Ground_Controller(ros::NodeHandle nh): nh_(nh)
        {
            // Auto navigation is disabled by default
            auto_navigation_ = false;
            landing_         = false;
            node_rate_       = 20;

            // Initialize subscribers
            joy_sub_ = nh_.subscribe("/bebop/joy", 1, &Ground_Controller::from_joystick, this);
            auto_sub_ = nh_.subscribe("/bebop_auto/cmd_vel", 1, &Ground_Controller::auto_nav, this);

            // Initialize publishers
            takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
            landing_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/land", 1);
            cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
            cmd_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/bebop/cmd_vel_stamped", 1);
        }

        //! Passing joystick commands
        void from_joystick(const sensor_msgs::Joy &msg)
        {
            // axes 0 -> yawrate(left)
            // axes 1 -> vertical velocity(up)
            // axes 2 -> roll(left)
            // axes 3 -> pitch(forward)
            // buttons 0 -> X
            // buttons 1 -> A
            // buttons 2 -> B
            // buttons 3 -> Y
            // buttons 8 -> back
            // buttons 9 -> start

            ROS_INFO_STREAM("AUTO MODE: " << this->auto_navigation_);

            // take off
            if(msg.buttons[1])                          // A
            {
                this->auto_navigation_ = false;
                this->landing_ = false;
                std_msgs::Empty takeoff_msg;
                takeoff_pub_.publish(takeoff_msg);
            }

            // landing
            if(msg.buttons[2])                          // B
            {
                this->landing_ = true;
                std_msgs::Empty landing_msg;
                landing_pub_.publish(landing_msg);
            }

            // piloting
            if(!this->auto_navigation_ && !this->landing_)
            {
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = msg.axes[3];
                cmd_vel.linear.y = msg.axes[2];
                cmd_vel.linear.z = msg.axes[1];
                cmd_vel.angular.z = msg.axes[0];
                cmd_pub_.publish(cmd_vel);

                geometry_msgs::TwistStamped cmd_vel_stamped;
                cmd_vel_stamped.header.stamp = ros::Time::now();
                cmd_vel_stamped.twist = cmd_vel;
                cmd_stamped_pub_.publish(cmd_vel_stamped);
            }

            // enable auto-navigation
            if(msg.buttons[9])                      // Start
            {
                this->auto_navigation_ = true;
            }

            // disable auto-navigation
            if(msg.buttons[8])                      // Back
            {
                this->auto_navigation_ = false;
            }

        }

        void auto_nav(const geometry_msgs::Twist &msg)
        {
            if(this->auto_navigation_ && !this->landing_)
            {
                cmd_pub_.publish(msg);
                geometry_msgs::TwistStamped cmd_vel_stamped;
                cmd_vel_stamped.header.stamp = ros::Time::now();
                cmd_vel_stamped.twist = msg;
                cmd_stamped_pub_.publish(cmd_vel_stamped);
            }
        }
};

int main(int argc, char **argv)  
{  
	// Initialize ROS  
	ros::init(argc, argv, "ground_control_node");        // node name
    ros::NodeHandle nh;  

	// Create an object of class
	Ground_Controller ground_controller(nh);

    // Not specify the node rate
	// ros::spin();  

    // Specify the node rate
    int node_rate;
    node_rate = ground_controller.node_rate_;
    ros::Rate loop_rate(node_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    

	return 0;  
}  
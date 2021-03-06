//
// Created by hai on 4/24/19.
//


#include <bebop2_state_estimator/full_state_kalman_filter.h>

// Constructor:  this will get called whenever an instance of this class is created
Full_State_Kalman_Filter::Full_State_Kalman_Filter(ros::NodeHandle nh): nh_(nh)
{
    ROS_INFO("In class constructor of Full_State_Kalman_Filter");

    // Initialization subscriber and publisher
    initializeSubscribers();
    initializePublishers();

    // Initialization the state covariance, this is important for starting the Kalman filter
    double cov_pos = 1^2;
    double cov_vel = 1^2;
    pos_vel_cov_estimated_.setZero();
    pos_vel_cov_estimated_(0, 0) = cov_pos;
    pos_vel_cov_estimated_(1, 1) = cov_pos;
    pos_vel_cov_estimated_(2, 2) = cov_pos;
    pos_vel_cov_estimated_(3, 3) = cov_vel;
    pos_vel_cov_estimated_(4, 4) = cov_vel;
    pos_vel_cov_estimated_(5, 5) = cov_vel;

    double cov_euler  = 1^2;
    double cov_rate = 1^2;
    euler_rate_cov_estimated_.setZero();
    euler_rate_cov_estimated_(0, 0) = cov_euler;
    euler_rate_cov_estimated_(1, 1) = cov_euler;
    euler_rate_cov_estimated_(2, 2) = cov_euler;
    euler_rate_cov_estimated_(3, 3) = cov_rate;
    euler_rate_cov_estimated_(4, 4) = cov_rate;
    euler_rate_cov_estimated_(5, 5) = cov_rate;

    // Other initialization
    pos_measured_.setZero();
    euler_measured_.setZero();
    pos_vel_estimated_.setZero();
    euler_rate_estimated_.setZero();

    time_stamp_ = ros::Time::now();
    time_stamp_previous_ = ros::Time::now();
    dt_ = 0.01;

}


// Set up subscribers
void Full_State_Kalman_Filter::initializeSubscribers()
{
    ROS_INFO("Initializing subscribers");
    sub_ = nh_.subscribe("/bebop/pose", 1, &Full_State_Kalman_Filter::subscriberCallback, this);
}


// Set up publisher
void Full_State_Kalman_Filter::initializePublishers()
{
    ROS_INFO("Initializing publishers");
    pub_ = nh_.advertise<bebop2_msgs::FullStateWithCovarianceStamped>("/bebop/full_state_estimation", 1, true);
}


// Subscriber callback function
void Full_State_Kalman_Filter::subscriberCallback(const geometry_msgs::PoseStamped &msg)
{
    // the real work is done in this callback function
    // it wakes up every time a new message is published on bebop2_sub_topic_

    // for debugging
    // ROS_INFO("Filtering");

    // get measured position
    pos_measured_(0) = msg.pose.position.x;
    pos_measured_(1) = msg.pose.position.y;
    pos_measured_(2) = msg.pose.position.z;

    // get measure orintention and convert to euler angles, seems to be not very correct
//     Eigen::Quaterniond quaternion_measured;
//     quaternion_measured.w() = msg.pose.orientation.w;
//     quaternion_measured.x() = msg.pose.orientation.x;
//     quaternion_measured.y() = msg.pose.orientation.y;
//     quaternion_measured.z() = msg.pose.orientation.z;
//     euler_measured_ = quaternion_measured.toRotationMatrix().eulerAngles(0, 1, 2);  // roll, pitch and yaw

    // Using tf for conversion
    tf::Quaternion quat_measured;
    tf::quaternionMsgToTF(msg.pose.orientation, quat_measured);
    tf::Matrix3x3(quat_measured).getRPY(euler_measured_(0), euler_measured_(1), euler_measured_(2));
//     std::cout << "Roll:" << euler_measured_(0)*180/3.1415 << std::endl;
//     std::cout << "Pitch:" << euler_measured_(1)*180/3.1415 << std::endl;
//     std::cout << "Yaw:" << euler_measured_(2)*180/3.1415 << std::endl << std::endl;

    // current time stamp of the message
    time_stamp_      = msg.header.stamp;

    // time difference. If using the node_rate to derive, then comment the following lines
    dt_ = (time_stamp_ - time_stamp_previous_).toSec();

    // perform Kalman Filtering, first for position and velocity
    // matrix of state transition model
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0, 0, dt_, 0, 0,
         0, 1, 0, 0, dt_, 0,
         0, 0, 1, 0, 0, dt_,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 3> B;
    double dt_2 = 0.5*dt_*dt_;
    B << dt_2, 0, 0,
         0, dt_2, 0,
         0, 0, dt_2,
         0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    // matrix of observation model
    Eigen::Matrix<double, 3, 6> H;
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    // noise matrix
    double R_pos = 10E-4;
    Eigen::Matrix<double, 3, 3> R;      // observation noise covariance
    R << R_pos, 0, 0,
         0, R_pos, 0,
         0, 0, R_pos;
    double Q_acc = 1000;                // mean of noisy acceleration
    double Q_pos = Q_acc * dt_2;
    double Q_vel = Q_acc * dt_;
    Eigen::Matrix<double, 6, 6> Q;
    Q << Q_pos, 0, 0, 0, 0, 0,
         0, Q_pos, 0, 0, 0, 0,
         0, 0, Q_pos, 0, 0, 0,
         0, 0, 0, Q_vel, 0, 0,
         0, 0, 0, 0, Q_vel, 0,
         0, 0, 0, 0, 0, Q_vel;

    // prediction
    Eigen::Vector3d u(0, 0, 0);
    pos_vel_estimated_ = A*pos_vel_estimated_ + B*u;
    pos_vel_cov_estimated_ = A*pos_vel_cov_estimated_*A.transpose() + Q;

    // update
    Eigen::Vector3d pos_residual;       // pos estimation residual
    pos_residual = pos_measured_ - H * pos_vel_estimated_;
    Eigen::Matrix<double, 3, 3> S;
    S = H*pos_vel_cov_estimated_*H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K;      // the Kalman gain matrix
    K = (pos_vel_cov_estimated_*H.transpose()) * S.inverse();
    // new estimated state
    pos_vel_estimated_ = pos_vel_estimated_ + K*pos_residual;
    // new covariance
    Eigen::Matrix<double, 6, 6> I;
    I.setIdentity();                    // I is an identity matrix
    pos_vel_cov_estimated_ = (I - K*H) * pos_vel_cov_estimated_;


    // perform Kalman Filtering, then for euler angles and their rate
    // noise matrix
    double R_euler = 10E-4;
    R << R_euler, 0, 0,
         0, R_euler, 0,
         0, 0, R_euler;
    double Q_euler = 0.1;
    double Q_rate  = 1.0;
    Q << Q_euler, 0, 0, 0, 0, 0,
         0, Q_euler, 0, 0, 0, 0,
         0, 0, Q_euler, 0, 0, 0,
         0, 0, 0, Q_rate, 0, 0,
         0, 0, 0, 0, Q_rate, 0,
         0, 0, 0, 0, 0, Q_rate;

    // prediction
    euler_rate_estimated_ = A*euler_rate_estimated_;
    euler_rate_cov_estimated_ = A*euler_rate_cov_estimated_*A.transpose() + Q;

    // update
    Eigen::Vector3d euler_residual;       // euler estimation residual
    euler_residual = euler_measured_ - H * euler_rate_estimated_;
    S = H*euler_rate_cov_estimated_*H.transpose() + R;
    K = (euler_rate_cov_estimated_*H.transpose()) * S.inverse();
    // new estimated state
    euler_rate_estimated_ = euler_rate_estimated_ + K*euler_residual;
    // new covariance
    euler_rate_cov_estimated_ = (I - K*H) * euler_rate_cov_estimated_;


//     std::cout << "Roll:" << euler_rate_estimated_(0)*180/3.1415 << std::endl;
//     std::cout << "Pitch:" << euler_rate_estimated_(1)*180/3.1415 << std::endl;
//     std::cout << "Yaw:" << euler_rate_estimated_(2)*180/3.1415 << std::endl << std::endl;


    // prepare published message
    bebop2_msgs::FullStateWithCovarianceStamped msg_pub;        // published bebop2 full state estimation
    msg_pub.header = msg.header;                                // save the header information
    // position
    msg_pub.state.x = pos_vel_estimated_(0);
    msg_pub.state.y = pos_vel_estimated_(1);
    msg_pub.state.z = pos_vel_estimated_(2);
    // velocity
    msg_pub.state.x_dot = pos_vel_estimated_(3);
    msg_pub.state.y_dot = pos_vel_estimated_(4);
    msg_pub.state.z_dot = pos_vel_estimated_(5);
    // euler angles
    msg_pub.state.roll  = euler_rate_estimated_(0);
    msg_pub.state.pitch = euler_rate_estimated_(1);
    msg_pub.state.yaw   = euler_rate_estimated_(2);
    msg_pub.state.roll_dot  = euler_rate_estimated_(3);
    msg_pub.state.pitch_dot = euler_rate_estimated_(4);
    msg_pub.state.yaw_dot   = euler_rate_estimated_(5);
    // position covariance
    msg_pub.state.pos_cov[0] = pos_vel_cov_estimated_(0,0);
    msg_pub.state.pos_cov[1] = pos_vel_cov_estimated_(0,1);
    msg_pub.state.pos_cov[2] = pos_vel_cov_estimated_(0,2);
    msg_pub.state.pos_cov[3] = pos_vel_cov_estimated_(1,0);
    msg_pub.state.pos_cov[4] = pos_vel_cov_estimated_(1,1);
    msg_pub.state.pos_cov[5] = pos_vel_cov_estimated_(1,2);
    msg_pub.state.pos_cov[6] = pos_vel_cov_estimated_(2,0);
    msg_pub.state.pos_cov[7] = pos_vel_cov_estimated_(2,1);
    msg_pub.state.pos_cov[8] = pos_vel_cov_estimated_(2,2);
    // velocity covariance
    msg_pub.state.vel_cov[0] = pos_vel_cov_estimated_(3,3);
    msg_pub.state.vel_cov[1] = pos_vel_cov_estimated_(3,4);
    msg_pub.state.vel_cov[2] = pos_vel_cov_estimated_(3,5);
    msg_pub.state.vel_cov[3] = pos_vel_cov_estimated_(4,3);
    msg_pub.state.vel_cov[4] = pos_vel_cov_estimated_(4,4);
    msg_pub.state.vel_cov[5] = pos_vel_cov_estimated_(4,5);
    msg_pub.state.vel_cov[6] = pos_vel_cov_estimated_(5,3);
    msg_pub.state.vel_cov[7] = pos_vel_cov_estimated_(5,4);
    msg_pub.state.vel_cov[8] = pos_vel_cov_estimated_(5,5);
    // euler angle covariance
    msg_pub.state.euler_cov[0] = euler_rate_cov_estimated_(0,0);
    msg_pub.state.euler_cov[1] = euler_rate_cov_estimated_(0,1);
    msg_pub.state.euler_cov[2] = euler_rate_cov_estimated_(0,2);
    msg_pub.state.euler_cov[3] = euler_rate_cov_estimated_(1,0);
    msg_pub.state.euler_cov[4] = euler_rate_cov_estimated_(1,1);
    msg_pub.state.euler_cov[5] = euler_rate_cov_estimated_(1,2);
    msg_pub.state.euler_cov[6] = euler_rate_cov_estimated_(2,0);
    msg_pub.state.euler_cov[7] = euler_rate_cov_estimated_(2,1);
    msg_pub.state.euler_cov[8] = euler_rate_cov_estimated_(2,2);

    // publish the message
    pub_.publish(msg_pub);

    // set time
    time_stamp_previous_ = time_stamp_;
}

#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from bebop_nmpc_solver import BebopNmpcFormulationParam, bebop_nmpc_casadi_solver


# The frame by default is NWU


class BebopNmpcControl:
    def __init__(self, mpc_form_param):
        # MPC formulation settings
        self.mpc_form_param_ = mpc_form_param

        # bebop param
        self.roll_max_ = self.mpc_form_param_.roll_max
        self.pitch_max_ = self.mpc_form_param_.pitch_max
        self.vz_max_ = self.mpc_form_param_.vz_max
        self.yawrate_max_ = self.mpc_form_param_.yawrate_max
        self.K_yaw_ = self.mpc_form_param_.K_yaw
        self.bebop_size_ = self.mpc_form_param_.bebop_size

        # state and goal pose, size
        self.bebop_state_current_ = np.zeros(9)
        # self.bebop_pose_goal_ = np.array([0, 0, 1.0, 0])
        self.bebop_pose_goal_ = np.array([2.0, 1.0, 1.0, -np.pi])

        # collision avoidance obs param
        self.nobs_ = self.mpc_form_param_.nobs
        self.obs_size_ = self.mpc_form_param_.obs_size
        self.obs_state_current_ = np.array([0, 0, -1.0, 0, 0, 0])
        self.obs_state_prediction_ = np.tile(np.array(self.obs_state_current_), (self.mpc_form_param_.N, 1)).T

        # MPC settings
        self.mpc_dt_ = self.mpc_form_param_.dt
        self.mpc_N_ = self.mpc_form_param_.N
        self.mpc_Tf_ = self.mpc_form_param_.Tf
        self.mpc_nx_ = self.mpc_form_param_.nx
        self.mpc_nu_ = self.mpc_form_param_.nu
        self.mpc_ns_ = self.mpc_form_param_.ns
        self.mpc_np_ = self.mpc_form_param_.nparam
        self.mpc_weights_wp_ = self.mpc_form_param_.mpc_weights_wp
        self.mpc_weights_input_ = self.mpc_form_param_.mpc_weights_input
        self.mpc_weights_coll_ = self.mpc_form_param_.mpc_weights_coll
        self.mpc_weights_slack_ = self.mpc_form_param_.mpc_weights_slack

        # MPC variables
        self.mpc_nlp_traj_ = np.zeros((self.mpc_nu_ + self.mpc_nx_, self.mpc_N_)).reshape(-1)
        self.mpc_nlp_param_ = self.mpc_nx_ + self.mpc_np_ * self.mpc_N_
        self.mpc_x_plan_ = np.zeros((self.mpc_nx_, self.mpc_N_))
        self.mpc_u_plan_ = np.zeros((self.mpc_nu_, self.mpc_N_))
        self.mpc_s_plan_ = np.zeros((self.mpc_ns_, self.mpc_N_))
        self.mpc_u_now_ = np.zeros(self.mpc_nu_)
        self.mpc_feasible_ = False
        self.mpc_success_ = False

        # MPC solver
        [self.nlp_solver_complied_, self.nlp_lbx_, self.nlp_ubx_, self.nlp_lbg_, self.nlp_ubg_] = \
            bebop_nmpc_casadi_solver(self.mpc_form_param_, True)

        # ROS subscriber
        self.odom_sub_ = rospy.Subscriber("/mav_sim_odom", Odometry, self.set_bebop_odom)  # bebop_odom
        self.received_first_odom_ = False
        self.odom_received_time_ = rospy.Time.now()
        self.odom_time_out_ = 0.2

        self.pose_sub_ = rospy.Subscriber("/bebop/pose", PoseStamped, self.set_bebop_pose)
        self.twist_sub_ = rospy.Subscriber("/bebop/twist", TwistStamped, self.set_bebop_twist)

        self.pose_goal_sub_ = rospy.Subscriber("/bebop_pose_goal", PoseStamped, self.set_pose_goal)

        # ROS publisher
        self.bebop_cmd_vel_ = np.array(4)
        self.bebop_cmd_vel_pub_ = rospy.Publisher("/bebop_auto/cmd_vel", Twist, queue_size=1)
        self.mpc_traj_plan_vis_pub_ = rospy.Publisher("/mpc/trajectory_plan_vis", Marker, queue_size=1)

    def set_bebop_odom(self, odom_msg):
        if self.received_first_odom_ is False:
            self.received_first_odom_ = True
            rospy.loginfo('First odometry received!')
        # read data
        self.odom_received_time_ = rospy.Time.now()
        px = odom_msg.pose.pose.position.x
        py = odom_msg.pose.pose.position.y
        pz = odom_msg.pose.pose.position.z
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        vz = odom_msg.twist.twist.linear.z
        rpy = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                                        odom_msg.pose.pose.orientation.y,
                                                        odom_msg.pose.pose.orientation.z,
                                                        odom_msg.pose.pose.orientation.w])
        self.bebop_state_current_ = np.array([px, py, pz, vx, vy, vz, rpy[0], rpy[1], rpy[2]])

    def set_bebop_pose(self, pose_msg):
        if self.received_first_odom_ is False:
            self.received_first_odom_ = True
            rospy.loginfo('First odometry received!')
        self.odom_received_time_ = rospy.Time.now()
        px = pose_msg.pose.position.x
        py = pose_msg.pose.position.y
        pz = pose_msg.pose.position.z
        rpy = tf.transformations.euler_from_quaternion([pose_msg.pose.orientation.x,
                                                        pose_msg.pose.orientation.y,
                                                        pose_msg.pose.orientation.z,
                                                        pose_msg.pose.orientation.w])
        self.bebop_state_current_[0:3] = np.array([px, py, pz])
        self.bebop_state_current_[6:9] = np.array([rpy[0], rpy[1], rpy[2]])

    def set_bebop_twist(self, twist_msg):
        vx = twist_msg.twist.linear.x
        vy = twist_msg.twist.linear.y
        vz = twist_msg.twist.linear.z
        self.bebop_state_current_[3:6] = np.array([vx, vy, vz])

    def set_pose_goal(self, pose_goal_msg):
        px_goal = pose_goal_msg.pose.position.x
        py_goal = pose_goal_msg.pose.position.y
        pz_goal = pose_goal_msg.pose.position.z
        rpy_goal = tf.transformations.euler_from_quaternion([pose_goal_msg.pose.orientation.x,
                                                             pose_goal_msg.pose.orientation.y,
                                                             pose_goal_msg.pose.orientation.z,
                                                             pose_goal_msg.pose.orientation.w])
        self.bebop_pose_goal_ = np.array([px_goal, py_goal, pz_goal, rpy_goal[2]])

    def obs_motion_prediction(self):
        for iStage in range(0, self.mpc_N_):
            self.obs_state_prediction_[0:3] = self.obs_state_current_[0:3] \
                                              + self.obs_state_current_[3:6] * (iStage+1) * self.mpc_dt_

    def reset_nlp_solver(self):
        # initialize plan
        u_reset = np.zeros(self.mpc_nu_)
        x_reset = np.zeros(self.mpc_nx_)
        s_reset = np.zeros(self.mpc_ns_)
        # x_reset = self.bebop_state_current_[:self.mpc_nx_]
        x_reset[0:3] = self.bebop_state_current_[0:3]
        x_reset[6:8] = self.bebop_state_current_[6:8]
        nlp_plan = np.concatenate((u_reset, x_reset, s_reset), axis=0).reshape(-1)
        self.mpc_nlp_traj_ = np.tile(np.array(nlp_plan), self.mpc_N_).reshape(-1)

    def initialize_nlp_solver(self):
        u_traj_init = np.concatenate((self.mpc_u_plan_[:, 1:], self.mpc_u_plan_[:, -1:]), axis=1)
        x_traj_init = np.concatenate((self.mpc_x_plan_[:, 1:], self.mpc_x_plan_[:, -1:]), axis=1)
        s_traj_init = np.concatenate((self.mpc_s_plan_[:, 1:], self.mpc_s_plan_[:, -1:]), axis=1)
        self.mpc_nlp_traj_ = np.vstack((u_traj_init, x_traj_init, s_traj_init)).reshape(-1)

    def set_nlp_params(self):
        parameters_all_stage = np.zeros((self.mpc_np_, self.mpc_N_))  # all parameters on each stage
        for iStage in range(0, self.mpc_N_):
            parameters_all_stage[self.mpc_form_param_.param_index_bebop_pose_start, iStage] = \
                np.array([self.bebop_state_current_[0], self.bebop_state_current_[1], self.bebop_state_current_[2],
                          self.bebop_state_current_[8]])
            parameters_all_stage[self.mpc_form_param_.param_index_bebop_pose_goal, iStage] = self.bebop_pose_goal_
            parameters_all_stage[self.mpc_form_param_.param_index_bebop_size, iStage] = self.bebop_size_
            parameters_all_stage[self.mpc_form_param_.param_index_obs_info, iStage] = np.concatenate((
                self.obs_state_prediction_[0:3, iStage], self.obs_size_
            ))
            if iStage == self.mpc_N_ - 1:  # terminal weights
                parameters_all_stage[self.mpc_form_param_.param_index_mpc_weights, iStage] = np.hstack(
                    (self.mpc_weights_wp_, 0.1 * self.mpc_weights_input_,
                     self.mpc_weights_coll_, self.mpc_weights_slack_)
                )
            else:
                parameters_all_stage[self.mpc_form_param_.param_index_mpc_weights, iStage] = np.hstack(
                    (0.05 * self.mpc_weights_wp_, self.mpc_weights_input_,
                     self.mpc_weights_coll_, self.mpc_weights_slack_)
                )
        # set parameters
        self.mpc_nlp_param_ = np.hstack((self.bebop_state_current_[:self.mpc_nx_],
                                         np.transpose(parameters_all_stage).reshape(-1)))

    def run_nlp_solver(self):
        # initialize solver
        if self.mpc_feasible_ is True:
            self.initialize_nlp_solver()
        else:
            self.reset_nlp_solver()

        # set solver params
        self.set_nlp_params()

        # call the solver
        time_before_solver = rospy.get_rostime()
        nlp_sol = self.nlp_solver_complied_(x0=self.mpc_nlp_traj_,
                                            p=self.mpc_nlp_param_,
                                            lbx=self.nlp_lbx_,
                                            ubx=self.nlp_ubx_,
                                            lbg=self.nlp_lbg_,
                                            ubg=self.nlp_ubg_)

        # deal with infeasibility
        if self.nlp_solver_complied_.stats()['success'] is False:  # if infeasible
            self.mpc_feasible_ = False
            self.mpc_success_ = False
            rospy.logwarn("MPC infeasible!")
        else:
            self.mpc_feasible_ = True
            self.mpc_success_ = True

        solver_time = (rospy.get_rostime() - time_before_solver).to_sec() * 1000.0
        solver_iter = self.nlp_solver_complied_.stats()['iter_count']
        rospy.loginfo('MPC feasible, iter: %d, computation time: %.1f ms.', solver_iter, solver_time)

        # obtain solution
        traj_opt = nlp_sol['x'].reshape((self.mpc_nu_ + self.mpc_nx_ + self.mpc_ns_, self.mpc_N_))
        self.mpc_u_plan_ = np.array(traj_opt[:self.mpc_nu_, :])
        self.mpc_x_plan_ = np.array(traj_opt[self.mpc_nu_:self.mpc_nu_+self.mpc_nx_, :])
        self.mpc_s_plan_ = np.array(traj_opt[self.mpc_nu_+self.mpc_nx_:, :])
        self.mpc_u_now_ = self.mpc_u_plan_[:, 0]

    def calculate_bebop_cmd_vel(self):
        # if odom received
        time_now = rospy.Time.now()
        if (time_now - self.odom_received_time_).to_sec() > self.odom_time_out_:
            rospy.logwarn('Odometry time out! Will try to make the MAV hover.')
            self.bebop_pose_goal_ = np.concatenate((self.bebop_state_current_[0:3], self.bebop_state_current_[8:9]))
        else:
            # run the nlp solver
            self.run_nlp_solver()

        # control commands
        if self.mpc_success_ is True:
            roll_cmd = self.mpc_u_now_[0]
            pitch_cmd = self.mpc_u_now_[1]
            vz_cmd = self.mpc_u_now_[2]
        else:
            rospy.logwarn('MPC failure! Default commands sent.')
            roll_cmd = 0.0
            pitch_cmd = 0.0
            vz_cmd = 0.0

        # yaw control
        yaw_now = self.bebop_state_current_[8]
        yaw_ref = self.bebop_pose_goal_[3]
        yaw_error = yaw_ref - yaw_now
        while np.abs(yaw_error) > np.pi:
            if yaw_error > 0.0:
                yaw_error = yaw_error - 2.0 * np.pi
            else:
                yaw_error = yaw_error + 2.0 * np.pi
        yawrate_cmd = self.K_yaw_ * yaw_error
        yawrate_cmd = np.clip(yawrate_cmd, -self.yawrate_max_, self.yawrate_max_)

        # obtained command
        self.bebop_cmd_vel_ = np.array([roll_cmd, pitch_cmd, vz_cmd, yawrate_cmd])

    def pub_bebop_cmd_vel(self):
        try:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.bebop_cmd_vel_[1] / self.pitch_max_  # pitch to move along x
            cmd_vel_msg.linear.y = -self.bebop_cmd_vel_[0] / self.roll_max_  # roll to move along y
            cmd_vel_msg.linear.z = self.bebop_cmd_vel_[2] / self.vz_max_
            cmd_vel_msg.angular.z = self.bebop_cmd_vel_[3] / self.yawrate_max_
            self.bebop_cmd_vel_pub_.publish(cmd_vel_msg)
        except:
            rospy.logwarn('Bebop cmd_vel command not published!')

    def pub_mpc_traj_plan_vis(self):
        try:
            marker_msg = Marker()
            marker_msg.header.frame_id = "map"
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.type = 8
            marker_msg.action = 0
            # set the scale of the marker
            marker_msg.scale.x = 0.2
            marker_msg.scale.y = 0.2
            marker_msg.scale.z = 0.2
            # set the color
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0
            # Set the pose of the marker
            marker_msg.pose.position.x = 0.0
            marker_msg.pose.position.y = 0.0
            marker_msg.pose.position.z = 0.0
            marker_msg.pose.orientation.x = 0
            marker_msg.pose.orientation.y = 0
            marker_msg.pose.orientation.z = 0
            marker_msg.pose.orientation.w = 1.0
            # points
            mpc_traj_plan_points = []
            for iStage in range(0, self.mpc_N_):
                point = Point(self.mpc_x_plan_[0, iStage], self.mpc_x_plan_[1, iStage], self.mpc_x_plan_[2, iStage])
                mpc_traj_plan_points.append(point)
            marker_msg.points = mpc_traj_plan_points
            self.mpc_traj_plan_vis_pub_.publish(marker_msg)
        except:
            rospy.logwarn("MPC trajectory plan not published!")


def bebop_nmpc_control():
    # create a node
    rospy.loginfo("Starting Bebop NMPC Control...")
    rospy.init_node("bebop_nmpc_control_node", anonymous=False)
    hz = 50
    rate = rospy.Rate(hz)
    rospy.sleep(1.0)

    # formulation
    mpc_form_param = BebopNmpcFormulationParam()

    # control
    bebop_nmpc = BebopNmpcControl(mpc_form_param)

    while not rospy.is_shutdown():
        if bebop_nmpc.received_first_odom_ is False:
            rospy.logwarn('Waiting for first Odometry!')
        else:
            bebop_nmpc.calculate_bebop_cmd_vel()
            bebop_nmpc.pub_bebop_cmd_vel()
            bebop_nmpc.pub_mpc_traj_plan_vis()
        rate.sleep()


if __name__ == "__main__":
    bebop_nmpc_control()

import numpy as np
import casadi as cd


def obj_desired_pos(ego_pos, ego_start_pos, ego_goal_pos, mpc_weights_pos):
    # Compute the goal position progressing cost, normalization is used
    lenToGoal = cd.dot(ego_goal_pos - ego_start_pos, ego_goal_pos - ego_start_pos)
    # length between current start
    # and goal position, using
    #  quadratic form
    lenToGoal = cd.fmax(lenToGoal, 1)  # in case arriving at goal position
    pos_error = ego_goal_pos - ego_pos
    cost = (mpc_weights_pos[0] * pos_error[0] ** 2 + mpc_weights_pos[1] * pos_error[1] ** 2 \
           + mpc_weights_pos[2] * pos_error[2] ** 2) / lenToGoal

    return cost


def obj_input(ego_input, input_max, mpc_weights_input):
    # Compute the control input cost, normalization is used
    ego_input_normalized = cd.vertcat(ego_input[0] / input_max[0],
                                      ego_input[1] / input_max[1],
                                      ego_input[2] / input_max[2])
    cost = mpc_weights_input[0] * ego_input_normalized[0] ** 2 + mpc_weights_input[1] * ego_input_normalized[1] ** 2 \
           + mpc_weights_input[2] * ego_input_normalized[2] ** 2

    return cost


def obj_collision_potential(ego_pos, ego_size, obs_pos, obs_size, mpc_weights_coll):
    # Compute the potential filed based collision avoidance cost
    obs_scale = 1.6
    a = ego_size[0] + obs_scale * obs_size[0]
    b = ego_size[1] + obs_scale * obs_size[1]
    c = ego_size[2] + obs_scale * obs_size[2]

    d_vec = ego_pos - obs_pos
    d = d_vec[0] ** 2 / a ** 2 + d_vec[1] ** 2 / b ** 2 + d_vec[2] ** 2 / c ** 2
    # if else based
    d_c = d - 1.0
    cost = cd.if_else(d_c>0, 0, -d_c)
    # logistic based
    # cost = mpc_weights_coll * 1 / (1 + cd.exp(10 * (d - obs_scale)))

    return cost

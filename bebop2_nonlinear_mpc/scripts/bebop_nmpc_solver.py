import numpy as np
import casadi as cd
import os
from dataclasses import dataclass
import bebop_nmpc_obj


@dataclass
class BebopNmpcFormulationParam:
    # gravity constants
    g = 9.81
    # horizon
    dt = 0.05
    N = 20
    Tf = N * dt
    # stage dims
    nx = 8
    nu = 3
    nobs = 1
    ns = nobs
    # dynamics coefficients
    drag_coefficient_x = 0.2500
    drag_coefficient_y = 0.3300
    roll_time_constant = 0.2368
    roll_gain = 1.1260
    pitch_time_constant = 0.2318
    pitch_gain = 1.1075
    vz_time_constant = 0.3367
    vz_gain = 1.2270
    # state bound
    x_bound = np.array([-5.0, 5.0])
    y_bound = np.array([-2.0, 2.0])
    z_bound = np.array([0.0, 3.0])
    vx_max = 0.5
    vy_max = 0.5
    # control bound
    roll_max = np.deg2rad(15.0)
    pitch_max = np.deg2rad(15.0)
    vz_max = 1.0
    yawrate_max = np.deg2rad(90.0)
    K_yaw = 0.8
    # size
    bebop_size = np.array([0.3, 0.3, 0.4])
    obs_size = np.array([0.4, 0.4, 0.5])
    # mpc weights
    mpc_weights_wp = 8.0 * np.array([1.0, 1.0, 1.0])
    mpc_weights_input = 1.0 * np.array([1.0, 1.0, 1.0])
    mpc_weights_coll = 0.4
    mpc_weights_slack = 1E4
    # params index
    nparam_obs = 6
    nparam = 19 + nparam_obs * nobs
    # bebop info param
    param_index_bebop_pose_start = np.s_[0: 4]  # pos with yaw: 0, 1, 2, 3
    param_index_bebop_pose_goal = np.s_[4: 8]  # pos with yaw: 4, 5, 6, 7
    param_index_bebop_size = np.s_[8: 11]  # ellipse size: 8, 9, 10
    # obs info param
    param_index_obs_info = np.s_[11: 11+nparam_obs*nobs]
    param_index_obs_info_pos = np.s_[0: 3]
    param_index_obs_info_size = np.s_[3: 6]
    # mpc info param
    param_index_mpc_weights = np.s_[param_index_obs_info.stop: param_index_obs_info.stop + 8]
    param_index_mpc_weights_wp = np.s_[0: 3]  # error: px, py, pz
    param_index_mpc_weights_input = np.s_[3: 6]  # input: roll, pitch, vz
    param_index_mpc_weights_coll = np.s_[6: 7]  # collision avoidance
    param_index_mpc_weights_slack = np.s_[7: 8]  # slack


def bebop_nmpc_casadi_shooting_nlp_form(mpc_form_param):
    # Define stage variables
    # control
    roll_cmd = cd.MX.sym('roll_cmd')
    pitch_cmd = cd.MX.sym('pitch_cmd')
    vz_cmd = cd.MX.sym('vz_cmd')
    u = cd.vertcat(roll_cmd, pitch_cmd, vz_cmd)
    # state
    px = cd.MX.sym('px')
    py = cd.MX.sym('py')
    pz = cd.MX.sym('pz')
    vx = cd.MX.sym('vx')
    vy = cd.MX.sym('vy')
    vz = cd.MX.sym('vz')
    roll = cd.MX.sym('roll')
    pitch = cd.MX.sym('pitch')
    x = cd.vertcat(px, py, pz, vx, vy, vz, roll, pitch)
    # slack
    s = cd.MX.sym('s', mpc_form_param.ns)
    # params
    p_stage = cd.MX.sym('p_stage', mpc_form_param.nparam)
    bebop_pose_start = p_stage[mpc_form_param.param_index_bebop_pose_start]
    bebop_pose_goal = p_stage[mpc_form_param.param_index_bebop_pose_goal]
    bebop_size = p_stage[mpc_form_param.param_index_bebop_size]
    obs_info = p_stage[mpc_form_param.param_index_obs_info]
    mpc_weights = p_stage[mpc_form_param.param_index_mpc_weights]
    mpc_weights_wp = mpc_weights[mpc_form_param.param_index_mpc_weights_wp]
    mpc_weights_input = mpc_weights[mpc_form_param.param_index_mpc_weights_input]
    mpc_weights_coll = mpc_weights[mpc_form_param.param_index_mpc_weights_coll]
    mpc_weights_slack = mpc_weights[mpc_form_param.param_index_mpc_weights_slack]

    # Define model dynamics equations
    yaw = bebop_pose_start[3]
    px_dot = vx
    py_dot = vy
    pz_dot = vz
    vx_dot = (np.cos(yaw) * np.tan(pitch) / np.cos(roll) + np.sin(yaw) * np.tan(roll)) * mpc_form_param.g \
             - mpc_form_param.drag_coefficient_x * vx
    # vx_dot = np.tan(pitch) * mpc_form_param.g - mpc_form_param.drag_coefficient_x * vx
    vy_dot = (np.sin(yaw) * np.tan(pitch) / np.cos(roll) - np.cos(yaw) * np.tan(roll)) * mpc_form_param.g \
             - mpc_form_param.drag_coefficient_y * vy
    # vy_dot = - np.tan(roll) * mpc_form_param.g - mpc_form_param.drag_coefficient_y * vy
    vz_dot = (mpc_form_param.vz_gain * vz_cmd - vz) / mpc_form_param.vz_time_constant
    roll_dot = (mpc_form_param.roll_gain * roll_cmd - roll) / mpc_form_param.roll_time_constant
    pitch_dot = (mpc_form_param.pitch_gain * pitch_cmd - pitch) / mpc_form_param.pitch_time_constant
    x_dot = cd.vertcat(px_dot, py_dot, pz_dot, vx_dot, vy_dot, vz_dot, roll_dot, pitch_dot)
    dyn_f = cd.Function('dyn_f', [x, u, p_stage], [x_dot])

    # Discrete dynamics via ERK 4
    dt = mpc_form_param.dt
    X0 = cd.MX.sym('X0', mpc_form_param.nx)
    U = cd.MX.sym('U', mpc_form_param.nu)
    k1 = dyn_f(X0, U, p_stage)
    k2 = dyn_f(X0 + dt / 2 * k1, U, p_stage)
    k3 = dyn_f(X0 + dt / 2 * k2, U, p_stage)
    k4 = dyn_f(X0 + dt * k3, U, p_stage)
    X = X0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    F_ERK = cd.Function('F_ERK', [X0, U, p_stage], [X], ['x0', 'u', 'p'], ['xf'])

    # Define objective function
    # control input cost
    input_max = [mpc_form_param.roll_max, mpc_form_param.pitch_max, mpc_form_param.vz_max]
    cost_input = bebop_nmpc_obj.obj_input(u, input_max, mpc_weights_input)
    obj_input = cd.Function('obj_input', [u, p_stage], [cost_input])
    # waypoint navigation cost
    cost_wp = bebop_nmpc_obj.obj_desired_pos(x[0:3], bebop_pose_start[0:3], bebop_pose_goal[0:3],
                                             mpc_weights_wp[0:3])
    obj_wp = cd.Function('obj_wp', [x, p_stage], [cost_wp])
    # collision avoidance cost
    obs_param = obs_info[0: mpc_form_param.nparam_obs]
    obs_pos = obs_param[mpc_form_param.param_index_obs_info_pos]
    obs_size = obs_param[mpc_form_param.param_index_obs_info_size]
    cost_coll = bebop_nmpc_obj.obj_collision_potential(x[0:3], bebop_size, obs_pos, obs_size, mpc_weights_coll)
    obj_coll = cd.Function('obj_coll', [x, p_stage], [cost_coll])
    # slack cost
    cost_slack = mpc_weights_slack * cd.dot(s, s)
    obj_slack = cd.Function('obj_slack', [s, p_stage], [cost_slack])

    # Define inequality constraints function
    a = bebop_size[0] + 1.05 * obs_size[0]
    b = bebop_size[1] + 1.05 * obs_size[1]
    c = bebop_size[2] + 1.05 * obs_size[2]
    d = x[0:3] - obs_pos
    ego_obs_dis = d[0] ** 2 / a ** 2 + d[1] ** 2 / b ** 2 + d[2] ** 2 / c ** 2 - 1 + s  # ego_obs_dis > 0
    h_ego_obs_dis = cd.Function('h_ego_obs_dis', [x, p_stage, s], [ego_obs_dis])

    # NLP Formulation
    nlp_x = []  # decision variables
    nlp_x0 = []  # initial guess
    nlp_lbx = []  # lower bound on decision variables
    nlp_ubx = []  # upper bound on decision variables
    nlp_J = 0  # cost accumulator
    nlp_g = []  # inequality constraint equations
    nlp_lbg = []  # lower bound for g
    nlp_ubg = []  # upper bound for g
    nlp_p = []  # real time parameters

    # Parameters for initial conditions
    p = cd.MX.sym('P_initial', mpc_form_param.nx)
    nlp_p += [p]

    # Initial conditions
    Xk = p[0: mpc_form_param.nx]

    # Shooting
    for k in range(mpc_form_param.N):
        # Parameters
        Pk = cd.MX.sym('P_' + str(k + 1), mpc_form_param.nparam)  # parameter of this stage
        nlp_p += [Pk]

        # New NLP variables for the control
        Uk = cd.MX.sym('U_' + str(k), mpc_form_param.nu)
        nlp_x += [Uk]
        nlp_lbx += [-mpc_form_param.roll_max, -mpc_form_param.pitch_max, -mpc_form_param.vz_max]
        nlp_ubx += [mpc_form_param.roll_max, mpc_form_param.pitch_max, mpc_form_param.vz_max]
        nlp_x0 += [0] * mpc_form_param.nu

        # Integrate till the end of the interval
        Fk = F_ERK(x0=Xk, u=Uk, p=Pk)
        Xk_end = Fk['xf']

        # New NLP variables for the state
        Xk = cd.MX.sym('X_' + str(k + 1), mpc_form_param.nx)
        nlp_x += [Xk]
        nlp_lbx += [mpc_form_param.x_bound[0], mpc_form_param.y_bound[0], mpc_form_param.z_bound[0],
                    -mpc_form_param.vx_max, -mpc_form_param.vy_max, -mpc_form_param.vz_max,
                    -mpc_form_param.roll_max, -mpc_form_param.pitch_max]
        nlp_ubx += [mpc_form_param.x_bound[1], mpc_form_param.y_bound[1], mpc_form_param.z_bound[1],
                    mpc_form_param.vx_max, mpc_form_param.vy_max, mpc_form_param.vz_max,
                    mpc_form_param.roll_max, mpc_form_param.pitch_max]
        nlp_x0 += [0] * mpc_form_param.nx

        # Add equality constraint
        nlp_g += [Xk_end - Xk]
        nlp_lbg += [0] * mpc_form_param.nx
        nlp_ubg += [0] * mpc_form_param.nx

        # Slack
        Sk = cd.MX.sym('S_' + str(k + 1), mpc_form_param.ns)
        nlp_x += [Sk]  # adding sk as decision variable
        nlp_lbx += [0]
        nlp_ubx += [10.0]
        nlp_x0 += [0] * mpc_form_param.ns

        # Add path constraint
        nlp_g += [h_ego_obs_dis(Xk, Pk, Sk)]
        nlp_lbg += [0]
        nlp_ubg += [cd.inf]

        # Cumulate stage cost
        nlp_J += obj_input(Uk, Pk)
        nlp_J += obj_wp(Xk, Pk)
        nlp_J += obj_coll(Xk, Pk)
        nlp_J += obj_slack(Sk, Pk)

    prob = {'f': nlp_J, 'x': cd.vertcat(*nlp_x), 'g': cd.vertcat(*nlp_g), 'p': cd.vertcat(*nlp_p)}
    opts = {'print_time': 0, 'ipopt.print_level': 0, 'ipopt.max_iter': 100,
            'ipopt.tol': 1E-4}
    nlp_solver = cd.nlpsol('solver', 'ipopt', prob, opts)

    return nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg


def bebop_nmpc_casadi_solver(mpc_form_param, recompile):
    # formulation
    [nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = \
        bebop_nmpc_casadi_shooting_nlp_form(mpc_form_param)

    # compiling
    solver_build_name = 'bebop_nmpc_casadi_nlp_solver'
    solver_build_c = solver_build_name + '.c'
    solver_build_o = solver_build_name + '.so'
    if recompile:
        nlp_solver.generate_dependencies(solver_build_c)
        print('Compiling...')
        os.system('gcc -fPIC -shared ' + solver_build_c + ' -o ' + solver_build_o)
        print('Done Compiling!')
    opts = {'print_time': 0, 'ipopt.print_level': 0, 'ipopt.max_iter': 100,
            'ipopt.tol': 1E-4}
    nlp_solver_compiled = cd.nlpsol('solver', 'ipopt', './' + solver_build_o, opts)

    return nlp_solver_compiled, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg


if __name__ == '__main__':
    mpc_form_param = BebopNmpcFormulationParam()
    bebop_nmpc_casadi_solver(mpc_form_param, True)

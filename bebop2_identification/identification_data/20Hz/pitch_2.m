clear
clc

%% set-up parameters
pitch_max = deg2rad(12);
roll_max  = deg2rad(12);
vz_max    = 1;
yaw_rate_max = deg2rad(120);

%% load bag file
pitch_bag = ros.Bag.load('pitch_2.bag');
input_msg_all = pitch_bag.readAll({'/q4/real/cmd_vel_stamped'});
input_msg = input_msg_all(2:311);
state_msg = pitch_bag.readAll({'/Bebop4/full_state_estimation'});

%% read input message
input_samples = length(input_msg);
input_start_time = input_msg{1}.header.stamp.time;
input = zeros(input_samples, 5);
for i = 1 : input_samples
    input(i, 1) = input_msg{i}.header.stamp.time - input_start_time;         % time
    input(i, 2) = input_msg{i}.twist.linear(1) * pitch_max;                  % pitch
    input(i, 3) = input_msg{i}.twist.linear(2) * (-roll_max);                % roll
    input(i, 4) = input_msg{i}.twist.linear(3) * vz_max;                     % vertical velocity
    input(i, 5) = input_msg{i}.twist.angular(3) * yaw_rate_max;              % yaw rate
end

%% read and select corresponding state
% all messages and time sequences
state_samples = length(state_msg);
state_time_seq = zeros(state_samples, 1);
for i = 1 : state_samples
    state_time_seq(i) = state_msg{i}.header.stamp.time - input_start_time;
end
% input-state data
input_state = zeros(input_samples, 17);
input_state(:, 1) = input(:, 1);
input_state(:, 14:17) = input(:, 2:5);
% state information
for i = 1 : input_samples
    % find the nearest time seq
    time_now = input(i, 1);                     % current time
    time_seq_now = state_time_seq - time_now;   % current time seq
    time_seq_now(time_seq_now < 0) = inf;
    [min_now, index_now] = min(time_seq_now);   % minimum positive number
    % record current state
    % position
    input_state(i, 2) = state_msg{index_now}.state.x;
    input_state(i, 3) = state_msg{index_now}.state.y;
    input_state(i, 4) = state_msg{index_now}.state.z;
    % velocity
    input_state(i, 5) = state_msg{index_now}.state.x_dot;
    input_state(i, 6) = state_msg{index_now}.state.y_dot;
    input_state(i, 7) = state_msg{index_now}.state.z_dot;
    % euler angles
    input_state(i, 8) = state_msg{index_now}.state.pitch;           % pitch
    input_state(i, 9) = state_msg{index_now}.state.roll;            % roll
    input_state(i, 10) = state_msg{index_now}.state.yaw;            % yaw
    input_state(i, 11) = state_msg{index_now}.state.pitch_dot;
    input_state(i, 12) = state_msg{index_now}.state.roll_dot;
    input_state(i, 13) = state_msg{index_now}.state.yaw_dot;
end


%% plot the trajectory
figure;
hold all;
axis([-3 3 -1.5 1.5 0.2 2.5]);
plot3(input_state(:, 2), input_state(:, 3), input_state(:, 4));

%% plot angle
figure;
hold all;
plot(input_state(:, 1), input_state(:, 14).*180/pi, '-r');
plot(input_state(:, 1), input_state(:, 8).*180/pi, '-b');

%% store the data
save('pitch_2.mat', 'input_state');

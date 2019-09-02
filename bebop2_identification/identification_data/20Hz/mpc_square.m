clear
clc

%% set-up parameters
pitch_max = deg2rad(12);
roll_max  = deg2rad(12);
vz_max    = 1;
yaw_rate_max = deg2rad(120);

%% load bag file
pitch_bag = rosbag('mpc_square.bag');
input_bag = select(pitch_bag, 'Topic', '/q4/real/cmd_vel_stamped');
state_bag = select(pitch_bag, 'Topic', '/Bebop4/position_velocity_orientation_estimation');

%% read input message
input_msg = readMessages(input_bag, 200:900);
input_samples = size(input_msg, 1);
input_start_time = input_msg{1}.Header.Stamp.seconds;
input = zeros(input_samples, 5);
for i = 1 : input_samples
    input(i, 1) = input_msg{i}.Header.Stamp.seconds - input_start_time;     % time
    input(i, 2) = input_msg{i}.Twist.Linear.X * pitch_max;                  % pitch
    input(i, 3) = input_msg{i}.Twist.Linear.Y * (-roll_max);                % roll
    input(i, 4) = input_msg{i}.Twist.Linear.Z * vz_max;                     % vertical velocity
    input(i, 5) = input_msg{i}.Twist.Angular.Z * yaw_rate_max;              % yaw rate
end

%% read and select corresponding state
% all messages and time sequences
state_msg = readMessages(state_bag);
state_samples = size(state_msg, 1);
state_time_seq = zeros(state_samples, 1);
for i = 1 : state_samples
    state_time_seq(i) = state_msg{i}.Header.Stamp.seconds - input_start_time;
end
% input-state data
input_state = zeros(input_samples, 14);
input_state(:, 1) = input(:, 1);
input_state(:, 11:14) = input(:, 2:5);
% state information
for i = 1 : input_samples
    % find the nearest time seq
    time_now = input(i, 1);                     % current time
    time_seq_now = state_time_seq - time_now;   % current time seq
    time_seq_now(time_seq_now < 0) = inf;
    [min_now, index_now] = min(time_seq_now);   % minimum positive number
    % record current state
    % position
    input_state(i, 2) = state_msg{index_now}.Pose.Pose.Position.X;
    input_state(i, 3) = state_msg{index_now}.Pose.Pose.Position.Y;
    input_state(i, 4) = state_msg{index_now}.Pose.Pose.Position.Z;
    % velocity
    input_state(i, 5) = state_msg{index_now}.Twist.Twist.Linear.X;
    input_state(i, 6) = state_msg{index_now}.Twist.Twist.Linear.Y;
    input_state(i, 7) = state_msg{index_now}.Twist.Twist.Linear.Z;
    % euler angles
    quat_now = [state_msg{index_now}.Pose.Pose.Orientation.W, ...
                state_msg{index_now}.Pose.Pose.Orientation.X, ...
                state_msg{index_now}.Pose.Pose.Orientation.Y, ...
                state_msg{index_now}.Pose.Pose.Orientation.Z];
    euler_XYZ = quat2eul(quat_now, 'XYZ');      % roll, pitch, yaw
    input_state(i, 8) = euler_XYZ(2);           % pitch
    input_state(i, 9) = euler_XYZ(1);           % roll
    input_state(i, 10) = euler_XYZ(3);          % yaw
end


%% plot the trajectory
figure;
plot3(input_state(:, 2), input_state(:, 3), input_state(:, 4));

%% store the data
save('mpc_square.mat', 'input_state');

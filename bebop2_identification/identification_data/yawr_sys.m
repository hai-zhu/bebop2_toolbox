clear; clc; close all;
%% training 
file_name = "yaw_trn";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Angular.Z * pi / 2]; % 90 deg/s
    timeStamp = double(cmd{i, 1}.Header.Stamp.Sec) + double(cmd{i, 1}.Header.Stamp.Nsec) * 1e-9;
    t = [t;timeStamp];
end

bSel = select(bag,'Topic','/simulator/odometry');
odom = readMessages(bSel,'DataFormat','struct');

% output data
y = [];
u_idx = 1;
for i = 1:numel(odom)
    timeStamp_next = double(odom{i, 1}.Header.Stamp.Sec) + double(odom{i, 1}.Header.Stamp.Nsec) * 1e-9;
    if(timeStamp_next) >= t(u_idx)
        quat = [odom{i-1, 1}.Pose.Pose.Orientation.W,...
                odom{i-1, 1}.Pose.Pose.Orientation.X,...
                odom{i-1, 1}.Pose.Pose.Orientation.Y,...
                odom{i-1, 1}.Pose.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_last = eul(1);
        quat = [odom{i, 1}.Pose.Pose.Orientation.W,...
                odom{i, 1}.Pose.Pose.Orientation.X,...
                odom{i, 1}.Pose.Pose.Orientation.Y,...
                odom{i, 1}.Pose.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_next = eul(1);
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        yawr = (ang_next - ang_last) / (timeStamp_next - timeStamp_last);
        y = [y;yawr];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
y = smooth(y);
Ts = mean(t(2:end) - t(1:end-1));
datatrn = iddata(y,u,Ts);
sys = n4sid(datatrn,2,'InputDelay',3);
sys.K = [0;0];

% sysc = d2c(sys);
% [b,a] = ss2tf(sysc.A,sysc.B,sysc.C,sysc.D);
% systf = tf(b,a);
% % step(systf,5);
% b = [0,b];
% a = [a,0];
% [A,B,C,D] = tf2ss(b,a);
% sysy = ss(A,B,C,D);
% sys = c2d(sysy,Ts);
A = [1,Ts * sys.C;zeros(2,1),sys.A];
B = [0;sys.B];
C = [1,0,0];
D = 0;
sys = ss(A,B,C,D,Ts);
%% testing 
file_name = "yaw_tst";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Angular.Z * pi / 2]; % 90 deg/s
    timeStamp = double(cmd{i, 1}.Header.Stamp.Sec) + double(cmd{i, 1}.Header.Stamp.Nsec) * 1e-9;
    t = [t;timeStamp];
end

bSel = select(bag,'Topic','/simulator/odometry');
odom = readMessages(bSel,'DataFormat','struct');

% output data
y = [];
u_idx = 1;
for i = 1:numel(odom)
    timeStamp_next = double(odom{i, 1}.Header.Stamp.Sec) + double(odom{i, 1}.Header.Stamp.Nsec) * 1e-9;
    if(timeStamp_next) >= t(u_idx)
        quat = [odom{i-1, 1}.Pose.Pose.Orientation.W,...
                odom{i-1, 1}.Pose.Pose.Orientation.X,...
                odom{i-1, 1}.Pose.Pose.Orientation.Y,...
                odom{i-1, 1}.Pose.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_last = eul(1);
        quat = [odom{i, 1}.Pose.Pose.Orientation.W,...
                odom{i, 1}.Pose.Pose.Orientation.X,...
                odom{i, 1}.Pose.Pose.Orientation.Y,...
                odom{i, 1}.Pose.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_next = eul(1);
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        yawr = (ang_next - ang_last) / (timeStamp_next - timeStamp_last);
        y = [y;yawr];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
y = smooth(y);
datatst = iddata(y,u,Ts);
compare(datatst,sys);


% step(systry,5)

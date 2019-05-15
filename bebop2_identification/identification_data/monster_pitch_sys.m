clear; clc; close all;
%% training 
file_name = "monster_pitch_trn";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Linear.X * 10 / 180 * pi]; % 1 -> 10 deg pitch
    timeStamp = double(cmd{i, 1}.Header.Stamp.Sec) + double(cmd{i, 1}.Header.Stamp.Nsec) * 1e-9;
    t = [t;timeStamp];
end

bSel = select(bag,'Topic','/rigid_body_1/pose');
odom = readMessages(bSel,'DataFormat','struct');

% output data
y = [];
u_idx = 1;
for i = 1:numel(odom)
    timeStamp_next = double(odom{i, 1}.Header.Stamp.Sec) + double(odom{i, 1}.Header.Stamp.Nsec) * 1e-9;
    if(timeStamp_next) >= t(u_idx)
        quat = [odom{i-1, 1}.Pose.Orientation.W,...
                odom{i-1, 1}.Pose.Orientation.X,...
                odom{i-1, 1}.Pose.Orientation.Y,...
                odom{i-1, 1}.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_last = eul(2);
        quat = [odom{i, 1}.Pose.Orientation.W,...
                odom{i, 1}.Pose.Orientation.X,...
                odom{i, 1}.Pose.Orientation.Y,...
                odom{i, 1}.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_next = eul(2);
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        ang = ang_last + (t(u_idx) - timeStamp_last) / (timeStamp_next - timeStamp_last) * (ang_next - ang_last);
        y = [y;ang];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
y = y - mean(y);
Ts = mean(t(2:end) - t(1:end-1));
datatrn = iddata(y,u,Ts);
sys = n4sid(datatrn,1,'InputDelay',1);
% sys = ssest(datatrn,2);
%% testing
file_name = "monster_pitch_tst";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Linear.X * 10 / 180 * pi]; % 1 -> 10 deg pitch
    timeStamp = double(cmd{i, 1}.Header.Stamp.Sec) + double(cmd{i, 1}.Header.Stamp.Nsec) * 1e-9;
    t = [t;timeStamp];
end

bSel = select(bag,'Topic','/rigid_body_1/pose');
odom = readMessages(bSel,'DataFormat','struct');

% output data
y = [];
u_idx = 1;
for i = 1:numel(odom)
    timeStamp_next = double(odom{i, 1}.Header.Stamp.Sec) + double(odom{i, 1}.Header.Stamp.Nsec) * 1e-9;
    if(timeStamp_next) >= t(u_idx)
        quat = [odom{i-1, 1}.Pose.Orientation.W,...
                odom{i-1, 1}.Pose.Orientation.X,...
                odom{i-1, 1}.Pose.Orientation.Y,...
                odom{i-1, 1}.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_last = eul(2);
        quat = [odom{i, 1}.Pose.Orientation.W,...
                odom{i, 1}.Pose.Orientation.X,...
                odom{i, 1}.Pose.Orientation.Y,...
                odom{i, 1}.Pose.Orientation.Z];
        eul = quat2eul(quat);
        ang_next = eul(2);
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        ang = ang_last + (t(u_idx) - timeStamp_last) / (timeStamp_next - timeStamp_last) * (ang_next - ang_last);
        y = [y;ang];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
y = y - mean(y);
datatst = iddata(y,u,Ts);
sys.K = [0];
compare(datatst,sys);
sys
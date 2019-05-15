clear; clc; close all;
%% training 
file_name = "monster_vert_tst";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% mass
m = 0.5; % kg

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Linear.Z]; % 1 m/s
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
        alt_last = odom{i-1, 1}.Pose.Position.Z;
        alt_next = odom{i, 1}.Pose.Position.Z;
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        alt = alt_last + (alt_next - alt_last) / (timeStamp_next - timeStamp_last) * (t(u_idx) - timeStamp_last);
        y = [y;alt];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
Ts = mean(t(2:end) - t(1:end-1));
y = diff(y) / Ts;
u = u(2:end);
y = smooth(y,10);
y = y - y(1);
datatrn = iddata(y,u,Ts);
sys = n4sid(datatrn,2,'InputDelay',0);
% C = sys.C;
% sys.C = (C * sys.A - C) / Ts * m;
% sys.D = C * sys.B / Ts * m;
sys.K = [0;0];
%% testing 
file_name = "monster_vert_trn";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Linear.Z]; % 1 m/s
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
        alt_last = odom{i-1, 1}.Pose.Position.Z;
        alt_next = odom{i, 1}.Pose.Position.Z;
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        alt = alt_last + (alt_next - alt_last) / (timeStamp_next - timeStamp_last) * (t(u_idx) - timeStamp_last);
        y = [y;alt];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
% y = gradient(y) / Ts * m;
y = diff(y) / Ts;
y = smooth(y,10);
y = y - y(1);
u = u(2:end);
datatst = iddata(y,u,Ts);
compare(datatst,sys);
% close all;
% sysc = d2c(sys);
% [b,a] = ss2tf(sysc.A,sysc.B,sysc.C,sysc.D);
% bd = [b(2),b(3),0];
% [A,B,C,D] = tf2ss(bd,a);
% % sysc = ss(A,B,C,D);

clear; clc; close all;
%% training 
file_name = "airdrag_y";
bag = rosbag([file_name + ".bag"]);

bSel = select(bag,'Topic','/data_logger/cmd_vel');
cmd = readMessages(bSel,'DataFormat','struct');

% mass
m = 0.5; % kg

% input data
u = [];
t = [];
for i = 1:numel(cmd)
    u = [u;cmd{i, 1}.Twist.Linear.Y]; % 1 -> 10 deg pitch
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
        x_last = odom{i-1, 1}.Pose.Pose.Position.Y;
        x_next = odom{i, 1}.Pose.Pose.Position.Y;
        timeStamp_last = double(odom{i-1, 1}.Header.Stamp.Sec) + double(odom{i-1, 1}.Header.Stamp.Nsec) * 1e-9;
        x_vel = (x_next - x_last) / (timeStamp_next - timeStamp_last);
        quat = [odom{i-1, 1}.Pose.Pose.Orientation.W,...
                odom{i-1, 1}.Pose.Pose.Orientation.X,...
                odom{i-1, 1}.Pose.Pose.Orientation.Y,...
                odom{i-1, 1}.Pose.Pose.Orientation.Z];
        eul = -quat2eul(quat);
        ang_last = eul(3);
        quat = [odom{i, 1}.Pose.Pose.Orientation.W,...
                odom{i, 1}.Pose.Pose.Orientation.X,...
                odom{i, 1}.Pose.Pose.Orientation.Y,...
                odom{i, 1}.Pose.Pose.Orientation.Z];
        eul = -quat2eul(quat);
        ang_next = eul(3);
        ang = ang_last + (t(u_idx) - timeStamp_last) / (timeStamp_next - timeStamp_last) * (ang_next - ang_last);
        y = [y;ang,x_vel];
        u_idx = u_idx + 1;
        if(u_idx) > numel(cmd)
           break; 
        end
    end
end
Ts = mean(t(2:end) - t(1:end-1));
% calculate accerlation
acc = gradient(y(:,2)) / Ts * m;
acc = smooth(acc,5);
% thrust on x direction
thrust_x = tan(y(:,1)) * m * 9.81;
thrust_x = smooth(thrust_x,5);
drag_x = thrust_x - acc;
drag_x = smooth(drag_x,5);
p = polyfit(y(:,2),drag_x,1)
scatter(y(:,2),drag_x,'.'); xlim([-3,3]);ylim([-3,3]); hold on;
v = -3:0.01:3;
d = polyval(p,v);
plot(v,d);
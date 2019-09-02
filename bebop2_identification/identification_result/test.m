clear; clc; close all;

% Continuous-time model
Ts =   1 / 15; % sampling time
m =       0.5; % mass [kg]
g =      9.81; % gravity
Ax =   0.1840; % air drag
Ay =   0.2007; % air drag
va1 =  -2.292; % vertical velocity model
va2 =  -2.550; % vertical velocity model
va3 =   9.742; % vertical velocity model
va4 =  -5.845; % vertical velocity model
vb1 = 0.00642; % vertical velocity model
vb2 = -0.6570; % vertical velocity model
vc1 =  17.650; % vertical velocity model
vc2 = -0.3225; % vertical velocity model
vd =        0; % vertical velocity model
ra =  -6.7529; % roll model
rb =  6.86585; % roll model
pa =  -6.7317; % pitch model
pb =  6.85417; % pitch model
ya1 = -2.9396; % yall model
ya2 = -2.6554; % yall model
ya3 = 13.4409; % yall model
ya4 = -9.5284; % yall model
yb1 =  0.0075; % yall model
yb2 = -0.7522; % yall model
yc1 = 29.0473; % yall model
yc2 = -0.1190; % yall model
yd =   0.0000; % yall model

A = [   0,  0,  0,  1,  0,  0,  0,  0,  0; % x
        0,  0,  0,  0,  1,  0,  0,  0,  0; % y
        0,  0,  0,  0,  0,vc1,vc2,  0,  0; % z
        0,  0,  0,-Ax,  0,  0,  0,  0,  g; % vx
        0,  0,  0,  0,-Ay,  0,  0, -g,  0; % vy
        0,  0,  0,  0,  0,va1,va2,  0,  0; % z1
        0,  0,  0,  0,  0,va3,va4,  0,  0; % z2
        0,  0,  0,  0,  0,  0,  0, ra,  0; % roll
        0,  0,  0,  0,  0,  0,  0,  0, pa];% pitch
    
B = [   0,  0,  0;
        0,  0,  0;
        0,  0, vd;
        0,  0,  0;
        0,  0,  0;
        0,  0,vb1;
        0,  0,vb2;
       rb,  0,  0;
        0, pb,  0];
    
C = [1,  0,  0,  0,  0,  0,  0,  0,  0; % x
     0,  1,  0,  0,  0,  0,  0,  0,  0; % y
     0,  0,  1,  0,  0,  0,  0,  0,  0; % z
     0,  0,  0,  1,  0,  0,  0,  0,  0; % vx
     0,  0,  0,  0,  1,  0,  0,  0,  0; % vy
     0,  0,  0,  0,  0,vc1,vc2,  0,  0; % vz
     0,  0,  0,  0,  0,  0,  0,  1,  0; % roll
     0,  0,  0,  0,  0,  0,  0,  0,  1];% pitch

D = [0,  0,  0;
     0,  0,  0;
     0,  0,  0;
     0,  0,  0;
     0,  0,  0;
     0,  0, vd;
     0,  0,  0;
     0,  0,  0];

sysc = ss(A,B,C,D);

% Discrete-time model 
sysd = c2d(sysc, Ts);

% initial state
states = [0; 0; 1; 0; 0; 0; 0; 0; 0]; % x, y, z, vx, vy, z1, z2, R, P

% MPC setup
MV_R = struct('Min',-0.0556 * pi,'Max',0.0556 * pi);
MV_P = struct('Min',-0.0556 * pi,'Max',0.0556 * pi);
MV_V = struct('Min',-1,'Max',1);
MV = [MV_R, MV_P, MV_V];
W = [1,1,1,0,0,0,0,0];
mpcobj = mpc(sysd,Ts,50,1,[],MV,[],[],[]);
mpcobj.Weight.OutputVariables = W;
options = mpcsimopt(mpcobj);
options.PlantInitialState = states;
refs = [0,10,1,0,0,0,0,0];

% MPC simulation
[~,~,u,~,~,~] = sim(mpcobj,300,refs,options);

% visualization
output = sysd.C * states + sysd.D * [0,0,0]';
for i = 1:size(u,1)

    states = sysd.A * states + sysd.B * u(i,:)';
    output = sysd.C * states + sysd.D * u(i,:)';
    output = [output;0];
    % viz body frame
    plot_3d_obj(output,options.PlantInitialState(1:3),refs(1:3));...
    title({["t="+num2str(i*Ts,'%1.1f')];...
           ["x="+num2str(output(1),'%1.1f')+" y="+num2str(output(2),'%1.1f')+" z="+num2str(output(3),'%1.1f')];...
           ["vx="+num2str(output(4),'%1.1f')+" vy="+num2str(output(5),'%1.1f')+" vz="+num2str(output(6),'%1.1f')];...
           ["R_w="+num2str(rem(output(7)/pi*180+180,360)-180,'%1.1f')+" P_w="+num2str(rem(output(8)/pi*180+180,360)-180,'%1.1f')+" Y_w="+num2str(rem(output(9)/pi*180+180,360)-180,'%1.1f')]});
    % viz path
    pause(0.0)
end
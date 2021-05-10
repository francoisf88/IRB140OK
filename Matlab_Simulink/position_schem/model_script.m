
clc,close all,clear all

%%%% ABB IRB 1200




%%% load parameters 
load('data_robot_DH.mat'); %%% DH parameters  (d,a)
load('data_robot_Dy.mat'); %%% Dynamic paramenters (m,I,r)
%%% LINK ROBOT
L_abb(1) = Link('revolute', 'd', d1, 'a', a1, 'alpha', -pi/2,'offset',0,'m',m1,'I',I1,'r',r1); %% \theta_1
L_abb(2) = Link('revolute', 'd', 0, 'a',a2, 'alpha', 0,'offset',-pi/2,'m',m2,'I',I2,'r',r2); %% \theta_2
L_abb(3) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m3,'I',I3,'r',r3); %% \theta_3
L_abb(4) = Link('revolute', 'd', d4, 'a',0, 'alpha', pi/2,'offset',0,'m',m4,'I',I4,'r',r4); %% \theta_4
L_abb(5) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m5,'I',I5,'r',r5); %% \theta_5
L_abb(6) = Link('revolute', 'd', d6, 'a',0, 'alpha', 0,'offset',-pi/2,'m',m6,'I',I6,'r',r6); %% \theta_6

%%% joint limit %%%% (MANDATORY FOR PRISMATIC JOINT)

L_abb(1).qlim = [-180 180]*pi/180;
L_abb(2).qlim = [-100 100]*pi/180;
L_abb(3).qlim = [-140 140]*pi/180;
L_abb(4).qlim = [-200 200]*pi/180;
L_abb(5).qlim = [-115 115]*pi/180;
L_abb(6).qlim = [-400 400]*pi/180;



%%% Create a SerialLink robot object
Robot = SerialLink(L_abb,'name','IRB_140');


%%%%% Define robot position and velocity

q = [0 0 0 0 0 0];
dot_q = [0 0 0 0 0 0];

%%% display graphical representation of robot
Robot.plot(q);

%%% drive the graphical robot
Robot.teach


%%%% Robot Kinematic Model 

%%%% Forward kinematics
T = Robot.fkine(q);
pos = T.t;
x = pos(1)
y = pos(2)
z = pos(3)
rpy = tr2rpy(T)

%%%% Geometric Jacobian
J = Robot.jacob0(q);
%%%% Analytical Jacobian (eul angle rapp -ZYZ) three first line == above,
%%%% change in the last 3 lines
Ja = Robot.jacob0(q,'eul'); 
%%%% Analytical Jacobian (rpy angle rapp)
Ja = Robot.jacob0(q,'rpy'); 

%%% Trajectory 

q0 = q; %inital position
qf = [pi/6 0 pi/3 pi/4 0 0]; %final position

Ts = 1e-1; % Samplig Time 
Tf = 10; % Duration trajectory (s)
tt = [0:Ts:10]; %% time vector

[pos,vel,acc] = jtraj(q0,qf,tt);%%% trajectory

%%% plot trajectory 

Robot.plot(pos);

figure(1)
plot(tt,pos); grid on;xlabel('Time[s]'); ylabel('Position[rad]'); %% plot position
legend('q_1','q_2','q_3','q_4','q_5','q_6') 
figure(2)
plot(tt,vel); grid on;xlabel('Time[s]'); ylabel('Velocity[rad/s]'); %% plot velocity
legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','Interpreter', 'latex') 
figure(3)
plot(tt,acc); grid on; xlabel('Time[s]'); ylabel('Acceleration[rad/s{^2}]');%plot acceleration
legend('$\ddot{q}_1$','$\ddot{q}_2$','$\ddot{q}_3$','$\ddot{q}_4$','$\ddot{q}_5$','$\ddot{q}_6$','Interpreter', 'latex') 



%%%%% Robot Dynamic Model %%%%

%%%%% Inertia Matrix
B = Robot.inertia(q);
%%%%% Coriolis/Centripetal Matrix
C = Robot.coriolis(q,dot_q);
%%%% Gravity Vector
g_vect = Robot.gravload(q);







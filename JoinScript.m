%% Defining the robot structure

%%% load parameters, from the robot datasheet
load('data_robot_DH.mat'); %%% DH parameters  (d,a)
load('data_robot_Dy.mat'); %%% Dynamic paramenters (m,I,r)

%%% LINK ROBOT.
L_abb(1) = Link('revolute', 'd', d1, 'a', a1, 'alpha', -pi/2,'offset',0,'m',m1,'I',I1,'r',r1); %% \theta_1
L_abb(2) = Link('revolute', 'd', 0, 'a',a2, 'alpha', 0,'offset',-pi/2,'m',m2,'I',I2,'r',r2); %% \theta_2
L_abb(3) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m3,'I',I3,'r',r3); %% \theta_3
L_abb(4) = Link('revolute', 'd', d4, 'a',0, 'alpha', pi/2,'offset',0,'m',m4,'I',I4,'r',r4); %% \theta_4
L_abb(5) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m5,'I',I5,'r',r5); %% \theta_5
L_abb(6) = Link('revolute', 'd', 0, 'a',0, 'alpha', 0,'offset',0,'m',m6,'I',I6,'r',r6); %% \theta_6

%----> it is needed to change the files data_robot_DH and data_robot_Dy according to the datasheet

%%% joint limit %%%% (MANDATORY FOR PRISMATIC JOINT)
L_abb(1).qlim = [-180 180]*pi/180;
L_abb(2).qlim = [-90 110]*pi/180;
L_abb(3).qlim = [-230 50]*pi/180;
L_abb(4).qlim = [-200 200]*pi/180;
L_abb(5).qlim = [-115 115]*pi/180;
L_abb(6).qlim = [-400 400]*pi/180;

%%% Create a SerialLink robot object
MyRobot = SerialLink(L_abb,'name','abb');

%%%%% Define robot position and velocity. This is the initial configuration
q = [0 0 0 0 0 0];
dot_q = [0 0 0 0 0 0];

%%% display graphical representation of robot: This fucntion need the
%%% configuration of my robot
MyRobot.plot(q);

%%% drive the graphical robot
MyRobot.teach

%%  Direct kinematics. Preliminar Analysis

T = MyRobot.fkine(q);  

%%% ----> to do
%T_analytical = T_Analytical() %% with the simbolic toolbox you obtain the analytical version of the T 

pos = T.t;
x = pos(1);
y = pos(2);
z = pos(3);

% To convert the homogenous transform to rotational matrix (gives me the orientation of the end effector) and translation (the position) 
[R,t] = tr2rt(T);
% From the T matrix gives the orientation in Euler angles
eu = tr2eul(T);
% The orientation matrix in Roll-Pitch-yaw
rpy = tr2rpy(T);

%%%% Geometric Jacobian (needed for the book page 154)
J = MyRobot.jacob0(q);
%%%% Analytixal Jacobian (eul angle rapp -ZYZ)
Ja = MyRobot.jacob0(q,'eul'); 
%%%% Analytixal Jacobian (rpy angle rapp)
Ja = MyRobot.jacob0(q,'rpy'); 


%% Robot Dynamic Model

%%%%% Inertia Matrix
B = MyRobot.inertia(q);
%%%%% Coriolis/Centripetal Matrix
C = MyRobot.coriolis(q,dot_q);
%%%% Gravity Vector
g_vect = MyRobot.gravload(q);


%% Trajectory. Analyse the Dynamic part

%%%Task space position for trajectory
S0 = SE3(0,0.2,0); %inital position
Sf = SE3(1.4,0.2,0); %final position

Ts = 1e-1; % Samplig Time 0.1 sec
Tf = 1; % Duration trajectory (s)
tt = [0:Ts:1]; %% time vector

%[pos,vel,acc] = jtraj(S0,Sf,tt);%%% trajectory both in the joint space

% ctraj function: Cartesian trajectory between two poses
traje = ctraj(S0,Sf,tt); %% Trajectory in the task space

qs=MyRobot.ikine6s(traje)
%%% Plot trajectory. Just visualization, no control
%MyRobot.plot(pos);

% % Note that the trajectory needs to be polynomial 
% figure(1)
% plot(tt,pos); grid on;xlabel('Time[s]'); ylabel('Position[rad]'); %% plot position
% legend('q_1','q_2','q_3','q_4','q_5','q_6') 
% title('Position')
% % the velocity is desirable with a bell shape
% figure(2)
% plot(tt,vel); grid on;xlabel('Time[s]'); ylabel('Velocity[rad/s]'); %% plot velocity
% legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','Interpreter', 'latex')
% title('Velocity')
% figure(3)
% plot(tt,acc); grid on; xlabel('Time[s]'); ylabel('Acceleration[rad/s{^2}]');%plot acceleration
% legend('$\ddot{q}_1$','$\ddot{q}_2$','$\ddot{q}_3$','$\ddot{q}_4$','$\ddot{q}_5$','$\ddot{q}_6$','Interpreter', 'latex') 
% title('Acceleration')

%% A first trajectory in joint space

%%%Task space position for trajectory
q0 = [0 0 0 0 0 0 ]; %inital position
qf = [pi/2 pi/3 pi/12 pi/2 0 pi/2]; %final position

Ts = 1e-1; % Samplig Time 0.1 sec
Tf = 10; % Duration trajectory (s)
tt = [0:Ts:10]; %% time vector

[pos,vel,acc] = jtraj(q0,qf,tt);%%% trajectory both in the joint space
MyRobot.plot(pos);

%% A second trajectory in task space

%Getting the cartesian coordinates of the end point
T = MyRobot.fkine(qf);

x0 = T.t;
xf = [x0(1)-2 x0(2) x0(3)] %A straight line 
S0 = SE3(x0); %inital position
Sf = SE3(xf); %final position

Ts = 1e-1; % Samplig Time 0.1 sec
Tf = 1; % Duration trajectory (s)
tt = [0:Ts:Tf]; %% time vector

SS = ctraj(S0,Sf,tt);
%From now, we need inverse kinematics to have 







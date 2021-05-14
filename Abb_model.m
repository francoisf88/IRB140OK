
function Abb = Abb_model()

%%% load parameters 
load('data_robot_DH.mat'); %%% DH parameters  (d,a)
load('data_robot_Dy.mat'); %%% Dynamic paramenters (m,I,r)
%%% LINK ROBOT
L_abb(1) = Link('revolute', 'd', d1, 'a', a1, 'alpha', -pi/2,'offset',0,'m',m1,'I',I1,'r',r1); %% \theta_1
L_abb(2) = Link('revolute', 'd', 0, 'a',a2, 'alpha', 0,'offset',-pi/2,'m',m2,'I',I2,'r',r2); %% \theta_2
L_abb(3) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m3,'I',I3,'r',r3); %% \theta_3
L_abb(4) = Link('revolute', 'd', d4, 'a',0, 'alpha', pi/2,'offset',0,'m',m4,'I',I4,'r',r4); %% \theta_4
L_abb(5) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',0,'m',m5,'I',I5,'r',r5); %% \theta_5
L_abb(6) = Link('revolute', 'd', 0, 'a',0, 'alpha', 0,'offset',0,'m',m6,'I',I6,'r',r6); %% \theta_6

%%% joint limit %%%% (MANDATORY FOR PRISMATIC JOINT)

L_abb(1).qlim = [-180 180]*pi/180;
L_abb(2).qlim = [-100 100]*pi/180;
L_abb(3).qlim = [-140 140]*pi/180;
L_abb(4).qlim = [-200 200]*pi/180;
L_abb(5).qlim = [-115 115]*pi/180;
L_abb(6).qlim = [-400 400]*pi/180;


Abb = SerialLink(L_abb,'name','abb');

end
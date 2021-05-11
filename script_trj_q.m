
clc,close all,clear all

%%%% script trajectory %%%5

q0 = [0 0 0 0 0 0]*pi/180;
q0f = [30 37 5 80 18 -37]*pi/180;

Ts = 1e-3;
tt = [0:Ts:30];

[posd,veld,accd] = jtraj(q0,q0f,tt);

%%%% script trajectory polynomial %%%

%We take the s0 in q0
Abb = Abb_model(); % call function robot model 
T = Abb.fkine(q0);
s0 = T.t;
sf = [12 100 40];


[xd,dxd,ddxd] = tpoly(s0.',sf,tt.')

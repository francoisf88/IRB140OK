
clc,close all,clear all

%%%% script trajectory %%%5

q0 = [0 0 0 0 0 0]*pi/180;
q0f = [30 37 5 80 18 -37]*pi/180;

Ts = 1e-3;
tt = [0:Ts:30];

[posd,veld,accd] = jtraj(q0,q0f,tt);
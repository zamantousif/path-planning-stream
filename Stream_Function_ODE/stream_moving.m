% Vehicle path planning using stream function for avoiding stationary
% and moving obstacles using Circle Theorem as described in the below 
% reference.
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu
clc;
clear;
start = [8;9];
A=0;
bx = 3;
by = 3;
a = 2;
center = [bx by];
radius = a;
tf=100;
nofigure=0;
[T,X] = ode23(@(t,x) stream_moving_ode(t,x,A),[0 tf],start,A);
figure('Name','Stream Flow');
plot(X(:,1), X(:,2),'r-');
hold on



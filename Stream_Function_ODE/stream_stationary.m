% Vehicle path planning using stream function for avoiding stationary
% obstacles using Circle Theorem as described in the below reference.
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

clc;
clear;
close all;

start = [5;5];% initial condition

% A=0; 

bx_i = 3;
by_i = 3;
a_i = 1;

center = [bx_i by_i];
radius = a_i;

tstart = 0;
tfinal = 80;
tdelta = 0.1;

tspan = linspace(tstart,tfinal,tdelta);
nofigure=0;

[T,X] = ode23(@stream_stationary_ode, tspan, start);

% figure('Name','Stream Flow');
% viscircles(center, radius);
% hold on;
% plot(X(:,1), X(:,2),'g-');
% hold on;
% xlabel('x');
% ylabel('y');
% legend('Trajectory of robot')
% hold off;




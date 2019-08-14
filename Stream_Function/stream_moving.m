% Vehicle path planning using stream function for avoiding stationary
% and moving obstacles using Circle Theorem as described in the below 
% reference.
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

clc;
clear;

% coordinates of the center and radius of the circular obstacle
bx = 4;
by = 6;
a = 2;
center = [bx by];
radius = a;
% velocity of the moving obstacle
Vx = 2;
Vy = 1;
% strength of the Sink and Vortex
C = 22;

% build a grid from start to goal
start = [0,0];
goal = [10,10]; % input N for a NxN grid
[x,y] = meshgrid(start(1):1:goal(1),start(2):1:goal(2));

% create stream function with stationary and moving obstacle
[U, V, psi] = stream_moving_obstacle(a, bx, by, Vx, Vy, C);

% iterate over the grid points and evaluate the velocity components of the
% stream function at each grid point
for X=0:1:10
    for Y=0:1:10
        Vref=0.15;
        U1= eval(U);
        V1= eval(V);
        theta = atan2(V1,U1);
        u = Vref*cos(theta);
        v = Vref*sin(theta);
        Umat(Y+1,X+1) = u;
        Vmat(Y+1,X+1) = v;
        PSI1(Y+1,X+1) = eval(psi);
    end
end

% plot the stream flow around the obstacle
figure;
viscircles(center, radius);
hold on;
quiver(x,y,Umat,Vmat)
hold on;

% get potential field and stream function from velocity components
% [phi,psi] = flowfun(U,V);

% plot the streamlines around the obstacle
contour(x,y,PSI1,15,'g');

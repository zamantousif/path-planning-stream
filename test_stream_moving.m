% Vehicle path planning using stream function for avoiding stationary
% and moving obstacles using Circle Theorem as described in the below 
% reference.
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

clc;
clearvars;
close all;

% coordinates of the center and radius of the circular obstacle
bx_init = 1;
by_init = 7;
bx_final = 8;
by_final = 1;

bx_ = linspace(bx_init,bx_final,100);
by_ = by_init + ((by_final-by_init)/(bx_final-bx_init))*(bx_-bx_init);

% radius
a = 1;

center_init = [bx_init by_init];
center_final = [bx_final by_final];

len_center = length(bx_);

radius = a;
% velocity of the moving obstacle
Vx = 2;
Vy = 1;
% strength of the Sink and Vortex
C = 20;

% build a grid from start to goal
start = [0,0];
goal = [10,10]; % input N for a NxN grid
[x,y] = meshgrid(start(1):1:goal(1),start(2):1:goal(2));

% plot the stream flow around the obstacle
figure;
% pause duration in seconds
duration = 0.001;

for i=1:len_center
center(i,:) = [bx_(i) by_(i)];
bx = bx_(i);
by = by_(i);
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

% plot the obstacle
vch = viscircles(center(i,:), radius);
hold on;
% plot the gradients of the stream function
qh = quiver(x,y,Umat,Vmat);
qh.Color = 'black';
hold on;
% plot the streamlines around the obstacle
% cph = contour(x,y,PSI1,15,'g');
pause(0.01);

if i ~= len_center
delete(vch);
delete(qh);
% delete(cph);
end

end

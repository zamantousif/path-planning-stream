function [bx, by, Vx, Vy] = get_obstacle(t)
% Implementation of the stream function for avoiding stationary obstacles
% using Circle Theorem
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

bx_i= 5;
by_i= 10;
Vx= 0;
Vy= -0.10;
bx= bx_i + Vx*t
by= by_i + Vy*t
end

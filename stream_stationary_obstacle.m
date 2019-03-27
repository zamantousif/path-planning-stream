function [u, v, psi] = stream_stationary_obstacle(bx, by, a)
% Implementation of the stream function for avoiding stationary obstacles
% using Circle Theorem
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

syms bx by X Y a

% dimension of the stationary obstacle
% bx = 5;
% by = 3;
% a = 2;

% strength of the sink and the vortex
C = 1;

% Y1 and X1 - considering circular shaped obstacle
Y1 = (((a^2)*(Y-by))/(((X-bx)^2)+((Y-by)^2)))+by;
X1 = (((a^2)*(X-bx))/(((X-bx)^2)+((Y-by)^2)))+bx;

% stream function from imaginary component of complex potential
psi = -C*(atan2(Y,X)) + C*(atan2(Y1,X1));

% velocity components
u = diff(psi,Y);
v = -1*diff(psi,X);
end

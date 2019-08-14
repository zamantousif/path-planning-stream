function [u, v, psi] = stream_obstacle(a, bx, by, Vx, Vy, C)
% Implementation of the stream function for avoiding single stationary and 
% a single moving obstacle using Circle Theorem
% Reference: https://doi.org/10.1109/ROBOT.2003.1241966
% Authors: Mitesh Agrawal and Mohammed Tousif Zaman
% Contact: msagrawal[at]wpi[dot]edu and mzaman[at]wpi[dot]edu

clc;
clearvars;

syms bx by X Y a Vx Vy C real

% C = 12;
% Vx = 2;
% Vy = 3;

% b = bx + i*by defines the velocity of the moving obstacle
b = bx + by*1i;
% b_ is the conjugate of b
b_ = conj(b);
% complex variables for defining the potential field
z = X + Y*1i;
Z = ((a^2/(z-b)) + b_);

% f = -C*log(z);
% f_ = -C*log(Z);
% w_stationary = f + f_;

% considering velocity of moving obstacle for the potential field
g = -Vx*z + Vy*z*1i;
g_ = -Vx*Z - Vy*Z*1i;
w_moving = g + g_;

% alternate calculation
% w = w_stationary + w_moving;
% psi = imag(w);

% stream function from imaginary component of complex potential for
% avoiding a single moving obstacle
psi2 = imag(w_moving);

% definition from single obstacle avoidance case
% Y1 and X1 - considering single circular shaped obstacle
Y1 = (((a^2)*(Y-by))/(((X-bx)^2)+((Y-by)^2)))+by;
X1 = (((a^2)*(X-bx))/(((X-bx)^2)+((Y-by)^2)))+bx;

% stream function from imaginary component of complex potential for
% avoiding a single stationary obstacle
psi1 = -C*(atan2(Y,X)) + C*(atan2(Y1,X1));

% stream function from imaginary component of the total complex potential
% considering both stationary and moving obstacles
psi = psi1 + psi2;

% velocity components of the stream function
u = diff(psi,Y);
v = -1*diff(psi,X);

end
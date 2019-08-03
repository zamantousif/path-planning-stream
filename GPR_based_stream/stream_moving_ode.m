function [ dx ] = stream_moving_ode(t,x,obstaclepred)
dx = zeros(2,1);

global i
global T

if T+0.1<= t <T+0.2
    i=2;
elseif T+0.2<= t <T+0.3
    i=3;
elseif T+0.3<= t <T+0.4
    i=4;
elseif T+0.4<= t <T+0.5
    i=5;
elseif T+0.5<= t <T+0.6
    i=6;
else 
    i=7;
end

obs_pose = obstaclepred(i,:);
bx= obs_pose(1);
by= obs_pose(2);
Vx= obs_pose(3);
Vy= obs_pose(4);
a = 2;
C = 12* sqrt(Vx*Vx + Vy*Vy);
[U, V, psi] = stream_moving_obstacle();
X= x(1);
Y= x(2);
Vref=0.15;
U1= eval(U);
V1= eval(V);
theta = atan2(V1,U1);
u = Vref*cos(theta);
v = Vref*sin(theta);
dx(1) = u;
dx(2) = v;
end
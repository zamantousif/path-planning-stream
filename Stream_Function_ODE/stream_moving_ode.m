function [ dx ] = stream_moving_ode(t,x,A)
dx = zeros(2,1);
[bx, by, Vx, Vy] = get_obstacle(t)
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
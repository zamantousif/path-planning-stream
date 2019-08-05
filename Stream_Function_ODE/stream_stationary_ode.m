function dxdt = stream_stationary_ode(t,x)
dxdt = zeros(size(x));
bx = 3;
by = 3;
a = 1;
C = 1;
[U, V, psi] = stream_stationary_obstacle();
X= x(1);
Y= x(2);
Vref=0.15;
U1= eval(U);
V1= eval(V);
theta = atan2(V1,U1);
u = Vref*cos(theta);
v = Vref*sin(theta);
dxdt(1) = u;
dxdt(2) = v;
end


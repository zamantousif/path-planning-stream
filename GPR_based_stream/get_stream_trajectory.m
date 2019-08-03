function [ traj ] = get_stream_trajectory(tstep,z,obstaclepred)

traj= [z;0;0];
xnew= z(1);
ynew= z(2);
for i= 1:1:7
    obs_pose = obstaclepred(i,:);
    bx= obs_pose(1);
    by= obs_pose(2);
    Vx= obs_pose(3);
    Vy= obs_pose(4);
    a = 2;
    C = 12* sqrt(Vx*Vx + Vy*Vy);
    [U, V, psi, U_dot, V_dot] = stream_moving_obstacle();
    X= xnew;
    Y= ynew;
%     Vref=0.015;
    U1= eval(U);
    V1= eval(V);
    U1_dot= eval(U_dot);
    V1_dot= eval(V_dot);
    theta = atan2(V1,U1);
%     u = Vref*cos(theta);
%     v = Vref*sin(theta);
    xnew= xnew + U1*tstep;
    ynew= ynew + V1*tstep;
    vstep= U1*cos(theta)+V1*sin(theta);
    traj(4,i)= vstep;
    traj(5,i)= (theta-traj(3,i))/tstep;
    znew= [xnew;ynew;theta;0;0];
    traj= [traj,znew];
end




end
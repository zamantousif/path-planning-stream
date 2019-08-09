function [ traj ] = get_stream_trajectory(tstep,z,obstacle_type,obstaclepred)

traj= [z;0;0];
xnew= z(1);
ynew= z(2);
% planning for the horizon

%% Run the horizon
for i= 1:1:2
    obs_pose = obstaclepred(i,:);
    bx= obs_pose(1);
    by= obs_pose(2);
    Vx= obs_pose(3);
    Vy= obs_pose(4);
    % equations from paper
    [U, V, psi, U_dot, V_dot] = stream_moving_obstacle();
    X= xnew;
    Y= ynew;

    % Vref=0.015;

    %% Initialization for stream function
    a = 2;
    % Set the source/sink strength based on obstacle type
    if obstacle_type == 0
        % Stationary obstacles
        C = 22;
    elseif obstacle_type == 1
        % Moving obstacles
        C = 12 * sqrt(Vx*Vx + Vy*Vy);
    end

    % store the robot states using the stream function with
    % obstacle states given from prediction model
    U1= eval(U);
    V1= eval(V);
    U1_dot= eval(U_dot);
    V1_dot= eval(V_dot);
    theta = atan2(V1,U1);
%     u = Vref*cos(theta);
%     v = Vref*sin(theta);
    xnew= xnew + U1*tstep;
    ynew= ynew + V1*tstep;
    % linear velocity
    vstep= U1*cos(theta)+V1*sin(theta);
    traj(4,i)= vstep;
    % angular velocity = dOmega = (current_step_theta - last_step_theta)/dt
    traj(5,i)= (theta-traj(3,i))/tstep;
    
    % store the new predicted robot states of the horizon
    znew= [xnew;ynew;theta;0;0];
    traj= [traj,znew];
end

end
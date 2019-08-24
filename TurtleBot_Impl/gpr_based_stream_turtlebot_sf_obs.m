%% DO THIS BEFORE RUNNING THIS SCRIPT
% Open a terminal on TurtleBot and run the below command to launch VICON 
% at the IP address specified (cross-check the IP of the system running 
% VICON)

% roslaunch vicon_bridge vicon.launch ip:=130.215.206.243

%% Initialize
clc;
clear;
close all;

%% Select ROS Connectivity
% true : Enable ROS features
% false : Disable ROS features
enable_ros = true;

%% Terminate previous ROS connectivity, if any
if enable_ros == true
    terminate_ros();
end

%% Set IP address, initialize ROS and connect to desired ROS nodes
if enable_ros == true
    % create a ROS MASTER
    % master = robotics.ros.Core; % use if master_uri is same as ros_ip
%     ROS_IP = '130.215.206.232';
%     ROS_MASTER_URI = 'http://130.215.223.68:11311'; % URI of TurtleBot
    % Set ROS environment variables and ROS MASTER at specified IP address
%     initialize_ros(ROS_IP, ROS_MASTER_URI);

    % Set ROS_IP and ROS_MASTER_URI
    setenv('ROS_IP','130.215.206.232')
    setenv('ROS_MASTER_URI','http://130.215.171.227:11311')

    % Initialize ROS and connect to TurtleBot
    ip_turtlebot = '130.215.171.227';
    rosinit(ip_turtlebot)
    
end

%% Select obstacle type 
% 0 : Stationary Obstacles 
% 1 : Moving Obstacles
obstacle_type = 0;

%% Select state feedback type
% 0 : Fixed positions for both robot and obstacle
% 1 : VICON motion capture system gives feedback for both robot and
% obstacle
% 2 : VICON motion capture system gives feedback for the obstacle
%% Hard coded for testing
state_fb = 1;

% if enable_ros == true
%     state_fb = 1;
%     if obs_fb_alone == true
%         state_fb = 2;
% else
%     state_fb = 0;
% end

if state_fb == 0    
    % Set initial robot state
    x0= 4;
    y0= 4;
    theta0= 0;
    % Set initial obstacle state
    bx0= 1.1;
    by0= 1.2;
elseif state_fb == 1
    % Subscribe to the VICON topic to get TurtleBot state feedback
    turtlebot_sub = rossubscriber('/vicon/turtlebot_traj_track/turtlebot_traj_track1');
    
    %% Provide the initial/starting state of robot
    % Get x0 y0 theta0 of TurtleBot
    turtlebot_pose_data = receive(turtlebot_sub,3); % timeout of 3s
    x0 = turtlebot_pose_data.Transform.Translation.X;
    y0 = turtlebot_pose_data.Transform.Translation.Y;

    quatW = turtlebot_pose_data.Transform.Rotation.W;
    quatX = turtlebot_pose_data.Transform.Rotation.X;
    quatY = turtlebot_pose_data.Transform.Rotation.Y;
    quatZ = turtlebot_pose_data.Transform.Rotation.Z;

    angles = quat2eul([quatW quatX quatY quatZ]); % Euler ZYX
%     theta0 = rad2deg(angles(1));
    theta0 = angles(1);
    
    % Subscribe to the VICON topic to get Stationary Obstacle state feedback
    obstacle_sub = rossubscriber('/vicon/stationary_obs_track/stationary_obs_track');
    
    %% Provide the initial/starting state of obstacle
    % Get bx0 by0 of Obstacle
    obstacle_pose_data = receive(obstacle_sub,3); % timeout of 3s
    bx0 = obstacle_pose_data.Transform.Translation.X;
    by0 = obstacle_pose_data.Transform.Translation.Y;
    
elseif state_fb == 2
    % Subscribe to the VICON topic to get Stationary Obstacle state feedback
    obstacle_sub = rossubscriber('/vicon/stationary_obs_track/stationary_obs_track');
    
    %% Provide the initial/starting state of obstacle
    % Get bx0 by0 of Obstacle
    obstacle_pose_data = receive(obstacle_sub,3); % timeout of 3s
    bx0 = obstacle_pose_data.Transform.Translation.X;
    by0 = obstacle_pose_data.Transform.Translation.Y;
    
    
    % Set initial robot state to fixed values
    x0= 4;
    y0= 4;
    theta0= 0;
end

if obstacle_type == 0
    % Stationary obstacles have zero velocity
    vx0 = 0;
    vy0 = 0;
elseif obstacle_type == 1
    % Moving obstacles have non-zero velocity    
    vx0= 1.5;
    vy0= -0.55;
end

%% GPR block
% past obstacle trajectory
bXtrain= [bx0];
bYtrain= [by0];
VXtrain= [vx0];
VYtrain= [vy0];
% initialize time simulation from -tstep
Xtrain= [-0.10];
%% added one line below
% obstaclepred=[bx0,by0,vx0,vy0];
% store the robot state
zinit= [x0;y0;theta0];
% store the robot state with MPC
zmpc= [x0;y0;theta0];

%% Update radii of the robot and obstacle
if enable_ros == true
    % Physical dimensions of TurtleBot and obstacle
    radius_robot = 0.177;   % diameter = 354mm
    radius_obs = 0.109;    % diameter = 218mm
else
    % Dummy dimensions for simulation plots
%     radius_robot = 0.3;
%     radius_obs = 1.4;

    % Physical dimensions of TurtleBot and obstacle
    radius_robot = 0.177;   % diameter = 354mm
    radius_obs = 0.109;    % diameter = 218mm

end

%% MPC block

% controller params
N = 7; % length of window
Q = [1;1;1];
R = [1;1];
% actuator constraints
% u_max = [0.7;180]; % controller boundary; v = 0.7m/s, w = 180deg/s = pi
% rad/s
u_max = [1;pi]; % v = 1m/s, w = pi rad/s
warm_start = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];

% calculation of const params
u_min = -u_max;
Qhat = diag(repmat(Q,[N,1]));
Rhat = diag(repmat(R,[N,1]));
d = [repmat(u_max,[N,1]);repmat(u_min,[N,1])];
D = [eye(2*N);-eye(2*N)];
k=1;

%% Initialize simulation time and horizon time step
tf = 10;
tstep = 1; % For a loop rate = 10Hz

%% Sync the time horizon with ROS
% r = robotics.ros.Rate(node,10);
r = rosrate(1);
tic;
reset(r)
pause on
% Run the simulation for the simulation time with tstep increments
for t = 0:tstep:tf
    %% Obstacle prediction using GPR
    
    % current coordinates and velocity calculation for the obstacle
    % assuming the velocity of the obstacles are constant
    % current velocity of obstacle = initial velocity of obstacle
    % provide the current state of the obstacle here
    obstacle_curr_state = receive(obstacle_sub,3); % timeout of 3s
    bx0_curr = obstacle_curr_state.Transform.Translation.X;
    by0_curr = obstacle_curr_state.Transform.Translation.Y;
    
    
%     bx= bx0_curr + vx0*t; 
%     by= by0_curr + vy0*t;
    
    vx= vx0;
    vy= vy0;
    Xtrain =[Xtrain;t];
    
    % updating trains for learning
    % update the X Y positions each time step
%     bXtrain= [bXtrain;bx];
%     bYtrain= [bYtrain;by];

    bXtrain= [bXtrain;bx0_curr];
    bYtrain= [bYtrain;by0_curr];
    VXtrain= [VXtrain;vx];
    VYtrain= [VYtrain;vy];
    
    %creating exact GPR models
    gprMdbx = fitrgp(Xtrain,bXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdby = fitrgp(Xtrain,bYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvx = fitrgp(Xtrain,VXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvy = fitrgp(Xtrain,VYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    
    %% create future time step horizon, sync with ROS
    Xtrain_pred=[t;t+0.1;t+0.2;t+0.3;t+0.4;t+0.5;t+0.6;t+0.7];
    
%   testing
%     Xtrain_pred=[t;t+0.1];
%   testing

    % prediction from GPR for the horizon
    [bxtestpred] = predict(gprMdbx,Xtrain_pred);
    [bytestpred] = predict(gprMdby,Xtrain_pred);
    [vxtestpred] = predict(gprMdvx,Xtrain_pred);
    [vytestpred] = predict(gprMdvy,Xtrain_pred);
    % predicted horizon for obstacle
    obstaclepred = [bxtestpred,bytestpred,vxtestpred,vytestpred];
%     obstaclepred = [obstaclepred;bxtestpred,bytestpred,vxtestpred,vytestpred];
    %% trajectory planning
    
    %% provide the current state of the robot
    turtlebot_curr_state = receive(turtlebot_sub,3); % timeout of 3s
    x0_curr = turtlebot_curr_state.Transform.Translation.X;
    y0_curr = turtlebot_curr_state.Transform.Translation.Y;

    quatW_curr = turtlebot_curr_state.Transform.Rotation.W;
    quatX_curr = turtlebot_curr_state.Transform.Rotation.X;
    quatY_curr = turtlebot_curr_state.Transform.Rotation.Y;
    quatZ_curr = turtlebot_curr_state.Transform.Rotation.Z;
    
    % Euler ZYX
    angles_curr = quat2eul([quatW_curr quatX_curr quatY_curr quatZ_curr]);
    theta0_curr = angles_curr(1);
%     theta0_curr = rad2deg(angles_curr(1));
    
    current_state = [x0_curr;y0_curr;theta0_curr];
    traj= get_stream_trajectory(tstep,current_state,obstacle_type,obstaclepred);

    % current state of robot from initial values -- testing
%     current_state = [x0;y0;theta0];
%     traj= get_stream_trajectory(tstep,current_state,obstacle_type,obstaclepred);

    % get desired robot state from horizon data
    x0_d= traj(1,2);
    y0_d= traj(2,2);
    theta0_d= traj(3,2);
    
    z=[x0_d;y0_d;theta0_d];
    
    % update robot state
    zinit=[zinit,z];
    % figure('Name','Stream Flow');
    % plot(traj(1,:), traj(2,:),'r-');
    % hold on
    
    %% Stop when robot is found very close to origin
%     dist_from_origin = sqrt(x0_curr*x0_curr + y0_curr*y0_curr);
    
    dist_from_origin = sqrt(x0*x0 + y0*y0);
    if dist_from_origin <= 1
        break;
    end
    
%     delete(vch);
%     delete(vch1);

%     vch1 = viscircles([bx,by], radius_obs,'Color','r');
%     plot(bxtestpred, bytestpred,'r-');
%     vch = viscircles([x0,y0], radius_robot,'Color','g');
%     plot(traj(1,:), traj(2,:),'g-');
%     
%     hold on;
%     pause(0.01);
%     delete(vch);
%     delete(vch1);
    
    
    
    %% MPC
    % get the velocity and state values from stream function
    vr= traj(4,:);
    qr= traj(1:3,:);
    
    % check time step for MPC calculation for turtlebot simulation
    dt=tstep;
    % update A and B matrix for the window
    for i = 1:N
        A(:,:,i) = [1, 0, -vr(i)*sin(qr(3,i)*dt);
            0, 1, vr(i)*cos(qr(3,i)*dt);
            0, 0 ,1];
        B(:,:,i) = [cos(qr(3,i))*dt, 0;
            sin(qr(3,i))*dt, 0;
            0, dt];
    end
    
    
   % introduce new Ahat(3N,3), Bhat(3N,2N)
    Ahat = repmat(eye(3,3),N,1);
    Bhat = repmat(zeros(3,2),N,N);
    for i = 1:N
        Bhat(i*3-2:end,i*2-1:i*2) = repmat(B(:,:,i),N+1-i,1);
        for j = i:N
            Ahat(j*3-2:j*3,:) = A(:,:,i)*Ahat(j*3-2:j*3,:);
            for m = i+1:j
                Bhat(j*3-2:j*3,i*2-1:i*2) = A(:,:,m)*Bhat(j*3-2:j*3,i*2-1:i*2);
            end
        end
    end

    % Optimization
    ehat= [0.00001;0.00001;0.00001];
    % error = current - desired
%     ehat = [x0_curr-x0_d; y0_curr-y0_d; theta0_curr-theta0_d];
    H = 2*(Bhat'*Qhat*Bhat+Rhat);
    f = 2*Bhat'*Qhat*Ahat*ehat;
    OPTIONS = optimset('Display','off','MaxIter',5);
    % OPTIONS = optimset('Display','off');
    q_op = quadprog(H,f,[],[],[],[],[],[],warm_start,OPTIONS);
    
    warm_start = q_op;
    
%   testing
    dt=tstep;
%   testing
%     q_op(1) = 0;
%     q_op(2) = 0;
    
    % take velocity from stream function 
    % use current values of theta for next step calculations ?
    %% theta0_curr OR traj(3,1) % traj(3,1) = theta0_curr in code
%     x0 = x0 + (traj(4,1)+q_op(1))*cos(theta0)*dt; 
%     y0 = y0 + (traj(4,1)+q_op(1))*sin(theta0)*dt;
%     theta0 = theta0 + (traj(5,1)+q_op(2))*dt;
    
    %% Send control inputs to TurtleBot
    % Control inputs for TurtleBot
    velocityX = (traj(4,1)+q_op(1))*cos(theta0);
    velocityY = (traj(4,1)+q_op(1))*sin(theta0);
    omegaZ = traj(5,1)+q_op(2);
%     omegaZ_rad = deg2rad(omegaZ);
    
    velocityLinTot = sqrt(velocityX^2 + velocityY^2);
    
    %% Velocity Limitation Status
    if abs(velocityLinTot) > 1
        velocity_limit_reqd = true;
        disp('Velocity limitation active')
    else
        velocity_limit_reqd = false;
        disp('Velocity limitation not active')
    end
    
    
    if enable_ros == true
        % Create a ROS publisher and message for velocity topic
        my_turtlebot = rospublisher('/mobile_base/commands/velocity');
        velmsg = rosmessage(my_turtlebot);

%         velmsg = rosmessage('geometry_msgs/Twist');
%       my_turtlebot = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
        
        % Velocity in X and Y axes
        % Velocity is limited to safe values
        %velmsg.Linear.X = limiter_min_max(velocityX, -0.7, 0.7); % 0.7m/s
        %velmsg.Linear.Y = limiter_min_max(velocityY, -0.7, 0.7); % 0.7m/s
        %velmsg.Linear.X = velocityX;
        %velmsg.Linear.Y = velocityY;
        
        %% Select and apply velocity limitation loop
        
        if velocity_limit_reqd == true
            
            %% Velocity Limitation Block
            % Initialize the limits
            OldMin = min(nonzeros(traj(4,:)));
            OldMax = max(nonzeros(traj(4,:)));
            NewMin = 0;
            NewMax = 1;
            row = 1;
            
            for OldValue = traj(4,:)    

                if (OldMin ~= OldMax && NewMin ~= NewMax)
                    NewValue = ((((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin);
                else
                    NewValue = (NewMax + NewMin) / 2;
                end

                VelList(row,:) = NewValue;
                % Apply the new velocity
                velmsg.Linear.X = NewValue;
                row = row + 1;

                % ----

                % Total Linear Velocity
                % velmsg.Linear.X = velocityLinTot;
                velmsg.Linear.Y = 0;
                % Steer about Z-axis
                %velmsg.Angular.Z = limiter_min_max(omegaZ, -180, 180); % 180deg/s
%                 velmsg.Angular.Z = omegaZ_rad; % calculation in deg but converted to radians
                velmsg.Angular.Z = omegaZ; % calculation is in radians
                % Publish velocity and steer to the TurtleBot
                send(my_turtlebot, velmsg);

                %% Wait the desired Hz time for the actuators to react
                waitfor(r);

                % ----

            end
            
        else
            % Total Linear Velocity
            velmsg.Linear.X = velocityLinTot;
            velmsg.Linear.Y = 0;
            % Steer about Z-axis
            %velmsg.Angular.Z = limiter_min_max(omegaZ, -180, 180); % 180deg/s
%             velmsg.Angular.Z = omegaZ_rad; % calculation in deg but converted to radians
            velmsg.Angular.Z = omegaZ; % calculation is in radians
            % Publish velocity and steer to the TurtleBot
            send(my_turtlebot, velmsg);

            %% Wait the desired Hz time for the actuators to react
            waitfor(r);
            
        end
            
    end
    
    if enable_ros == false
        %% Final robot state calculated by Stream function and corrected by MPC
        zf=[x0;y0;theta0];
        % total robot state/trajectory using MPC
        zmpc=[zmpc,zf];

        xlim([0 10]);
        ylim([0 10]);

        vch1 = viscircles([bx,by], radius_obs,'Color','r');
        %figure('Name','Realtime trajectory Plot of Robot and Obstacle');
        plot(bxtestpred, bytestpred,'r-');
        vch = viscircles([x0,y0], radius_robot,'Color','g');
        plot(traj(1,:), traj(2,:),'g-');
        legend({'Robot','Obstacle'})

        hold on;
        pause(0.01);
        delete(vch);
        delete(vch1);
    end
 
end
toc;

if enable_ros == false
    %% Plot what is required
    figure('Name','Trajectory Plot of Robot and Obstacle');
    plot(zinit(1,:), zinit(2,:),'b-');
    hold on
    plot(zmpc(1,:), zmpc(2,:),'g-');
    hold on
    plot(bXtrain, bYtrain,'r-');
    hold on
    legend({'Robot trajectory from Stream Function','Robot trajectory using MPC','Obstacle trajectory using GPR'},'Location','southeast')
end

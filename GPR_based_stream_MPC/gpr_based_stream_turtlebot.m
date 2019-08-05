%% Initialize
clc;
clear;
close all;

%% Select obstacle type 
% 0 : Stationary Obstacles 
% 1 : Moving Obstacles
obstacle_type = 0;

%% Select state feedback type
% 0 : Fixed positions
% 1 : VICON motion capture system
state_fb = 0;

if state_fb == 0    
    % Initial robot state
    x0= 9;
    y0= 5;
    theta0= 0;
    % Initial obstacle state
    bx0= 5.00;
    by0= 5.00;
elseif state_fb == 1
    % Provide the initial/starting state of robot and obstacle
    state_sub = rossubscriber('/vicon/turtlebot/turtlebot');
    % Set x0 y0 theta0
    
    % Set bx0 by0
    
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
% store the robot state
zinit= [x0;y0;theta0];
% store the robot state with MPC
zmpc= [x0;y0;theta0];

figure;
%% Update radii of the robot and obstacle
radius_robot=0.3;
radius_obs= 1.4;

%% MPC block
% controller params
N = 7; % length of window
Q = [1;1;0.5];
R = [0.1;0.1];
u_max = [0.47;3.77]; % controller boundary
warm_start = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];

% calculation of const params
u_min = -u_max;
Qhat = diag(repmat(Q,[N,1]));
Rhat = diag(repmat(R,[N,1]));
d = [repmat(u_max,[N,1]);repmat(u_min,[N,1])];
D = [eye(2*N);-eye(2*N)];
k=1;

% % Initialize ROS and connect to TurtleBot
% ip_turtlebot = '130.215.121.108';
% rosinit(ip_turtlebot)

% initialize_turtlebot();

% Initialize simulation time and horizon time step
tf = 100.00;
tstep = 0.10;

%% Sync the time horizon with ROS 

% Run the simulation for the simulation time with tstep increments
for t = 0.00:tstep:tf    
    %% Obstacle prediction using GPR
    
    % current coordinates and velocity calculation for the obstacle
    % provide the current state of the obstacle here
    bx= bx0 + vx0*t; 
    by= by0 + vy0*t;
    vx= vx0;
    vy= vy0;
    Xtrain =[Xtrain;t];
    
    % updating trains for learning
    % update the X Y positions each time step
    bXtrain= [bXtrain;bx];
    bYtrain= [bYtrain;by];
    VXtrain= [VXtrain;vx];
    VYtrain= [VYtrain;vy];
    
    %creating exact GPR models
    gprMdbx = fitrgp(Xtrain,bXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdby = fitrgp(Xtrain,bYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvx = fitrgp(Xtrain,VXtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    gprMdvy = fitrgp(Xtrain,VYtrain,'Basis','linear','FitMethod','exact','PredictMethod','exact');
    
    %% create future time step horizon, sync with ROS
    Xtrain_pred=[t;t+0.1;t+0.2;t+0.3;t+0.4;t+0.5;t+0.6;t+0.7];
    % prediction from GPR for the horizon
    [bxtestpred] = predict(gprMdbx,Xtrain_pred);
    [bytestpred] = predict(gprMdby,Xtrain_pred);
    [vxtestpred] = predict(gprMdvx,Xtrain_pred);
    [vytestpred] = predict(gprMdvy,Xtrain_pred);
    % predicted horizon for obstacle
    obstaclepred= [bxtestpred,bytestpred,vxtestpred,vytestpred];
    
    %% trajectory planning
    
    % provide the current state of the robot
    current_state = [x0;y0;theta0];
    traj= get_stream_trajectory(tstep,current_state,obstacle_type,obstaclepred);
    
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
    
    % Stop when robot state is very close to origin
    if x0*x0 + y0*y0 <= 1
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
    vr= traj(4,:);
    qr= traj(1:3,:);
    
    % check time step for MPC calculation for turtlebot simulation
    dt=tstep;
     % update A and B matrix for the window
    for i = 1:N
        A(:,:,i) = [1, 0, -vr(i)*sin(qr(3,i)*dt);
            0, 1, vr(i)*cos(qr(3,i)*dt)
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

    
    ehat= [0.00001;0.00001;0.00001];
    H = 2*(Bhat'*Qhat*Bhat+Rhat);
    f = 2*Bhat'*Qhat*Ahat*ehat;
    OPTIONS = optimset('Display','off','MaxIter',5);
    % OPTIONS = optimset('Display','off');
    q_op = quadprog(H,f,[],[],[],[],[],[],warm_start,OPTIONS);
    
    warm_start = q_op;
    
    % take velocity from stream function and use current theta for next
    % step calculations
    x0= x0+(traj(4,1)+q_op(1))*cos(theta0)*dt;
    y0= y0+(traj(4,1)+q_op(1))*sin(theta0)*dt;
    theta0 = theta0+(traj(5,1)+q_op(2))*dt;
    
    %% Send control inputs to TurtleBot
    % Velocity
    % V = traj(4,1)+q_op(1) 
    
    % Steer
    % theta = theta0
    
    
    % final robot state calculated using Stream function and MPC
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
    figure('Name','Trajectory Plot of Robot and Obstacle');
    plot(zinit(1,:), zinit(2,:),'b-');
    hold on
    plot(zmpc(1,:), zmpc(2,:),'g-');
    hold on
    plot(bXtrain, bYtrain,'r-');
    hold on
    legend({'Robot trajectory from Stream Function','Robot trajectory using MPC','Obstacle trajectory using GPR'},'Location','southeast')

    
% terminate_turtlebot();
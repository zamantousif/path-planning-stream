%% Script to connect MATLAB and VICON Motion Capture System

% Clear the workspace
clc;
clear;

%% DO THIS BEFORE RUNNING THIS SCRIPT
% Open a Terminal and run the below command to launch VICON at the IP
% address specified (cross-check the IP of the system running VICON)

% roslaunch vicon_bridge vicon.launch ip:= 130.215.206.243

% create a ROS MASTER
% master = robotics.ros.Core; % use if master_uri is same as ros_ip

ROS_IP = '130.215.206.232';
ROS_MASTER_URI = 'http://130.215.121.108:11311'; % URI of TurtleBot

% Set ROS environment variables and initialize ROS with MASTER at the 
% specified IP address
initialize_ros(ROS_IP, ROS_MASTER_URI);

%% Subscribe to the VICON topic to get Stationary Obstacle state feedback
obstacle_sub = rossubscriber('/vicon/stationary_obs_track/stationary_obs_track');

%% Provide the initial/starting state of obstacle
% Get bx0 by0 of Obstacle
obstacle_pose_data = receive(obstacle_sub,3); % timeout of 3s
bx0 = obstacle_pose_data.Transform.Translation.X;
by0 = obstacle_pose_data.Transform.Translation.Y;

%% Subscribe to the VICON topic to get TurtleBot state feedback
turtlebot_sub = rossubscriber('/vicon/turtlebot_traj_track/turtlebot_traj_track');

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
theta0 = rad2deg(angles(1));

terminate_ros();

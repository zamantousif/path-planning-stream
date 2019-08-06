function [] = initialize_turtlebot()
% INITIALIZE_TURTLEBOT 
% Function to initialize the connectivity between MATLAB and TurtleBot
% Set ROS_IP and ROS_MASTER_URI
setenv('ROS_IP','130.215.206.232')
setenv('ROS_MASTER_URI','http://130.215.121.108:11311')

% Initialize ROS and connect to TurtleBot
ip_turtlebot = '130.215.121.108';
rosinit(ip_turtlebot)

end


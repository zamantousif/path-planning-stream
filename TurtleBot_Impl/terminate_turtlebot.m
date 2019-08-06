function [] = terminate_turtlebot()
% TERMINATE_TURTLEBOT
% Function to terminate the connectivity between MATLAB and TurtleBot

% Clear the workspace of publishers, subscribers and other ROS objects
clear
% Disconnect from the ROS network
rosshutdown

end


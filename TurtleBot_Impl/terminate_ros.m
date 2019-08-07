function [] = terminate_ros()
% TERMINATE_ROS
% Function to terminate the connectivity between MATLAB and all ROS
% objects

% Clear the workspace of publishers, subscribers and other ROS objects
clear
% Disconnect from the ROS network
rosshutdown

end


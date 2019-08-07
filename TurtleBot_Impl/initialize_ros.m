function [] = initialize_ros(ros_ip, ros_master_uri)
% INITIALIZE_ROS 
% Function to initialize ROS and set the ROS MASTER
% Set ROS_IP
setenv('ROS_IP',ros_ip)
% Set the ROS_MASTER_URI to the IP of TurtleBot
setenv('ROS_MASTER_URI',ros_master_uri)
% Initialize ROS
rosinit

end
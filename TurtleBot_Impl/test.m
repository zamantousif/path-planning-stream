%% Script to test the connectivity between MATLAB and TurtleBot

% Set ROS_IP and ROS_MASTER_URI
setenv('ROS_IP','130.215.206.232')
setenv('ROS_MASTER_URI','http://130.215.121.108:11311')

% Initialize ROS and connect to TurtleBot
ip_turtlebot = '130.215.121.108';
rosinit(ip_turtlebot)

velocityX = 0.1;     % meters per second
velocityY = 0.3;     % meters per second

% Publish velocity message to TurtleBot
my_turtlebot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(my_turtlebot);
velmsg.Linear.X = velocityX;
velmsg.Linear.Y = velocityY;

for num=1:10
    send(my_turtlebot,velmsg);
end

% Publish and send sound message to TurtleBot
soundpub = rospublisher('/mobile_base/commands/sound', 'kobuki_msgs/Sound');
soundmsg = rosmessage('kobuki_msgs/Sound');
soundmsg.Value = 6;      % Any number 0-6
send(soundpub,soundmsg);


% Clear the workspace of publishers, subscribers and other ROS objects
% clear
% Disconnect from the ROS network
% rosshutdown

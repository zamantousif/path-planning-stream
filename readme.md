## path-planning-stream
Implementation of vehicle path planning using stream functions on TurtleBot2

### **File Structure:**
#### **Stream_Function**
Simulation of stream function in MATLAB

#### **Stream_Function_ODE**
Simulation using ODE function in MATLAB

#### **GPR_based_stream_MPC**
Simulation of obstacle avoidance with stream function + GPR + MPC in MATLAB

#### **TurtleBot_Impl**
Implementation of stream function based obstacle avoidance for stationary obstacle on TurtleBot2
##### **Required:**
- gpr_based_stream_turtlebot.m (main)
- get_stream_trajectory.m
- initialize_ros.m
- terminate_ros.m
- limiter_min_max.m

### **Prerequisites or Dependencies:**

- MATLAB with Robotics Toolbox [on workstation/ laptop]
- ROS Kinetic [on workstation/ laptop]
- vicon_bridge with latest vicon_sdk [on workstation/ laptop]

### **Simulation**:

**Stationary Obstacle:**
- Run stream_stationary.m

**Moving Obstacle**
- Run stream_moving.m


### **Demonstration:**

Run the below commands in sequence

#### On the TurtleBot2:

Launch a terminal then run

- roslaunch turtlebot_bringup minimal.launch

#### On the Workstation/Laptop running MATLAB:

Launch a terminal then run

- roslaunch vicon_bridge vicon.launch ip_vicon:=<enter ip of vicon>

Launch MATLAB, open TurtleBot_Impl (available on this repo) then run the below M script

- gpr_based_stream_turtlebot.m

### **Additional:**
- TurtleBot 2 runs on ROS Indigo
- TurtleBot 2 has turtlebot_bringup, turtlebot_gazebo packages setup already
- VICON Tracker 3.4.x setup on workstation with 8 overhead + 2 tripod mounted cameras
- TurtleBot data published at /vicon/turtlebot_traj_track/turtlebot_traj_track
- Stationary obstacle data published at /vicon/stationary_obs_track/stationary_obs_track
- IP address of VICON can be set in vicon.launch within vicon_bridge (reference file available in this repo). This is used as the default IP of VICON and is optional on the terminal.
- IP address of TurtleBot and workstation (running MATLAB) need to be updated in the .m scripts (see comments in gpr_based_stream_turtlebot.m)

# HaIBot Robot

![image](https://github.com/vuthanhcdt/HaIBot/blob/main/Image/IMG_7643.JPG)


## Demo Video ##
**Multi-Sensor Fusion for Localization video**<br/> 
[![image](http://img.youtube.com/vi/W5xouNktPd0/0.jpg)](https://youtu.be/W5xouNktPd0)<br/>
**Multi-Sensor Fusion for Localization**<br/> 
[![image](http://img.youtube.com/vi/5wkIK1AP9bE/0.jpg)](https://youtu.be/5wkIK1AP9bE)


## source code download and build code please follow steps ##
1. create a new workspace on your local machine first<br/>
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/HaIBot/src
```
(Reference: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. git clone source code to your workspace<br/>
```
$ cd ~/haibot/src
$ https://github.com/vuthanhcdt/HaIBot.git
```
3. build code<br/>
```
$ cd ~/haibot/
$ catkin_make

## Test mapping with ps3 ##

```
$ roscore
  open new terminal(Just only run once time for ps3 paired)
$ sudo bash
$ roslaunch loco ps3_pairing.launch
  open new terminal
$ roslaunch loco hector_slam_pcar.launch
```
**On your local machine setting commmand**
```
$ roslaunch loco map_rviz_pcar.launch
```
## Test amcl localization with ps3 ##
**On TX1 setting commmand**
```
$ roscore
  open new terminal(Just only run once time for ps3 paired)
$ sudo bash
$ hciconfig hci0 reset
$ roslaunch loco ps3_pairing.launch
  open new terminal
$ roslaunch loco amcl_pcar.launch
```
**On your local machine setting commmand**
```
$ roslaunch loco rviz.launch
```

## Test loop navigation between any two waypoints ##
## The two waypoints are initial pose and goal pose respectively ## 
## Navigation map is in office 5F building ##
**On TX1 setting commmand**
```
$ roscore
  open new terminal
$ roslaunch loco nav_pcar.launch
```
**On your local machine setting commmand**
```
$ roslaunch loco nav_rviz_pcar.launch
```

Before running navigation, you first need to set robot initial pose and goal's pose:</br> 
1. First, you need to put robot on navigation start point in map
2. Click "2D Pose Estimate" button on rviz and click robot current pose in map on rviz to set initial pose
3. Use below command to know intial pose value and set initial_pose_x,initial_pose_y,initial_pose_a parameter value in amcl_pcar.launch
```
$ rosparam get /amcl/initial_pose_x (robot's x position)
$ rosparam get /amcl/initial_pose_y (robot's y position)
$ rosparam get /amcl/initial_pose_a (robot's yaw)
```
4. Goals pose as initial pose determine method, click "2D Pose Estimate" button on rviz and click goal's pose in map
5. Using step3 command to get goal's pose value
6. "waypoint_nav.py" is a send multiple goals python</br>
    Setting goal's pose and initial pose in "waypoint_nav.py"



run below command to send navigation goal

```
$ rosrun loco waypoint_nav.py (send navigation goals and ctrl+c key can stop navigation)
```

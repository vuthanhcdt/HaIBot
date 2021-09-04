# HaIBot Robot

![image](https://github.com/vuthanhcdt/HaIBot/blob/main/Image/IMG_7643.JPG)


## Demo Video ##
**EKF Localization video**<br/> 
[![image](http://img.youtube.com/vi/W5xouNktPd0/0.jpg)](https://youtu.be/W5xouNktPd0)<br/>
**EKF for Localization**<br/> 
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
```
## Test mapping with ps3 ##

```
$ roscore
  open new terminal
$ sudo bash
$ roslaunch mapping hector_slam_pcar.launch
```

## Test EKF localization ##
```
$ roscore
  open new terminal
$ sudo bash
$ roslaunch navigation navigation.launch
```

## Test Path tracking ##
```
$ roscore
  open new terminal
$ sudo bash
$ roslaunch navigation navigation.launch
  open new terminal
$ roslaunch go_point go_point.launch

```

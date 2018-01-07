# Turtlebot_arm
Turtlebot arm ROS project
___
# Description
## Introduction

## What this readMe file includes

## Summary of code files

___
# Steps
## Setup

## Calibration

## Detection

## Autonomy


___
# Our contribution
## Detection 

## Autonomy

___
# How to?


## How to test the code?
To run project from this repository you need Ubuntu 14.04 version and ROS Indigo igloo distribution.
If you do not have an active ROS workspace, you can create one by:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Now that you have a workspace, clone or download this repository into the (source) src directory of your workspace:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/sniper0110/Turtlebot_arm.git
```
Build the project:
```
$ cd ~/catkin_ws
$ catkin_make
```

In terminal to run the demo code
```
$roscd turtlebot_arm_block_manipulation\demo
$roslaunch block_manip_complete.launch.
```
Once all these items are confirmed,  rviz window is opened with the workspace.
you can see the complete action, detection of cube and Manipulation trajectory performed sucessfully.


## How to do calibration?


## How to change calibration parameters?

___
# Results
Videos and comments

___
# Suggested improvements
## Detection for any color

___
# Conclusion

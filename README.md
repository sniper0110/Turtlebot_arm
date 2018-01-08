# Turtlebot_arm
Turtlebot arm ROS project
___
# Description
## Introduction
This repository contains all the code necessary to allow the turtlebot arm PhantomX Pincher to perform a specific task along with the Kinect v1. The task is as follows : we will use the kinect to detect a **_green cube_** on top of a turtlebot, the arm then will pick that cube and place it on top of the table -more on the setup can be found in the ***setup*** section.

## What this readMe file includes
In this readMe file you will find explanations regarding the following sections :
  * A summary of the code files used.
  * Steps taken by our team to ensure the task is working properly including :
    * Setting up the environment.
    * Calibrating the Kinect to work properly with the turtlebot arm.
    * Detection of the cube using the Kinect.
    * Making the process of detection and picking_and_placing autonomous.
  * Our contribution to the pre-existing repository that could be found [here](https://github.com/NathanCrombez/turtlebot_arm), specifically regarding :
    * Detection using the Kinect.
    * Making the complete process autonomous.
  * How to? A tutorial-like section on how to :
    * Run and test our code.
    * Calibrate the Kinect to work with turtlebot arm.
    * Change the calibration parameters in the launch file to make things work with your specific setup.
  * Results obtained.
  * Problems encountred.
  * Suggested improvements.
  * Conclusion.
  * References.
  

## Summary of code files
The code included in the repository is in a nutshell doing the following tasks :
  * _turtlebot_arm_block_manipulation_ : this folder includes the code necessary to run if you want to start testing immediatly, it contains a _lunch file_ in the _demo folder_ named **block_manip_complete.launch**. If you run this launch file, you will be able to see first hand how the process work without having to intervene in anything, just run it and you're good to go!
  * _turtlebot_arm_bringup_ : this folder includes some necessary configuration files to make the turtlebot arm work (you don't need to worry about this code nor do you need to modify it).
  * _turtlebot_arm_description_ : this folder includes the files that ensure we have a virtual model of our robot (urdf files).
  * _turtlebot_arm_kinect_calibration_ : this folder includes the code necessary to calibrate the Kinect with turtlebot arm.
  * _turtlebot_arm_moveit_config_ : this folder includes the code necessary for the turtlebot arm to move (kinematics).
  * _turtlebot_arm_moveit_demos_ : this folder contains a demo that you can run if you would like to test how the arm moves.
___
# Steps
In this section we will describe what we have done to ensure things work smoothly.

## Setup
For the setup, the environment looks like this :


## Calibration
The calibration is needed because we have to make the turtlebot arm work with the Kinect, so when the Kinect detects an object and the arm needs to pick it up, the arm should know where that object is, since the arm has no sensors attached to it, this means we have to use the Kinect as a sensor working closely with the arm, so the calibration is for exactly this task. The calibration process is described below in the __**how to**__ section. 

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
To calibrate your camera extrinsic parameters
Here we have kinect off-board setup allows us to keep our current robot setup, and gives a guaranteed good view of the workspace. However, since the kinect is off-board, it only allows you to use one workspace at a time.

This extrinsic calibration procedure works by using a calibration pattern. The kinect can localize the calibration very precisely, and by moving the arm to certain positions on the pattern, we can calculate the transform between the kinect and any frame on the robot.

### Setup
The setup for the calibration involves Kinect camera that can see the workspace, and a workspace that the arm can reach.
Next, print out this check_7x6_27mm.pdf calibration pattern on A4 paper and attach it to the workspace, making sure that the arm can reach every point on the pattern.The final setup for calibration can be seen in below figure.


![img1](https://user-images.githubusercontent.com/22390134/34654003-9d7e1092-f3f5-11e7-9ba6-24db04b799e4.jpg)



### Calibration steps

```
roslaunch turtlebot_arm_bringup arm.launch
roslaunch turtlebot_arm_kinect_calibration calibrate.launch
```

This should detect the checkerboard and pop up the image shown below figure, with the calibration pattern edges overlaid and four points marked on the image.

we have to keep the image

The next step is to move the edge of the gripper to the four specified points in order shown in below figure. Note that there is one specific edge you are trying to move: if you orient the arm so that the un-actuated side of the gripper is on the left, it will be the bottom left point. Make sure that your setup matches the one pictured below.


![img2](https://user-images.githubusercontent.com/22390134/34654008-ab7bb532-f3f5-11e7-824e-781dc92c156e.jpg)



## How to change calibration parameters?
If you're calibrating an external kinect, open up a new terminal window, and run the static transform output by the script, such as 
```
rosrun tf static_transform_publisher -0.26683 -0.122903 -0.537733 0.5 -0.499602 0.5 0.500398 /arm_base_link /openni_camera 100

```
As long as this command runs, the static_transform_publisher will publish the transform between a frame on theTurtleBot and the kinect frame. If you move the physical camera, you will need to recalibrate again.
___
# Results
Videos and comments

___
# Problems encountered

___
# Suggested improvements
## Detection for any color

___
# Conclusion

HELLO, IT'S ME. CAN YOU SEE ME?
___
# References





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
To calibrate your camera extrinsic parameters
Here we have kinect off-board setup allows us to keep our current robot setup, and gives a guaranteed good view of the workspace. However, since the kinect is off-board, it only allows you to use one workspace at a time.

This extrinsic calibration procedure works by using a calibration pattern. The kinect can localize the calibration very precisely, and by moving the arm to certain positions on the pattern, we can calculate the transform between the kinect and any frame on the robot.

# Setup
The setup for the calibration involves Kinect camera that can see the workspace, and a workspace that the arm can reach.
Next, print out this check_7x6_27mm.pdf calibration pattern on A4 paper and attach it to the workspace, making sure that the arm can reach every point on the pattern.The final setup for calibration can be seen in below figure.


![img1](https://user-images.githubusercontent.com/22390134/34654003-9d7e1092-f3f5-11e7-9ba6-24db04b799e4.jpg)



## Calibration steps

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
# Suggested improvements
## Detection for any color

___
# Conclusion

HELLO, IT'S ME. CAN YOU SEE ME?

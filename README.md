# Turtlebot_arm
Turtlebot arm ROS project
___
# Introduction
## What the complete project is about
The class project is about implementing a __scenario__ where multiple robots will work together to perform a cetain task. We have a turtlebot and two arms sitting on a table, one is in the **load** area and the other is in the **drop** area. The turtlebot needs to navivate it's way to the **load** area, then, the arm will pick a **cube** from the table and places it on top of the turtlebot once it receives a message from the turtlebot that it has reached the **load** area. Once the placing is done, the turtlebot needs to navigate its way to the **drop** area. After this comes our part of the task. Once the turtlebot reaches the **drop** area, the arm will receive a message from the robot indicating that it is positioned in the right place. After this, we need to detect the cube, then the arm should pick it and place it on top of the table and the task is done! The setup looks like this :

<img width="711" alt="screen shot 2018-01-08 at 17 09 40" src="https://user-images.githubusercontent.com/8482070/34679640-da3ca40c-f496-11e7-9c0f-14f2450fb57e.png">

___
## Description
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
For the setup, we have two main components, the turtlebot arm and the Kinect as described in an image below.


## Calibration
The calibration is needed because we have to make the turtlebot arm work with the Kinect, so when the Kinect detects an object and the arm needs to pick it up, the arm should know where that object is, since the arm has no sensors attached to it, this means we have to use the Kinect as a sensor working closely with the arm, so the calibration is for exactly this task. The calibration process is described below in the __**how to**__ section. 

## Detection
The detection part is done using the Kinect, we have developped our own method to detect the cube, it will be described in the **Our contribution** section. In the detection, the arm needs to be able to __"know"__ where the cube is using some operations done on the point cloud acquired by the kinect.

## Autonomy
Once we finished the steps above, we needed to work on making the process autonomous. To do this the arm had to wait for a message from the turtlebot, the message is `vsdone` of type `std_msgs/String` from the topic `/delay/robot_status2`. Once this message is received the detection part starts and when the cube is detected, the arm will pick the cube and place it on top of the table, all of this is done without our interference, that is, we run the launch file `turtlebot_arm_block_manipulation/demo/block_manip_complete.launch` and that's it! The code developed will do the rest.

___
# Our contribution
In this section, we will describe our contribution to the pre-existing project, specifically in the **detection** and **autonomy** parts.

## Detection 
For the detection, we implemented a method for detecting the cube on top of the turtlebot. The way we did this, is by detecting planes in the point cloud acquired by the Kinect, after this we remove the planes and go through the rest of the __chunks__ or __clusters__ of the point cloud and check whether that cluster is a **green cube** or not. We added the color feature to the detection process because it helped a lot in detecting the cube and in a very short amount of time as well! The drawback of course, is that now we can only detect green cubes. This process is implemented in `Turtlebot_arm/turtlebot_arm_block_manipulation/src/block_detection_action_server.cpp`. Here are some bits of code responsible for doing this.

__How to create a cluster and customize it to look for a cube :__
```
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
   
    ec.setClusterTolerance(0.003);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(2000); 

    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

```
__Adding a virtual block (cube) on top of the detected block :__
```
    if (greenSum > blueSum && greenSum > redSum)  // a green cube is used in this project
         addBlock(xmin , ymin , zmax , angle, (unsigned long) blueSum/pixelCount);
```
In fact, we removed two planes before we started searching for the cube, the first plane is of the table and the second one is of the turtlebot top. 
The point cloud before filtering anything looked like this :

![before_filtering](https://user-images.githubusercontent.com/8482070/34687242-6c06d008-f4ae-11e7-9508-3daa7af8158b.png)  
![before_filtering2](https://user-images.githubusercontent.com/8482070/34687243-6c37a93a-f4ae-11e7-88f8-922f24164b3f.png)


After filtering the table plane, it looked like this :

![filter_table](https://user-images.githubusercontent.com/8482070/34687246-6cb9d07c-f4ae-11e7-8f7f-f7d7bd8c3a82.png)
![filter_table2](https://user-images.githubusercontent.com/8482070/34687247-6ce42430-f4ae-11e7-9c75-b056bc4108ac.png)


And after fitering the robot top, it looked like this :

![filter_robot](https://user-images.githubusercontent.com/8482070/34687244-6c655132-f4ae-11e7-9a44-4b6d1bdc13f8.png)
![filter_robot2](https://user-images.githubusercontent.com/8482070/34687245-6c9161be-f4ae-11e7-8ebf-63516364adc7.png)


We can notice now that there are only some chunks of point clouds left and it will take less time and search to find the cube. The method worked very nicely and we were able to detect the cube in a very short amount of time (from 5 to 15 seconds).

## Autonomy
To make the process autonomous (as we described above), we needed to change some parts of the code in `turtlebot_arm_block_manipulation/src/interactive_manipulation_action_server.cpp`. The method is simple, we wait for a message from the turtlebot, this message tells the arm that the turtlebot is well positioned infront of it and that the arm can reach and pick the cube now.
Here are some code snippets for achieving this task :

```
    starting_pose = pose; // the pose where the cube was detected
    ending_pose.position.x = target_X; // target_X is predefined
    ending_pose.position.y = target_Y; // target_Y is predefined
    ending_pose.position.z = starting_pose.position.z; 
    ending_pose.orientation = starting_pose.orientation;
    
    moveBlock(starting_pose, ending_pose); // moving the cube from where it was detected to a predefined position on the table
```
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





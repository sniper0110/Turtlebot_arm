/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Distribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Distributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Distributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



/*
*   WARNING  **************
*
*    1) Follow safety guidelines at http://www.usfirst.org/sites/default/files/uploadedFiles/Robotics_Programs/FRC/Resources/2015%20FRC%20Team%20Safety%20Manual-%20FINAL%202.6.15.pdf
*    2) Follow manufacturer guidelines 
*    3) Set the arm to a very slow speed
*    4) Always be ready to emergency stop the arm
*    5) MOUNT THE ARM USING break-away mounts such as small spring based clips 
*    6) Review and understand the code
*
*  REQUIREMENTS ************
*
*  0. ROS Indigo
*  1. Kinect and PhantomX Pincher Arm
*  2. Turtlebot arm version https://github.com/answer17/turtlebot_arm
*  3. Arbotix ROS version https://github.com/answer17/arbotix_ros
*  4. Calibration as outlined in block_detection_action_server.cpp
*
*  USAGE ***************
*
*  Launch block_manipulation_demo.launch
*  When D is pressed, it will detect blocks at table height with size
*  specified.  The blocks will be shown in Rviz
*  When M is pressed, those blocks will be sorted according to color and moved to 
*   destination
*
*  SETUP ****************
* 
*     Kinect position: Y is appx 450mm from arm base and appx 450mm above table
*                      X is appx 170 from arm base
*                       
*      This gives Kinect best view of blocks without arm obstruction and is in
*      sweet spot of Kinect accuracy
*
*      blocks X position is from 12cm from arm base to 24cm
*
*
*  DIAGRAM (from above) *****************
*
*              /KINECT\     
*
*             [blocks]
    Y
*   ^   [base]-----<grip
*   |   
*   |         [destination (wrong color)]
    |         [destination (target color)]
*   ----->X
*
*
* When you have read the above statements
* remove the space between the asterisk and slash below:
*/



#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>
#include <turtlebot_arm_block_manipulation/PickAndPlaceAction.h>
#include <turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction.h>
#include <arbotix_msgs/Relax.h>

#include <string>
#include <sstream>
 #include "std_msgs/String.h"

const std::string gripper_param = "/gripper_controller";
const std::string pick_and_place_topic = "/pick_and_place";

// Flag to indicate block matched our target color
#define COLOR_MATCH 0.002

namespace turtlebot_arm_block_manipulation
{

class BlockManipulationAction
{
private:
  ros::NodeHandle nh_;
  int color1Count, color2Count;
  ros::Subscriber sub_;
  
  geometry_msgs::PoseArray poseMsg;
  geometry_msgs::Pose start_pose_bumped, end_pose_bumped;
  
  // Actions and services
  actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;
  actionlib::SimpleActionClient<PickAndPlaceAction> pick_and_place_action_;
  
  BlockDetectionGoal block_detection_goal_;
  InteractiveBlockManipulationGoal interactive_manipulation_goal_;
  PickAndPlaceGoal pick_and_place_goal_;

  // Parameters
  std::string arm_link;
  double gripper_open, gripper_tighten, gripper_closed, z_up, z_down, block_size, target_x, target_y;
  bool once;
  // TODO - don't limit to 20 blocks
  geometry_msgs::Pose blockList[20];  // list of the positions of blocks we;ve found
  float colorList[20];   // Colors of the blocks we've found
  int blockIndex;  // block we are working on
  int blockCount;  // number of blocks found

  
public:
    std::string armMode;
    bool block_exist;  //dna
    
  BlockManipulationAction() : nh_("~"),
    block_detection_action_("block_detection", true),
    interactive_manipulation_action_("interactive_manipulation", true),
    pick_and_place_action_("pick_and_place", true), block_exist(false)
  {
    // Load parameters
    nh_.param<std::string>("arm_link", arm_link, "/arm_link");
    nh_.param<double>(gripper_param + "/max_opening", gripper_open, 0.042);
    nh_.param<double>("grip_tighten", gripper_tighten, -0.0015);
    nh_.param<double>("z_up", z_up, 0.12);   // amount to lift during move
    nh_.param<double>("table_height", z_down, 0.01);
    nh_.param<double>("block_size", block_size, 0.03);  // block size to detect
    nh_.param<bool>("once", once, true);
    nh_.param<double>("target_x", target_x, .26);    // X target for first block
    nh_.param<double>("target_y", target_y, -.06);   // Y target for first block

    //Subscribe the topic from fine positioning
    sub_ = nh_.subscribe("/relay/robot_status2", 10, &BlockManipulationAction::moveCB, this);

    // Initialize goals
    block_detection_goal_.frame = arm_link;
    block_detection_goal_.table_height = z_down;
    block_detection_goal_.block_size = block_size;
    
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = block_size; - gripper_tighten ;//- 0.0035;//pick_and_place_goal_.gripper_closed = block_size - gripper_tighten;
    pick_and_place_goal_.topic = pick_and_place_topic;
    
    color2Count = color1Count = 0;
    
    ROS_INFO("Gripper settings: closed=%.4f block size=%.4f tighten=%.4f", (float) pick_and_place_goal_.gripper_closed, (float) block_size, (float) gripper_tighten );
    
    interactive_manipulation_goal_.block_size = block_size;
    interactive_manipulation_goal_.frame = arm_link;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    block_detection_action_.waitForServer();
    ROS_INFO(" 1. Found block_detection server.");
    
    interactive_manipulation_action_.waitForServer();
    ROS_INFO(" 2. Found interactive_manipulation server.");
    
    pick_and_place_action_.waitForServer();
    ROS_INFO(" 3. Found pick_and_place server.");
    
  }
  
  // dna
  void moveCB(const std_msgs::String::ConstPtr& msg)
  {
      if (std::strcmp(msg->data.c_str(), "vsdone") == 0 && armMode != "d")
      {
          detectBlocks(); armMode="d";
      }
        
  }
  
  void detectBlocks()
  {
  // Have Block Detection Server detect blocks and callback "addBlocks" when done
    bool block_exist = true;  //dna
    block_detection_action_.sendGoal(block_detection_goal_, boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
    
  }
  
  void addBlocks(const actionlib::SimpleClientGoalState& state, const BlockDetectionResultConstPtr& result)
  {
    //ROS_INFO(" Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    // Save blocks for later use during sorting
       for (unsigned int i=0; i < result->blocks.poses.size(); i++)
    {    
      blockList[i] = result->blocks.poses[i];
      colorList[i] = blockList[i].position.z;
      
      // TODO Fix kluge - Z POSITION WAS OVERWRITTEN TO PASS COLOR!!
      // TODO table height is hardcoded, and restored here
      blockList[i].position.z = 0.125; //* blockList[i].position.x -0.085;
      
      //ROS_INFO("   Saving block %d x=%f", i, blockList[i].position.x);
    }
    
    blockCount = result->blocks.poses.size();
    blockIndex = 0;
    
    // Add blocks to Interactive Manipulation Server for Rviz visualization
    //ROS_INFO("Adding blocks...");
    interactive_manipulation_action_.sendGoal(interactive_manipulation_goal_, boost::bind( &BlockManipulationAction::pickAndPlace, this, _1, _2));
  }
  
  void pickAndPlace(const actionlib::SimpleClientGoalState& state, const InteractiveBlockManipulationResultConstPtr& result)
  {
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Select Marker Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    //ROS_INFO(" Got interactive marker callback.");
    ROS_INFO("Picking and placing...");
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
  }
  
  void finish(const actionlib::SimpleClientGoalState& state, const PickAndPlaceResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO(" Pick and place - Succeeded!");
    else
      ROS_INFO(" Pick and place - Failed! %s",  state.toString().c_str());

    if (once)
      ros::shutdown();
            
    if (blockIndex < blockCount && armMode == "m")
      organizeBlocks();
    else 
      detectBlocks();
  }
  
  
  // This will organize the blocks by color
  void organizeBlocks() {
  ROS_INFO("=== Organize Blocks: block %d color %.4f ===", blockIndex, colorList[blockIndex]);
  int targetCount;
  
  geometry_msgs::Pose endPose;
  
  // This routine calculates the target position for block.
  // Blocks which match the target color go to one area, others go to second area
  // For each area, we need to keep incrementing the target so we dont place
  // a block on top of another.  Once we fill up a column, we move over to next
  // column.
  //  Note - we keep two target lists: one for color1 and one for color2
  
  if (colorList[blockIndex] == COLOR_MATCH)  // Does this block match target color?
  {
  ROS_INFO("  Block matches target Color");
  targetCount = color1Count;
  color1Count++;
  if (color1Count >= 4) color1Count = 0;  // Four blocks per column, then move to next column
  }
  else  // Not target color, use alternate destination
  {
  ROS_INFO("  Block is not target Color");
  targetCount = color2Count;
  color2Count++;
  if (color2Count >= 4) color2Count = 0;  // Four blocks per column, then move to next column
  }
  
  // each time we place a block we move to new row 
  // TODO dont hardcode offset
  endPose.position.x = target_x - ((targetCount % 5) * .05);
  endPose.position.y = target_y;
  
  // When we run out of space on a column, have endPose go to next column
  if (targetCount >= 5) endPose.position.y -=  .05;
  if (targetCount >= 10) endPose.position.y -=  .05;
  
  // TODO - replace hardcoded Z of endPose
  endPose.position.z = -.055;
  
  // Y Destination is offset if color matches target color
  if (colorList[blockIndex] == COLOR_MATCH) endPose.position.y +=  .05;
  ROS_INFO("targetX=%.4f targetY=%.4f", target_x, target_y);
  
  ROS_INFO(" EndPose x=%.4f y=%.4f z=%.4f ", endPose.position.x, endPose.position.y, endPose.position.z);
  
    // Go through list of blocks and move them
    if (blockIndex < blockCount) {
       std::cout << "--------------Block number : " << blockIndex << std::endl;
       moveBlock(blockList[blockIndex], endPose);
       blockIndex++;
       targetCount++;
       }
  }
    
  void moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
  double bump_size = 0.005;
  
    // Return pickup and place poses as the result of the action
    
    start_pose_bumped = start_pose;
    start_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    end_pose_bumped = end_pose;
    end_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    // Publish pickup and place poses for visualizing on RViz
    
    poseMsg.header.frame_id = arm_link;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.poses.push_back(start_pose_bumped);
    poseMsg.poses.push_back(end_pose_bumped);
    //ROS_INFO("Demo publishing to PickNPlace.  PoseX=%.4f", (float) poseMsg.poses[0].position.x);
    
    pick_and_place_goal_.pickup_pose = start_pose_bumped;
    pick_and_place_goal_.place_pose = end_pose_bumped;
    pick_and_place_goal_.topic = "";
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
    
    pick_and_place_goal_.topic = pick_and_place_topic;  // restore topic
   }
 };
};


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");
  turtlebot_arm_block_manipulation::BlockManipulationAction manip;
  
  ros::spin();
}

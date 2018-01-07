/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
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
 * 
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <cmath>
#include <algorithm>
#include <sstream> 

// Flag to indicate block matched our target color
//#define COLOR_MATCH 0.0        // zmax value for block on the turtlebot
//#define NO_COLOR_MATCH 0.0

#define COLOR_MATCH 0.002-0.054 // zmax value for block on the table
#define NO_COLOR_MATCH 0.0-0.054

//#define COLOR_MATCH 0.002  // the original code
//#define NO_COLOR_MATCH 0.0



namespace turtlebot_arm_block_manipulation
{

class BlockDetectionServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_arm_block_manipulation::BlockDetectionAction> as_;
  std::string action_name_;
  turtlebot_arm_block_manipulation::BlockDetectionFeedback feedback_;
  turtlebot_arm_block_manipulation::BlockDetectionResult result_;
  turtlebot_arm_block_manipulation::BlockDetectionGoalConstPtr goal_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;
  
  // Parameters from goal
  std::string arm_link_;
  double block_size_;
  double table_height_;
  
  double x_adj, y_adj, z_adj;
  double blueRedDiff;
  
  ros::Publisher block_pub_;
  ros::Publisher c_obj_pub_;
  
  // Parameters from node
  std::vector<double> table_pose_;
  
public:
  BlockDetectionServer(const std::string name) : 
    nh_("~"), as_(name, false), action_name_(name)
  {
    // Load parameters from the server.
    if ((nh_.getParam("table_pose", table_pose_) == true) && (table_pose_.size() != 3))
    {
      ROS_ERROR("Invalid table_pose vector size; must contain 3 values (x, y, z); ignoring");
      table_pose_.clear();
    }
    
    nh_.param<double>("/block_manipulation_demo/x_adj", x_adj, .008);
    nh_.param<double>("/block_manipulation_demo/y_adj", y_adj, 0.0);
    nh_.param<double>("/block_manipulation_demo/z_adj", z_adj, 0.0);
    
    // Parameter for target color.  Note: this is RGB blue minus red.  Green 
    // is ignored.  should switch this to true HSV hue based comparison
    nh_.param<double>("/block_manipulation_demo/blue_red", blueRedDiff, 0.0);
    
    
    ROS_INFO("Adjustments %.4f %.4f %.4f", (float) x_adj, (float) y_adj,(float) z_adj);
    
    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));
    
    as_.start();
    
    // Subscribe to point cloud
    sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockDetectionServer::cloudCb, this);

    // Publish the filtered point cloud for debug purposes
    pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish detected blocks poses
    block_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/turtlebot_blocks", 1, true);
  }

  void goalCB() //CB = callback
  {
    ROS_INFO("[block detection] Received goal!");
    // accept the new goal
    result_.blocks.poses.clear();
    
    goal_ = as_.acceptNewGoal();
    
    block_size_ = goal_->block_size;
    table_height_ = goal_->table_height;
    arm_link_ = goal_->frame;

    result_.blocks.header.frame_id = arm_link_;

    // Add the table as a collision object, so it gets filtered out from MoveIt! octomap
    // I let it optional, as I don't know if this will work in most cases, honesty speaking
    if (table_pose_.size() > 0)
    {
      if (std::abs(table_height_ - table_pose_[2]) > 0.05)
        ROS_WARN("The table height (%f) passed with goal and table_pose[2] parameter (%f) " \
                 "should be very similar", table_height_, table_pose_[2]);
      addTable();
    }
    
    // Reload params - TODO
    nh_.param<double>("/block_manipulation_demo/x_adj", x_adj, .008);
    nh_.param<double>("/block_manipulation_demo/y_adj", y_adj, 0.0);
    nh_.param<double>("/block_manipulation_demo/z_adj", z_adj, 0.0);
    
    ROS_INFO("Adjustments %.4f %.4f %.4f", (float) x_adj, (float) y_adj,(float) z_adj);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Only do this if we're actually actively working on a goal.
    if (!as_.isActive())
      return;

    result_.blocks.header.stamp = msg->header.stamp;

    // convert to PCL
    pcl::PointCloud < pcl::PointXYZRGB > cloud;
    pcl::fromROSMsg(*msg, cloud);

    // transform to whatever frame we're working in, probably the arm frame.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf_listener_.waitForTransform(std::string(arm_link_), cloud.header.frame_id,
                                  ros::Time(cloud.header.stamp), ros::Duration(1.0));
    if (!pcl_ros::transformPointCloud(std::string(arm_link_), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR("Error converting to desired frame");
      return;
    }

    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.005);

    // Limit to things we think are roughly at the table height.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("z");

    //pass.setFilterLimits(table_height_ - 0.12, table_height_ - 0.05);//pass.setFilterLimits(table_height_ - 0.05, table_height_ + block_size_ + 0.05);
    pass.setFilterLimits(table_height_ - 0.05 , table_height_ + block_size_ + 0.02 + 0.016); 
    pass.filter(*cloud_filtered);
    pcl::io::savePCDFile ("/home/mscv/Desktop/before_filtering.pcd", *cloud_filtered); // added by DNA
    if (cloud_filtered->points.size() == 0)
    {
      ROS_ERROR("0 points left");
      return;
    }
    else
      ROS_INFO("Filtered, %d points left", (int ) cloud_filtered->points.size());

    int nr_points = cloud_filtered->points.size ();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
      }

      std::cout << "Inliers: " << (inliers->indices.size()) << std::endl;

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: "
                << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
    }
    
    // Filtering second time
    pcl::io::savePCDFile ("/home/mscv/Desktop/filter_table.pcd", *cloud_filtered); // added by DNA
    /**********************************************************************************************/
    nr_points = cloud_filtered->points.size ();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
      }

      std::cout << "Inliers: " << (inliers->indices.size()) << std::endl;

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: "
                << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
    }
    
    
    /*******************************************************************************************/

    /*
     * We can add some stuff here to 
     * detect color DNA
     * */

    pcl::io::savePCDFile ("/home/mscv/Desktop/filter_robot.pcd", *cloud_filtered); // added by DNA

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    
    /* the original code
    ec.setClusterTolerance(0.003); 
    ec.setMinClusterSize(100);  
    ec.setMaxClusterSize(25000);
    */
    
    ec.setClusterTolerance(0.003);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(2000); // DNA: lower the maximum value for better searching the cube

    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pub_.publish(cloud_filtered);

    // std::cout << "Nbr of clusters : " << cluster_indices.size () << std::endl;  // DNA: checking
    // for each cluster, see if it is a block
    for (size_t c = 0; c < cluster_indices.size (); ++c)
    {  
      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;
      
      unsigned long redSum=0, greenSum=0, blueSum=0, pixelCount=0;

      for (size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
          int j = cluster_indices[c].indices[i];
          float x = cloud_filtered->points[j].x;
          float y = cloud_filtered->points[j].y;
          float z = cloud_filtered->points[j].z;
          unsigned long rg = cloud_filtered->points[j].rgba;
                 
          // Calculate avg color of cluster  
          pixelCount++;
          redSum += rg & 0xff;
          greenSum += (rg >> 8) & 0xff;
          blueSum += (rg >> 16) & 0xff;
          
          if (i == 0)
          {
            xmin = xmax = x;
            ymin = ymax = y;
            zmin = zmax = z;
          }
          else
          {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
          } 
          
      }
      
      
      
      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;
      
      /**************************DNA***********************************************************/
      //std::cout << "Cluster : " << c+1 << std::endl;
      //std::cout << "x_side = " << xside << "   y_side = " << yside << "   z_side = " << zside << std::endl;
    
      /*************************************************************************************/
      
      const float tol = 0.001; // 1 cm error tolerance
      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if (xside > block_size_ - tol && xside < block_size_*sqrt(2) + tol &&
          yside > block_size_ - tol && yside < block_size_*sqrt(2) + tol) //&&
          //zside > tol && zside < block_size_ + tol)   // DNA: comment it because the poor depth information is not as close as true value
      {
        // If so, then figure out the position and the orientation of the block
        float angle = atan(block_size_/((xside + yside)/2));
        
        if (yside < block_size_)
          angle = 0.0;
          
          // Then add it to our set
        ROS_INFO("   Found new block! x=%.3f y=%.3f z=%.3f", (float) xmin + xside/2.0, (float) ymin + (float) yside/2.0, zmax - block_size_/2.0);
        
        ROS_INFO_STREAM("  Block length x side: " << xside << "m y side: " << yside << "m z side " << zside << "m angle: " << angle);
        
        ROS_INFO("  Block color RGB  0x%x %x %x blue-red= 0x%x", redSum/pixelCount, greenSum/pixelCount, blueSum/pixelCount, (blueSum/pixelCount - redSum/pixelCount) );
        ROS_INFO("  blue-red target= 0x%x", blueRedDiff);
        
        xmin += xside/2.0;
        ymin += yside/2.0;
        zmax -= block_size_/2.0;
        /***************************************DNA****************************************************************/
        //std::cout << "For this cluster : x_min = " << xmin << "   y_min = " << ymin << "   z_max = " << zmax << std::endl;
        //std::cout << "Color for this cluster : redSum = " << redSum << "   blueSum = " << blueSum << "   greenSum = " << greenSum << std::endl;
        /*******************************************************************************************************/
        adjustBlock(&xmin, &ymin, &zmax);  // Fine tune beyond TF/calibrate 
        
        // TODO fix kluge to pass color match flag in zmax
        // TODO currently uses blue minus red to give appx hue. ignores green.  should use real HSV calc instead
        ulong dif = blueSum/pixelCount - redSum/pixelCount;


 /*       DNA: color difference between blue and red is useless
  *     if (dif < blueRedDiff ) {
             ROS_INFO("matched Target Color");
             zmax = COLOR_MATCH;}   // COLOR_MATCH is flag for a block matching target color
        else 
             {
             ROS_INFO("Does not match Target Color");
             zmax = NO_COLOR_MATCH;
             }    
*/   

        ROS_INFO("Adding a new block! x=%.3f y=%.3f z=%.3f", (float) xmin , (float) ymin , zmax );
        
        /******************************DNA********************************************************************/
        zmax = COLOR_MATCH;
        if (greenSum > blueSum && greenSum > redSum)  // a green cube is used in this project
           addBlock(xmin , ymin , zmax , angle, (unsigned long) blueSum/pixelCount);
        /**************************************************************************************************/
        //if (xmin > .11 && xmin < .29)  // TODO - remove hardcoded validation for x
        //   addBlock(xmin , ymin , zmax , angle, (unsigned long) blueSum/pixelCount);
      }
    }
    
    if (result_.blocks.poses.size() > 0)
    {
      as_.setSucceeded(result_);
      block_pub_.publish(result_.blocks);
      ROS_INFO("[block detection] Succeeded!");
    }
    else
      ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
    //  as_.setAborted(result_);
  }


private:


  void addBlock(float x, float y, float z, float angle, unsigned long color)
  {
    geometry_msgs::Pose block_pose;
    std_msgs::ColorRGBA rgb;
    
    rgb.r = color;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;
    
    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));
    
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();
    
    result_.blocks.poses.push_back(block_pose);
    // TODO use message to pass color     result_.colors.push_back(rgb);
  }

  void addTable()
  {
    // Add the table as a collision object into the world, so it gets excluded from the collision map
    // As the blocks are small, they should also be excluded (assuming that padding_offset parameter on
    // octomap sensor configuration is equal or bigger than block size)
    double table_size_x = 0.5; // table_size
    double table_size_y = 1.0;
    double table_size_z = 0.05;
    //double table_size_z = 0;

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = arm_link_;

    co.id = "table";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, co.id));

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = table_size_x;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = table_size_y;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = table_size_z;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = table_pose_[0] + table_size_x/2.0;
    co.primitive_poses[0].position.y = table_pose_[1];
    co.primitive_poses[0].position.z = table_pose_[2] - table_size_z/2.0;

    ROS_INFO("Add the table as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, co);
    planning_scene_interface_.addCollisionObjects(collision_objects);
  }
  
  
/*** 
 CALIBRATION - this is a kluge to correct for errors in the
   turtlebot_arm_calibration routine.  If you can fix the calibration routine that 
   would be far better although the manual calibration below only takes about 30 minutes.  
 
   1. run turtlebot_arm_calibration. Based on that you need to launch
        a "tf" using the values returned
   2. I found that the above didnt calibrate perfectly.  adjustBlock does
   further calibration based on physical measurements as below:
   3. Draw a 7x7 grid with 2cm for each box in the working area of the arm
   4. Place a 2cm cube on the first of the boxes. 
   5. set z offset so it will be above table and cube
   5a.  Launch block detection
   6. Press "D" for block detect "D" and then press "M" for block move 
   7. when the arm goes to Pick the block, measure and record the X, Y and Z error
      and the location of the block as reported.  Be careful to get sign correct. 
   8. you will need to repeat this for a sample of boxes
      covering the working area.  
   9. Case Y adjustment.  There are 3 cases below for Y based on a range for x.
       For each of the 3 cases below, you will need to find polynomial 
       coefficients that gives a close match.  
       
       Case Y1 - Take the measurements you made where x <= 0.19.  
       
       Enter the x,y pairs for 4 or so measurements into:
          http://www.wolframalpha.com/input/?i=polynomial+interpolation&lk=4
          
       This will give you the polynomial interpolation for your data.
       
       Update the a,b,c,d coefficients for Case Y1 below based on that result   
       
       Repeat for x between 0.19f and 0.23 for Case Y2 coefficients
       Repeat for x>0.23 for Case Y3 coefficients
       
   10. I found X was off a set amount depending on X. Follow similar procedure except you should use Wolfram's linear interpolation and enter results into Case X
   11. I found Z was off a set amount depending on X.  Follow same procedure except you should use Wolfram's linear interpolation and enter results into Case Z
   12. Once you have the adjustments set, change the end of the adjustBlock routine 
   to be "if (false)" so that the adjustments are used.
   
   ***** remove the space between the asterisk and slash below.
*/

  void adjustBlock(float *x, float *y, float *z)
  {
  float a, b, c, d;
  // Misalignment is physically measured. Trendline best fit formula as below
  
  // TODO - you must change the adjustment factors below for case X, Y and Z
  
  // Case X - X adjustment factor
  x_adj = .155 * (*x) - .06;

  // Case Z - Z adjustment factor
  *z = 0.125 * (*x) -0.085;
  
  ROS_INFO("## x=%.4f y=%.4f ##", *x, *y);
  
  // Case Y1 - Adjustments for Y
  if ( (*x) <= 0.19f) {
  // polynomial coefficients
  // 24.1147 x^3 - 2.08714 x^2 + 0.138863 x - 0.00626978
  a=24.1147;
  b=2.08714;
  c=0.138863;
  d=0.00626978;
  }
  
  // Case Y2 - Adjustments for Y
  if ( (*x) > 0.19f && (*x) <= 0.23f) {
  // polynomial coefficients
  // 20.8859 x^3 - 1.94219 x^2 + 0.163624 x - 0.00544517
  a=20.8859;
  b=1.94219;
  c=0.163624;
  d=0.00544517;
    }
  
  // Case Y3 - Adjustments for Y
  if (*x > 0.23f) {
  // polynomial coefficients
  // 14.2045 x^3 - 1.0579 x^2 + 0.136905 x - 0.00219481
  a=14.2045;
  b=1.0579;
  c=0.136905;
  d=0.00219481;
  }
  
  // Use above polynomial coefficients to adjust Y
  y_adj = (a *  pow((*y),3) ) - (b * pow((*y),2)) + (c * (*y)) - d;
  
  ROS_INFO("y=%.4f cub=%.4f sqr=%.4f pow=%.4f",(*y),  a *  pow((*y),3) , (b * pow((*y),2)) , (c * (*y)) );
  
  ROS_INFO("X Adj=%.4f Y Adj=%.4f Z Adj=%.4f", (float) x_adj, (float) y_adj, (float) z_adj);
  
  // Should we disable adjustment?
  if (true) {
  ROS_INFO("adjustBlock IS DISABLED!");
  }
  else {
    *x += (float) x_adj;   
    *y += (float) y_adj;
    }
    
    
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_detection_action_server");

  turtlebot_arm_block_manipulation::BlockDetectionServer server("block_detection");
  ros::spin();

  return 0;
}

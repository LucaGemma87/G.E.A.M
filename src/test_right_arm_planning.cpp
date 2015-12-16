
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <ios>
#include <iostream>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "grasp_estimator/ArmPlanning.h"


// ROS headers
#include <ros/ros.h>

// Moveit headers

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>



/*! Simply pings the cylinder fitting service and prints out the result.*/
int main(int argc, char **argv)
{  


  ros::init(argc, argv, "ping_right_arm_planning");
  ros::NodeHandle nh;

  std::string service_name("right_arm_planning_srv");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  
  grasp_estimator::ArmPlanning right_arm_planning_srv;
  
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);
  
 
  
  geometry_msgs::PoseStamped desidered_pose;
  // hardcode position and orientation
  desidered_pose.pose.position.x=-1.055369+0.10;//-1,057152
  desidered_pose.pose.position.y=0.624553-0.10;//0,108687
  desidered_pose.pose.position.z=0.212426+0.1; //1,112701
  desidered_pose.pose.orientation.x=0.560326; 
  desidered_pose.pose.orientation.y=-0.434411;
  desidered_pose.pose.orientation.z=-0.418089; 
  desidered_pose.pose.orientation.w=0.567912;






    // desidered_pose.pose.orientation.x=0.56029;
    // desidered_pose.pose.orientation.y=-0.43442; 
    // desidered_pose.pose.orientation.z=-0.41801; 
    // desidered_pose.pose.orientation.w=0.56799; 
    // desidered_pose.pose.position.x=-0.872592-0.01; 
    // desidered_pose.pose.position.y=0.67825-0.01; 
    // desidered_pose.pose.position.z=0.20653-0.01;

  desidered_pose.header.frame_id="vito_anchor";
  ros::Time now_time = ros::Time::now();
  //desidered_pose.stamp.secs=now_time.stamp.secs;


  
  right_arm_planning_srv.request.goal.des_pose=desidered_pose;
     

  int decision_grasp = 1;
  right_arm_planning_srv.request.goal.decision=(int)decision_grasp;
  //right_arm_planning_srv.request.goal.arm_name="right_arm";

  ROS_INFO("right_arm_planning  waiting for service on topic right_arm_planning_srv");
  
  if (!ros::service::call(service_name, right_arm_planning_srv))
  {
    ROS_ERROR("Call to right_arm_planning_srv service failed");
    exit(0);
  }
  ROS_INFO("right_arm_planning  recivint results for service on topic right_arm_planning_srv");
  if (right_arm_planning_srv.response.result != right_arm_planning_srv.response.SUCCESS)
    {
     ROS_ERROR("right_arm_planning_srv returned error %d", right_arm_planning_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    float error;
    result= right_arm_planning_srv.response.result;
    error= right_arm_planning_srv.response.error;
    ROS_INFO("Result: %f,Error  %f", (float)result, (float)error);
    ROS_INFO("ping_right_arm_planning Done !!!");
    
  }
 return true;
};


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


int main(int argc, char **argv)
{  

  ros::init(argc, argv, "ping_left_arm_planning");
  ros::NodeHandle nh;

  std::string service_name("left_arm_planning_srv");

  grasp_estimator::ArmPlanning left_arm_planning_srv;
  int    joint_change_position[7];
  float  joint_position[7];
  int decision = 3;

  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

    joint_change_position[0] = 1; 
  joint_change_position[1] = 0;
  joint_change_position[2] = 0;
  joint_change_position[3] = 0;
  joint_change_position[4] = 0;
  joint_change_position[5] = 0;
  joint_change_position[6] = 1;
  
  joint_position[0] =  -0.2; 
  joint_position[1] =  0; 
  joint_position[2] =  0; 
  joint_position[3] =  0; 
  joint_position[4] =  0; 
  joint_position[5] =  0; 
  joint_position[6] =  0.5;

  for( int i = 0; i < 7; ++i)
  {
    left_arm_planning_srv.request.goal.joint_change_position[i]=joint_change_position[i];
    left_arm_planning_srv.request.goal.joint_position[i]=joint_position[i];
    //ROS_INFO("Joint change position %f : %f",float(i),float(joint_change_position[i]));
    //ROS_INFO("Joint position %f : %f",float(i),float(joint_position[i]));
  }
 
  left_arm_planning_srv.request.goal.decision=(int)decision;
   left_arm_planning_srv.request.goal.arm_name="left_arm";
  
  ROS_INFO("left_arm_planning  waiting for service on topic left_arm_planning_srv");
  
  if (!ros::service::call(service_name, left_arm_planning_srv))
  {
    ROS_ERROR("Call to left_arm_planning_srv service failed");
    exit(0);
  }
  ROS_INFO("left_arm_planning  recivint results for service on topic left_arm_planning_srv");
  if (left_arm_planning_srv.response.result != left_arm_planning_srv.response.SUCCESS)
    {
     ROS_ERROR("left_arm_planning_srv returned error %d", left_arm_planning_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    float error;
    result= left_arm_planning_srv.response.result;
    error= left_arm_planning_srv.response.error;
    ROS_INFO("Result: %f,Error  %f", (float)result, (float)error);
    ROS_INFO("ping_left_arm_planning Done !!!");
    
  }

  joint_change_position[0] = 1; 
  joint_change_position[1] = 0;
  joint_change_position[2] = 0;
  joint_change_position[3] = 0;
  joint_change_position[4] = 0;
  joint_change_position[5] = 0;
  joint_change_position[6] = 1;
  
  joint_position[0] =  0.6; 
  joint_position[1] =  0; 
  joint_position[2] =  0; 
  joint_position[3] =  0; 
  joint_position[4] =  0; 
  joint_position[5] =  0; 
  joint_position[6] =  0;

  for( int i = 0; i < 7; ++i)
  {
    left_arm_planning_srv.request.goal.joint_change_position[i]=joint_change_position[i];
    left_arm_planning_srv.request.goal.joint_position[i]=joint_position[i];
    //ROS_INFO("Joint change position %f : %f",float(i),float(joint_change_position[i]));
    //ROS_INFO("Joint position %f : %f",float(i),float(joint_position[i]));
  }
 
  left_arm_planning_srv.request.goal.decision=(int)decision;
   left_arm_planning_srv.request.goal.arm_name="left_arm";
  
  ROS_INFO("left_arm_planning  waiting for service on topic left_arm_planning_srv");
  
  if (!ros::service::call(service_name, left_arm_planning_srv))
  {
    ROS_ERROR("Call to left_arm_planning_srv service failed");
    exit(0);
  }
  ROS_INFO("left_arm_planning  recivint results for service on topic left_arm_planning_srv");
  if (left_arm_planning_srv.response.result != left_arm_planning_srv.response.SUCCESS)
    {
     ROS_ERROR("left_arm_planning_srv returned error %d", left_arm_planning_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    float error;
    result= left_arm_planning_srv.response.result;
    error= left_arm_planning_srv.response.error;
    ROS_INFO("Result: %f,Error  %f", (float)result, (float)error);
    ROS_INFO("ping_left_arm_planning Done !!!");
    
  }


  sleep(3.0);

  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  joint_change_position[0] = 0; 
  joint_change_position[1] = 0;
  joint_change_position[2] = 0;
  joint_change_position[3] = 0;
  joint_change_position[4] = 0;
  joint_change_position[5] = 0;
  joint_change_position[6] = 1;
  
  joint_position[0] =  0; 
  joint_position[1] =  0; 
  joint_position[2] =  0; 
  joint_position[3] =  0; 
  joint_position[4] =  0; 
  joint_position[5] =  0; 
  joint_position[6] =  -0.5;

  for( int i = 0; i < 7; ++i)
  {
    left_arm_planning_srv.request.goal.joint_change_position[i]=joint_change_position[i];
    left_arm_planning_srv.request.goal.joint_position[i]=joint_position[i];
    //ROS_INFO("Joint change position %f : %f",float(i),float(joint_change_position[i]));
    //ROS_INFO("Joint position %f : %f",float(i),float(joint_position[i]));
  }
 
  left_arm_planning_srv.request.goal.decision=(int)decision;
  left_arm_planning_srv.request.goal.arm_name="left_arm";
  ROS_INFO("left_arm_planning  waiting for service on topic left_arm_planning_srv");
  
  if (!ros::service::call(service_name, left_arm_planning_srv))
  {
    ROS_ERROR("Call to left_arm_planning_srv service failed");
    exit(0);
  }
  ROS_INFO("left_arm_planning  recivint results for service on topic left_arm_planning_srv");
  if (left_arm_planning_srv.response.result != left_arm_planning_srv.response.SUCCESS)
    {
     ROS_ERROR("left_arm_planning_srv returned error %d", left_arm_planning_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    float error;
    result= left_arm_planning_srv.response.result;
    error= left_arm_planning_srv.response.error;
    ROS_INFO("Result: %f,Error  %f", (float)result, (float)error);
    ROS_INFO("ping_left_arm_planning Done !!!");
    
  }
  // per tornare in home position

  sleep(5.0);

  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);
  int    joint_change_position_home[7];
  joint_change_position_home[0] = 1; 
  joint_change_position_home[1] = 1;
  joint_change_position_home[2] = 1;
  joint_change_position_home[3] = 1;
  joint_change_position_home[4] = 1;
  joint_change_position_home[5] = 1;
  joint_change_position_home[6] = 1;
  
  float  joint_position_home[7];
   joint_position_home[0] =  -0.2; 
  joint_position_home[1] =  -0.4; 
  joint_position_home[2] =  0; 
  joint_position_home[3] =  -1.5; 
  joint_position_home[4] =  -0.6; 
  joint_position_home[5] =  0.1; 
  joint_position_home[6] =  -2;

  for( int i = 0; i < 7; ++i)
  {
    left_arm_planning_srv.request.goal.joint_change_position[i]=joint_change_position_home[i];
    left_arm_planning_srv.request.goal.joint_position[i]=joint_position_home[i];
    //ROS_INFO("Joint change position %f : %f",float(i),float(joint_change_position[i]));
    //ROS_INFO("Joint position %f : %f",float(i),float(joint_position[i]));
  }
  int decision_home = 3;
  left_arm_planning_srv.request.goal.decision=(int)decision_home;
   left_arm_planning_srv.request.goal.arm_name="left_arm"; 
  ROS_INFO("left_arm_planning  waiting for service on topic left_arm_planning_srv");
  
  if (!ros::service::call(service_name, left_arm_planning_srv))
  {
    ROS_ERROR("Call to left_arm_planning_srv service failed");
    exit(0);
  }
  ROS_INFO("left_arm_planning  recivint results for service on topic left_arm_planning_srv");
  if (left_arm_planning_srv.response.result != left_arm_planning_srv.response.SUCCESS)
    {
     ROS_ERROR("left_arm_planning_srv returned error %d", left_arm_planning_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    float error;
    result= left_arm_planning_srv.response.result;
    error= left_arm_planning_srv.response.error;
    ROS_INFO("Result: %f,Error  %f", (float)result, (float)error);
    ROS_INFO("ping_left_arm_planning Done !!!");
    
  }
 return true;
};

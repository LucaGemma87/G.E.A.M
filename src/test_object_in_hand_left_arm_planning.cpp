
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

#define n_joints 8


int main(int argc, char **argv)
{  

  ros::init(argc, argv, "ping_left_arm_planning");
  ros::NodeHandle nh;

  ros::Time start_time = ros::Time::now();
  std::string topic = nh.resolveName("/left_arm/joint_states");
  ROS_INFO(" Waiting for left arm joint state on topic %s", topic.c_str());
  
  
sensor_msgs::JointState::ConstPtr CurrentState_ConstPtr;
while(!CurrentState_ConstPtr) {
        CurrentState_ConstPtr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, nh, ros::Duration(3.0));
        ROS_INFO("wait for joint states!");
      }
   ROS_INFO(" Received state is not empty for left arm joint state on topic %s", topic.c_str());
  ROS_INFO(" Received state for left arm joint state on topic %s", topic.c_str());
  
    ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[0])); 
     ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[1]));
    ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[2]));
     ROS_INFO("Joint startposition %f",float(CurrentState_ConstPtr->position[3]));
   ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[4]));
     ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[5]));
     ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[6])); 
     ROS_INFO("Joint start position %f",float(CurrentState_ConstPtr->position[7]));
  ROS_INFO("Start position received");

  std::string service_name("left_arm_planning_srv");

  grasp_estimator::ArmPlanning left_arm_planning_srv;
  float  joint_position[n_joints];
  int decision = 3;

  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  joint_position[0] =  CurrentState_ConstPtr->position[0]+0.2; 
  joint_position[1] =  CurrentState_ConstPtr->position[1]; 
  joint_position[2] =  CurrentState_ConstPtr->position[2]; 
  joint_position[3] =  CurrentState_ConstPtr->position[3]; 
  joint_position[4] =  CurrentState_ConstPtr->position[4]; 
  joint_position[5] =  CurrentState_ConstPtr->position[5]; 
  joint_position[6] =  CurrentState_ConstPtr->position[6];
  joint_position[7] =  CurrentState_ConstPtr->position[7];

      ROS_INFO("Joint desidered position %f",float(joint_position[0])); 
     ROS_INFO("Joint desidered position %f",float(joint_position[1]));
    ROS_INFO("Joint desidered position %f",float(joint_position[2]));
     ROS_INFO("Joint desidered position %f",float(joint_position[3]));
   ROS_INFO("Joint desidered position %f",float(joint_position[4]));
     ROS_INFO("Joint desidered position %f",float(joint_position[5]));
     ROS_INFO("Joint desidered position %f",float(joint_position[6])); 
     ROS_INFO("Joint desidered position %f",float(joint_position[7]));


  for( int i = 0; i < n_joints; ++i)
  {

    left_arm_planning_srv.request.goal.joint_position[i]=joint_position[i];
    
  }
 
  left_arm_planning_srv.request.goal.decision=(int)decision;
  left_arm_planning_srv.request.goal.arm_name="left_hand_arm";
  
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

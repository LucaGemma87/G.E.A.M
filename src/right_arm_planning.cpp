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

// Control Headers

#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"
#include "ros/macros.h"
#include "ros/assert.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"



namespace grasp_estimator {

class Right_arm_planner
{ //typedef pcl::PointXYZ    Point;
 
  private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher right_arm_planning_pub_ ;
  //! Publisher for controller
  ros::Publisher right_arm_planning_controller_pub_ ;

  //! Service server for planning 
  ros::ServiceServer right_arm_planning_srv_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

   
  //------------------ Callbacks -------------------

  
  //! Callback for service calls
  bool serviceCallback(ArmPlanning::Request &request, ArmPlanning::Response &response);
  
  public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  Right_arm_planner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
   {
     
 
     right_arm_planning_srv_ = nh_.advertiseService(nh_.resolveName("right_arm_planning_srv"), &Right_arm_planner::serviceCallback, this);
     
     right_arm_planning_controller_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/right_arm/joint_trajectory_controller/follow_joint_trajectory/goal", 1, true);
     //right_arm_planning_controller_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/right_arm/joint_trajectory_controller/follow_joint_trajectory/feedback", 1, true);
    
     right_arm_planning_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    }
  //! Empty stub
  ~Right_arm_planner() {}
};

bool Right_arm_planner::serviceCallback(grasp_estimator::ArmPlanning::Request &request, grasp_estimator::ArmPlanning::Response &response)
{ 
  grasp_estimator::ArmPlanning service_request;
  service_request.request.goal=request.goal;	
  // Setup MoveIt enviroment
  moveit::planning_interface::MoveGroup group("right_arm");  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  moveit::planning_interface::MoveGroup::Plan simple_plan; 
  moveit_msgs::DisplayTrajectory display_trajectory;
  group.allowReplanning ("true");
  group.setPlannerId("PRMstarkConfigDefault");
  group.setPlanningTime((double)1);
  group.setGoalOrientationTolerance((double)0.05); 
  group.setGoalPositionTolerance((double)0.05); 
  //moveit_msgs::RobotTrajectory trajectory;  
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose target_pose;

  // Create useful variables
  geometry_msgs::PoseStamped des_position;
  std::vector<geometry_msgs::Pose> waypoints;

  // Trajectory controller 
  trajectory_msgs::JointTrajectory trajectory;
  control_msgs::FollowJointTrajectoryActionGoal trajectory_controller_msgs;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
   
  if(service_request.request.goal.decision==(int)1)
  { 
  	// checking and asking home trasform 
  	double pi(3.1415);
  	double threshold_distance((double)0.02);
  	ROS_INFO("Listening for world_link_2_wrist_stamped");
  	tf::StampedTransform world_link_2_wrist_stamped;
  	ros::Time now0 = ros::Time::now();
  	listener_.waitForTransform("vito_anchor","right_arm_7_link", now0, ros::Duration(4));
  	listener_.lookupTransform("vito_anchor","right_arm_7_link", now0, world_link_2_wrist_stamped);
  	tf::Transform world_link_2_wrist;
  	world_link_2_wrist.setOrigin(world_link_2_wrist_stamped.getOrigin());
  	world_link_2_wrist.setBasis(world_link_2_wrist_stamped.getBasis());
  	tf::Vector3 position_wrist_home = world_link_2_wrist.getOrigin();
  	tf::Quaternion quaternion_wrist_home = world_link_2_wrist.getRotation();
  	tf::Transform wrist_pose;
  	wrist_pose.setOrigin(position_wrist_home);
  	wrist_pose.setRotation(quaternion_wrist_home);
  	  
  	tf::Matrix3x3 rotation_wrist_home;
  	rotation_wrist_home.setRotation(quaternion_wrist_home);
  	  
  	double roll_wrist, pitch_wrist, yaw_wrist;
  	  
  	rotation_wrist_home.getRPY(roll_wrist, pitch_wrist, yaw_wrist);
  	  
  	ROS_INFO("RPY_wrist = (%f, %f, %f)", roll_wrist, pitch_wrist, yaw_wrist);
  	ROS_INFO ("Right_arm_planner_srv:Success in receving wrist transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",quaternion_wrist_home[0],quaternion_wrist_home[1],quaternion_wrist_home[2],quaternion_wrist_home[3],position_wrist_home[0],position_wrist_home[1],position_wrist_home[2]);
  	  
  	// Setup moveit start position
  	start_pose.orientation.x = quaternion_wrist_home[0];
  	start_pose.orientation.y = quaternion_wrist_home[1];
  	start_pose.orientation.z = quaternion_wrist_home[2];
  	start_pose.orientation.w = quaternion_wrist_home[3];

  	start_pose.position.x = position_wrist_home[0];
  	start_pose.position.y = position_wrist_home[1];
  	start_pose.position.z = position_wrist_home[2];
    // Receving desiderate position
    des_position=request.goal.des_pose;
     
    // check if the desidered pose is empty
    if( des_position.pose.orientation.x==0 && des_position.pose.orientation.y==0 && des_position.pose.orientation.z==0 && des_position.pose.orientation.w==0 && des_position.pose.position.x==0 &&  des_position.pose.position.y==0 &&  des_position.pose.position.z==0)
    {
      ROS_ERROR("Desidered Position is empty");
      response.result = response.NO_DES_POS_RECEIVED;
    }
    ROS_INFO ("Right_arm_planner_srv:Success in receving desidered wrist transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",des_position.pose.orientation.x,des_position.pose.orientation.y,des_position.pose.orientation.z,des_position.pose.orientation.w,des_position.pose.position.x,des_position.pose.orientation.y,des_position.pose.orientation.z);
      
    tf::Transform des_wrist_pose(tf::Quaternion(des_position.pose.orientation.x, des_position.pose.orientation.y, des_position.pose.orientation.z, des_position.pose.orientation.w), tf::Vector3( des_position.pose.position.x,  des_position.pose.position.y,  des_position.pose.position.z));
    
    // Get desidered wrist pose  
    //tf::Transform des_wrist_pose;
    //tf::poseMsgToTF(des_position,des_wrist_pose); 
    
    tf::Matrix3x3 des_rotation_wrist = des_wrist_pose.getBasis();
    tf::Vector3 des_position_wrist = des_wrist_pose.getOrigin();
    
    
    tf::Quaternion des_orientation_wrist;
    des_rotation_wrist.getRotation(des_orientation_wrist);
    // check if the resulting orientation is correct
    if (des_rotation_wrist.determinant() < 0.5 )
    {
      ROS_ERROR("Desidered Position is bad Posed");
      response.result = response.DES_POS_BAD_POSE;
    }
    ROS_INFO("Desidered Position Received");
    target_pose.position.x = des_position.pose.position.x; 
    target_pose.position.y = des_position.pose.position.y;
    target_pose.position.z = des_position.pose.position.z;
    target_pose.orientation.x = des_position.pose.orientation.x;
    target_pose.orientation.y = des_position.pose.orientation.y;
    target_pose.orientation.z = des_position.pose.orientation.z;
    target_pose.orientation.w = des_position.pose.orientation.w;
    //target_pose.orientation.w = 1;
    //group.(target_pose); 
    ROS_INFO("Starting wrist planning for right arm");


    ROS_INFO("Setting Pose Target");
    group.setPoseTarget(target_pose);
    ROS_INFO("Planning position Target");
    group.setPlanningTime((double)5);
    group.setGoalOrientationTolerance((double)0.05); 
    group.setGoalPositionTolerance((double)0.05); 
    
    
    bool success = group.plan(simple_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
       
       
    ROS_INFO("Visualizing simple plan ");
    display_trajectory.trajectory_start = simple_plan.start_state_;
    display_trajectory.trajectory.push_back(simple_plan.trajectory_);
    right_arm_planning_pub_.publish(display_trajectory);        
    //      Sleep to give Rviz time to visualize the plan. 
    if (success==(bool)1)
    {
      group.move();
      ROS_INFO("Robot Move DONE");
      response.result=response.SUCCESS;
      response.error = (float)0;
    }
    else
    {
      response.result=response.OTHER_ERROR;
      response.error = (float)0;
    }  
            
  }

  if(service_request.request.goal.decision==(int)3)
  { 
    ROS_INFO("Starting wrist planning for right arm");
  	std::vector<double> group_variable_values;
    //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values); 
    
      ros::Time start_time = ros::Time::now();
      std::string topic = nh_.resolveName("/right_arm/joint_states");
      ROS_INFO(" Waiting for right arm joint state on topic %s", topic.c_str());
      
      sensor_msgs::JointState::ConstPtr CurrentState_ConstPtr =
      ros::topic::waitForMessage<sensor_msgs::JointState>(topic, nh_, ros::Duration(3.0));
      if(!CurrentState_ConstPtr) ROS_ERROR("empty joint states!");
       else ROS_INFO(" Received state is not empty for right arm joint state on topic %s", topic.c_str());
      ROS_INFO(" Received state for right arm joint state on topic %s", topic.c_str());
      
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[0])); 
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[1]));
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[2]));
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[3]));
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[4]));
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[5]));
        // ROS_INFO("Joint position %f",float(CurrentState_ConstPtr->position[6])); 
      ROS_INFO("Start position received");
      float  des_joint_position[7]; 
      int    joint_change_position[7];
      group_variable_values=CurrentState_ConstPtr->position;
      int count =0;
      for( int i = 0; i < 7; ++i){
      	des_joint_position[i]=request.goal.joint_position[i];
        joint_change_position[i]=request.goal.joint_change_position[i];
      }
      //     int count=0;
     for( int i = 0; i < 7; ++i)
     { //ROS_INFO("Cycle %f",float(i));
          if (joint_change_position[i] ==(int)1)
          {
       	 
              group_variable_values[i] = des_joint_position[i];
            count= count=+1;
          }
      
           
         
     }

     if (count==7)
          {
               ROS_ERROR("joint change position is empty");
               response.result = response.DES_CHANGE_JOINT_EMPTY;
          }  	
    

    ROS_INFO("Setting joint value target done");
    group.setJointValueTarget(group_variable_values);
    //sleep(5.0);
     group.move();// questo move group blocca il programma
    response.result=response.SUCCESS;
    response.error = (float)0;
    //group.move();
    ROS_INFO("Planning Done");
    
   
    
      ROS_INFO("Visualizing simple plan  (again)");
      display_trajectory.trajectory_start = simple_plan.start_state_;
      display_trajectory.trajectory.push_back(simple_plan.trajectory_);
      right_arm_planning_pub_.publish(display_trajectory);
     
      //ROS_INFO("First trajectory Point %f",trajectory_controller_msgs.goal.trajectory.points[0]);
     //;
      
     //display_trajectory.trajectory();
     // 00034   , path_tolerance(_alloc)
     // 00035   , goal_tolerance(_alloc)
     // 00036   , goal_time_tolerance()
      //right_arm_planning_controller_pub_.publish(trajectory_controller_msgs);
   
  }
  
  ROS_INFO(" All is DONE !!! ");
}



} //namespace grasp_estimator

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "right_arm_planning_node");
  ROS_INFO("Right arm planner is Here!!!");
  ros::NodeHandle nh;
 
  grasp_estimator::Right_arm_planner node(nh);

  //ros::spin();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
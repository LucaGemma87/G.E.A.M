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

class Right_hand_planner
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
  moveit::planning_interface::MoveGroup group(service_request.request.goal.arm_name);  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  moveit::planning_interface::MoveGroup::Plan simple_plan; 
  moveit_msgs::DisplayTrajectory display_trajectory;
  group.allowReplanning ("true");
  group.setPlannerId("PRMstarkConfigDefault");
  group.setPlanningTime((double)1);
  group.setGoalOrientationTolerance((double)0.005); 
  group.setGoalPositionTolerance((double)0.005); 
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
  ros::AsyncSpinner spinner(8); // Use 8 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
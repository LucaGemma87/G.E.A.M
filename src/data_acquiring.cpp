
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <ios>
#include <iostream>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "grasp_estimator/DataAcquiring.h"

// ROS headers
#include <ros/ros.h>

// ROS Control
#include "control_msgs/JointTrajectoryControllerState.h"


namespace grasp_estimator {

class Data_Logger
{ //typedef pcl::PointXYZ    Point;
 
  private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher data_logger_pub_ ;

  //! Service server for planning 
  ros::ServiceServer data_logger_srv_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

   
  //------------------ Callbacks -------------------

  
  //! Callback for service calls
  bool serviceCallback(DataAcquiring::Request &request, DataAcquiring::Response &response);
  
  public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  Data_Logger(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
   {
     
 
     data_logger_srv_ = nh_.advertiseService(nh_.resolveName("data_logger_srv"), &Data_Logger::serviceCallback, this);
 
  
    
    }
  //! Empty stub
  ~Data_Logger() {}
};

bool Data_Logger::serviceCallback(grasp_estimator::DataAcquiring::Request &request, grasp_estimator::DataAcquiring::Response &response)
{
  grasp_estimator::DataAcquiring data_request;
  data_request.request.activation=request.activation;
  
  // Tf initilization
  tf::StampedTransform wrist_2_right_hand_index_distal_link_stamped,    wrist_2_right_hand_index_knuckle_link_stamped,   wrist_2_right_hand_index_middle_link_stamped,     wrist_2_right_hand_index_proximal_link_stamped;
  tf::StampedTransform wrist_2_right_hand_little_distal_link_stamped,   wrist_2_right_hand_little_knuckle_link_stamped,  wrist_2_right_hand_little_middle_link_stamped,    wrist_2_right_hand_little_proximal_link_stamped;  
  tf::StampedTransform wrist_2_right_hand_middle_distal_link_stamped,   wrist_2_right_hand_middle_knuckle_link_stamped,  wrist_2_right_hand_middle_middle_link_stamped,    wrist_2_right_hand_middle_proximal_link_stamped; 
  tf::StampedTransform wrist_2_right_hand_ring_distal_link_stamped,     wrist_2_right_hand_ring_knuckle_link_stamped,    wrist_2_right_hand_ring_middle_link_stamped,      wrist_2_right_hand_ring_proximal_link_stamped;
  tf::StampedTransform wrist_2_right_hand_thumb_distal_link_stamped,    wrist_2_right_hand_thumb_knuckle_link_stamped,   wrist_2_right_hand_thumb_proximal_link_stamped;
  // Tf msgs initialization
  geometry_msgs::TransformStamped right_hand_index_distal_link_msgs,  right_hand_index_knuckle_link_msgs,  right_hand_index_middle_link_msgs,  right_hand_index_proximal_link_msgs;
  geometry_msgs::TransformStamped right_hand_little_distal_link_msgs, right_hand_little_knuckle_link_msgs, right_hand_little_middle_link_msgs, right_hand_little_proximal_link_msgs;
  geometry_msgs::TransformStamped right_hand_middle_distal_link_msgs, right_hand_middle_knuckle_link_msgs, right_hand_middle_middle_link_msgs, right_hand_middle_proximal_link_msgs;
  geometry_msgs::TransformStamped right_hand_ring_distal_link_msgs,   right_hand_ring_knuckle_link_msgs,   right_hand_ring_middle_link_msgs,   right_hand_ring_proximal_link_msgs;
  geometry_msgs::TransformStamped right_hand_thumb_distal_link_msgs,  right_hand_thumb_knuckle_link_msgs,  right_hand_thumb_proximal_link_msgs;
  // Motor position initialization
  control_msgs::JointTrajectoryControllerState::ConstPtr Hand_Synergy_state_ConstPtr;
  double right_hand_synergy_joint_state_position;
  double right_hand_synergy_joint_state_velocity;
  double right_hand_synergy_joint_state_effort;
  double current_error;
  //double max_ticks = 15000;
  // Wrench right arm initialization
  geometry_msgs::WrenchStamped::ConstPtr right_arm_wrench_stamped_ConstPtr;



  ROS_INFO("Request received");
  if(data_request.request.activation==(int)1)
  {
    ROS_INFO("Starting data acquiring");
 
    ///// acquiring joint right hand
    ROS_INFO("Starting acquiring joint state right hand ");
    ros::Time start_time = ros::Time::now();
    // uint64_t start_acq;           
    // start_time.fromNSec(start_acq);
    //ROS_INFO("Start time %f",(float)start_acq);
    std::string topic_1 = nh_.resolveName("/right_hand/joint_states");
    ROS_INFO(" Waiting for right hand joint state on topic %s", topic_1.c_str());
    sensor_msgs::JointState::ConstPtr CurrentState_Hand_ConstPtr = 
    ros::topic::waitForMessage<sensor_msgs::JointState>(topic_1, nh_, ros::Duration(3.0));
    if(!CurrentState_Hand_ConstPtr) {
 	    ROS_ERROR("Empty joint states!");
      response.result=response.ERROR_SERVER;
    }
    else {ROS_INFO("Joint State acquired");}
    //ROS_INFO("Joint position %f",float(CurrentState_Hand_ConstPtr->position[0]));

    ////// Position motor right hand:

    // Now for simulation I use topic /right_hand/joint_trajectory_controller/state with  actual: positions: [value]
    // then I remap to motor tick from 0 (open hand) to 15000 (closed hand) linearly.
    // In a real scenario I will use hand position motor sensor (change the topic and eliminate linear remapping).
    std::string topic_2 = nh_.resolveName("/right_hand/joint_trajectory_controller/state");
    ROS_INFO(" Waiting for right Hand_Synergy_state %s", topic_2.c_str());
    Hand_Synergy_state_ConstPtr=ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(topic_2, nh_, ros::Duration(3.0));
    if(!Hand_Synergy_state_ConstPtr) {
      ROS_ERROR("Empty Hand_Synergy_state!");
      response.result=response.ERROR_SERVER;
    }
    else {ROS_INFO("Hand_Synergy_state Done");
    current_error=Hand_Synergy_state_ConstPtr->error.positions[0];
    //ROS_INFO("motor_error = %f",current_error);
    }
    // motor_position_sim=max_ticks*Current_Synergy_state;
    // ROS_INFO("motor_position_sim = %f",motor_position_sim);
    right_hand_synergy_joint_state_position=CurrentState_Hand_ConstPtr->position[28]; 
    right_hand_synergy_joint_state_velocity=CurrentState_Hand_ConstPtr->velocity[28]; 
    right_hand_synergy_joint_state_effort=CurrentState_Hand_ConstPtr->effort[28]; 
    


    //// acquiring tf Pose of right hand
    ROS_INFO("Starting acquiring tf pose right hand "); 
    ros::Time now1 = ros::Time::now();
    // right_hand_index_distal_link
    listener_.waitForTransform("right_arm_7_link","right_hand_index_distal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_index_distal_link",now1, wrist_2_right_hand_index_distal_link_stamped);
    //right_hand_index_knuckle_link
    listener_.waitForTransform("right_arm_7_link","right_hand_index_knuckle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_index_knuckle_link",now1, wrist_2_right_hand_index_knuckle_link_stamped);
    //right_hand_index_middle_link 
    listener_.waitForTransform("right_arm_7_link","right_hand_index_middle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_index_middle_link",now1, wrist_2_right_hand_index_middle_link_stamped);  
    //right_hand_index_proximal_link
    listener_.waitForTransform("right_arm_7_link","right_hand_index_proximal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_index_proximal_link",now1, wrist_2_right_hand_index_proximal_link_stamped); 

    //right_hand_little_distal_link   
    listener_.waitForTransform("right_arm_7_link","right_hand_little_distal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_little_distal_link",now1, wrist_2_right_hand_little_distal_link_stamped); 
    //right_hand_little_knuckle_link
    listener_.waitForTransform("right_arm_7_link","right_hand_little_knuckle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_little_knuckle_link",now1, wrist_2_right_hand_little_knuckle_link_stamped); 
    //right_hand_little_middle_link
    listener_.waitForTransform("right_arm_7_link","right_hand_little_middle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_little_middle_link",now1, wrist_2_right_hand_little_middle_link_stamped); 
    //right_hand_little_proximal_link  
    listener_.waitForTransform("right_arm_7_link","right_hand_little_proximal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_little_proximal_link",now1, wrist_2_right_hand_little_proximal_link_stamped);     
    
    //right_hand_middle_distal_link  
    listener_.waitForTransform("right_arm_7_link","right_hand_middle_distal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_middle_distal_link",now1, wrist_2_right_hand_middle_distal_link_stamped); 
    //right_hand_middle_knuckle_link
    listener_.waitForTransform("right_arm_7_link","right_hand_middle_knuckle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_middle_knuckle_link",now1, wrist_2_right_hand_middle_knuckle_link_stamped);
    //right_hand_middle_middle_link 
    listener_.waitForTransform("right_arm_7_link","right_hand_middle_middle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_middle_middle_link",now1, wrist_2_right_hand_middle_middle_link_stamped); 
    //right_hand_middle_proximal_link 
    listener_.waitForTransform("right_arm_7_link","right_hand_middle_proximal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_middle_proximal_link",now1, wrist_2_right_hand_middle_proximal_link_stamped);

    //right_hand_ring_distal_link     
    listener_.waitForTransform("right_arm_7_link","right_hand_ring_distal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_ring_distal_link",now1, wrist_2_right_hand_ring_distal_link_stamped);
    //right_hand_ring_knuckle_link    
    listener_.waitForTransform("right_arm_7_link","right_hand_ring_knuckle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_ring_knuckle_link",now1, wrist_2_right_hand_ring_knuckle_link_stamped);  
    //right_hand_ring_middle_link     
    listener_.waitForTransform("right_arm_7_link","right_hand_ring_middle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_ring_middle_link",now1, wrist_2_right_hand_ring_middle_link_stamped);    
    //right_hand_ring_proximal_link    
    listener_.waitForTransform("right_arm_7_link","right_hand_ring_proximal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_ring_proximal_link",now1, wrist_2_right_hand_ring_proximal_link_stamped);    
     
    //right_hand_thumb_distal_link  
    listener_.waitForTransform("right_arm_7_link","right_hand_thumb_distal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_thumb_distal_link",now1, wrist_2_right_hand_thumb_distal_link_stamped);    
    //right_hand_thumb_knuckle_link  
    listener_.waitForTransform("right_arm_7_link","right_hand_thumb_knuckle_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_thumb_knuckle_link",now1, wrist_2_right_hand_thumb_knuckle_link_stamped);    
    //right_hand_thumb_proximal_link
    listener_.waitForTransform("right_arm_7_link","right_hand_thumb_proximal_link",now1, ros::Duration(4));
    listener_.lookupTransform("right_arm_7_link","right_hand_thumb_proximal_link",now1, wrist_2_right_hand_thumb_proximal_link_stamped);    
    
     
    //// Acquiring the wrench by simulated sensor on right arm (Wrench Stamped)
    // This will chance in real scenario to the real sensor measuremnet
    std::string topic_3 = nh_.resolveName("/right_arm_7_link_ft_sensor_topic");
    ROS_INFO(" Waiting for WrenchStamped on right arm 7 link  %s", topic_3.c_str()); 
    right_arm_wrench_stamped_ConstPtr=ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_3, nh_, ros::Duration(3.0));
    if(!right_arm_wrench_stamped_ConstPtr) {
      ROS_ERROR("Empty WrenchStamped!");
      response.result=response.ERROR_SERVER;
    }
    else {ROS_INFO("WrenchStamped acquired");}

    

    

    //// COMPLETING THE RESPONSE MESSAGE
    // Joints Current states
    data_request.response.data.right_hand_joint_curr_state.header=CurrentState_Hand_ConstPtr->header;
    data_request.response.data.right_hand_joint_curr_state.name=CurrentState_Hand_ConstPtr->name;
    data_request.response.data.right_hand_joint_curr_state.position=CurrentState_Hand_ConstPtr->position;
    data_request.response.data.right_hand_joint_curr_state.position=CurrentState_Hand_ConstPtr->position;
    data_request.response.data.right_hand_joint_curr_state.velocity=CurrentState_Hand_ConstPtr->velocity;
    data_request.response.data.right_hand_joint_curr_state.effort=CurrentState_Hand_ConstPtr->effort;
    
    // Right hand links
    // right_hand_index_distal_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_index_distal_link_stamped,right_hand_index_distal_link_msgs); 	
    data_request.response.data.right_hand_index_distal_link=right_hand_index_distal_link_msgs;
    // right_hand_index_knuckle_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_index_knuckle_link_stamped,right_hand_index_knuckle_link_msgs);  
    data_request.response.data.right_hand_index_knuckle_link=right_hand_index_knuckle_link_msgs;
    //wrist_2_right_hand_index_middle_link_msg     
    tf::transformStampedTFToMsg(wrist_2_right_hand_index_middle_link_stamped,right_hand_index_middle_link_msgs);  
    data_request.response.data.right_hand_index_middle_link=right_hand_index_middle_link_msgs;
    //wrist_2_right_hand_index_proximal_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_index_proximal_link_stamped,right_hand_index_proximal_link_msgs);  
    data_request.response.data.right_hand_index_proximal_link=right_hand_index_proximal_link_msgs;
    
    //right_hand_little_distal_link_msg   
    tf::transformStampedTFToMsg(wrist_2_right_hand_little_distal_link_stamped,right_hand_little_distal_link_msgs);  
    data_request.response.data.right_hand_little_distal_link=right_hand_little_distal_link_msgs;
    //right_hand_little_knuckle_link_msg  
    tf::transformStampedTFToMsg(wrist_2_right_hand_little_knuckle_link_stamped,right_hand_little_knuckle_link_msgs);  
    data_request.response.data.right_hand_little_knuckle_link=right_hand_little_knuckle_link_msgs;
    //right_hand_little_middle_link_msg  
    tf::transformStampedTFToMsg(wrist_2_right_hand_little_middle_link_stamped,right_hand_little_middle_link_msgs);  
    data_request.response.data.right_hand_little_middle_link=right_hand_little_middle_link_msgs;
    //right_hand_little_proximal_link_msg  
    tf::transformStampedTFToMsg(wrist_2_right_hand_little_proximal_link_stamped,right_hand_little_proximal_link_msgs);  
    data_request.response.data.right_hand_little_proximal_link=right_hand_little_proximal_link_msgs;

    //right_hand_middle_distal_link_msg  
    tf::transformStampedTFToMsg(wrist_2_right_hand_middle_distal_link_stamped,right_hand_middle_distal_link_msgs);  
    data_request.response.data.right_hand_middle_distal_link=right_hand_middle_distal_link_msgs;
    //right_hand_middle_knuckle_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_middle_knuckle_link_stamped,right_hand_middle_knuckle_link_msgs);  
    data_request.response.data.right_hand_middle_knuckle_link=right_hand_middle_knuckle_link_msgs;
    //right_hand_middle_middle_link_msg 
    tf::transformStampedTFToMsg(wrist_2_right_hand_middle_middle_link_stamped,right_hand_middle_middle_link_msgs);  
    data_request.response.data.right_hand_middle_middle_link=right_hand_middle_middle_link_msgs;
    //right_hand_middle_proximal_link_msg 
    tf::transformStampedTFToMsg(wrist_2_right_hand_middle_proximal_link_stamped,right_hand_middle_proximal_link_msgs);  
    data_request.response.data.right_hand_middle_proximal_link=right_hand_middle_proximal_link_msgs;

    //right_hand_ring_distal_link_msg     
    tf::transformStampedTFToMsg(wrist_2_right_hand_ring_distal_link_stamped,right_hand_ring_distal_link_msgs);  
    data_request.response.data.right_hand_ring_distal_link=right_hand_ring_distal_link_msgs;
    //right_hand_ring_knuckle_link_msg    
    tf::transformStampedTFToMsg(wrist_2_right_hand_ring_knuckle_link_stamped,right_hand_ring_knuckle_link_msgs);  
    data_request.response.data.right_hand_ring_knuckle_link=right_hand_ring_knuckle_link_msgs;
    //wrist_2_right_hand_ring_middle_link_msg     
    tf::transformStampedTFToMsg(wrist_2_right_hand_ring_middle_link_stamped,right_hand_ring_middle_link_msgs);  
    data_request.response.data.right_hand_ring_middle_link=right_hand_ring_middle_link_msgs;
    //right_hand_ring_proximal_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_ring_proximal_link_stamped,right_hand_ring_proximal_link_msgs);  
    data_request.response.data.right_hand_ring_proximal_link=right_hand_ring_proximal_link_msgs;
    
    //right_hand_thumb_distal_link_msg   
    tf::transformStampedTFToMsg(wrist_2_right_hand_thumb_distal_link_stamped,right_hand_thumb_distal_link_msgs);  
    data_request.response.data.right_hand_thumb_distal_link=right_hand_thumb_distal_link_msgs;
    //right_hand_thumb_knuckle_link_msg  
    tf::transformStampedTFToMsg(wrist_2_right_hand_thumb_knuckle_link_stamped,right_hand_thumb_knuckle_link_msgs);  
    data_request.response.data.right_hand_thumb_knuckle_link=right_hand_thumb_knuckle_link_msgs;
    //right_hand_thumb_proximal_link_msg
    tf::transformStampedTFToMsg(wrist_2_right_hand_thumb_proximal_link_stamped,right_hand_thumb_proximal_link_msgs);  
    data_request.response.data.right_hand_thumb_proximal_link=right_hand_thumb_proximal_link_msgs;
    
    //Position motor right hand response:
    data_request.response.data.right_hand_synergy_joint_state_position=right_hand_synergy_joint_state_position;
    data_request.response.data.right_hand_synergy_joint_state_velocity=right_hand_synergy_joint_state_velocity;
    data_request.response.data.right_hand_synergy_joint_state_effort=right_hand_synergy_joint_state_effort;
    data_request.response.data.right_hand_error_position=current_error;
    
    // Wrench on the right arm 7 link
    data_request.response.data.right_arm_wrench_stamped.header=right_arm_wrench_stamped_ConstPtr->header;
    data_request.response.data.right_arm_wrench_stamped.wrench=right_arm_wrench_stamped_ConstPtr->wrench;
    
    
    
    

    

}
//

// ros::Time end_time = ros::Time::now();
// uint64_t end_acq;           
// end_time.fromNSec(end_acq);
// ROS_INFO("End time nSec %f",(float)end_acq);
// uint64_t diff_time_nsec;
// //diff_time_nsec=end_acq-start_acq;
// ROS_INFO("Diff time %f",(float)diff_time_nsec);
response.data=data_request.response.data;
ROS_INFO("Sending Response");
response.result=response.SUCCESS;
}	





} //namespace grasp_estimator

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "data_acquiring_node");
  ROS_INFO("data acquiring node is Here!!!");
  ros::NodeHandle nh;
 
  grasp_estimator::Data_Logger node(nh);

  ros::spin();
  //ros::AsyncSpinner spinner(4); // Use 4 threads
  //spinner.start();
  //ros::waitForShutdown();
  return 0;
}
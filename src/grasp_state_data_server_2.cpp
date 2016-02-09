

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

//Message filters

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace grasp_estimator
{

class Data_Logger_server
{
private:
	// suggested members, note the member_name_ naming convention

	// the node handle
	ros::NodeHandle nh_;

	// node handle in the private namespace
	ros::NodeHandle priv_nh_;
    //! A tf transform listener
   tf::TransformListener listener_;

	// publishers
	ros::Publisher pub_data_states_;


	grasp_estimator::DataAcquired msg_data_acquired_;




public:
	// callback functions
      void data_acquiring_server();
      void publishDataState();

	// constructor
	Data_Logger_server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
        
        
		// advertise topics
		pub_data_states_ = nh_.advertise<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"), 5);
	
		
	}

	//! Empty stub
	~Data_Logger_server() {}

};
//const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
void Data_Logger_server::data_acquiring_server()
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_

  // message initialization 
  grasp_estimator::DataAcquired msg_data_acquired_now;
  // synergy initialization
  control_msgs::JointTrajectoryControllerState::ConstPtr Hand_Synergy_state_ConstPtr;
  // Tf initilization
  tf::StampedTransform wrist_2_left_hand_index_distal_link_stamped,    wrist_2_left_hand_index_knuckle_link_stamped,   wrist_2_left_hand_index_middle_link_stamped,     wrist_2_left_hand_index_proximal_link_stamped;
  tf::StampedTransform wrist_2_left_hand_little_distal_link_stamped,   wrist_2_left_hand_little_knuckle_link_stamped,  wrist_2_left_hand_little_middle_link_stamped,    wrist_2_left_hand_little_proximal_link_stamped;  
  tf::StampedTransform wrist_2_left_hand_middle_distal_link_stamped,   wrist_2_left_hand_middle_knuckle_link_stamped,  wrist_2_left_hand_middle_middle_link_stamped,    wrist_2_left_hand_middle_proximal_link_stamped; 
  tf::StampedTransform wrist_2_left_hand_ring_distal_link_stamped,     wrist_2_left_hand_ring_knuckle_link_stamped,    wrist_2_left_hand_ring_middle_link_stamped,      wrist_2_left_hand_ring_proximal_link_stamped;
  tf::StampedTransform wrist_2_left_hand_thumb_distal_link_stamped,    wrist_2_left_hand_thumb_knuckle_link_stamped,   wrist_2_left_hand_thumb_proximal_link_stamped;
  // Tf msgs initialization
  geometry_msgs::TransformStamped left_hand_index_distal_link_msgs,  left_hand_index_knuckle_link_msgs,  left_hand_index_middle_link_msgs,  left_hand_index_proximal_link_msgs;
  geometry_msgs::TransformStamped left_hand_little_distal_link_msgs, left_hand_little_knuckle_link_msgs, left_hand_little_middle_link_msgs, left_hand_little_proximal_link_msgs;
  geometry_msgs::TransformStamped left_hand_middle_distal_link_msgs, left_hand_middle_knuckle_link_msgs, left_hand_middle_middle_link_msgs, left_hand_middle_proximal_link_msgs;
  geometry_msgs::TransformStamped left_hand_ring_distal_link_msgs,   left_hand_ring_knuckle_link_msgs,   left_hand_ring_middle_link_msgs,   left_hand_ring_proximal_link_msgs;
  geometry_msgs::TransformStamped left_hand_thumb_distal_link_msgs,  left_hand_thumb_knuckle_link_msgs,  left_hand_thumb_proximal_link_msgs;
  

  // wrench initialization
  geometry_msgs::WrenchStamped::ConstPtr left_arm_wrench_stamped_ConstPtr;
  tf::StampedTransform vito_anchor_2_left_gamma_measure;
  geometry_msgs::TransformStamped vito_anchor_2_left_gamma_measure_msgs;

  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Hand  Joint States    ////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  ros::Time start_time = ros::Time(0);
  std::string topic_1 = nh_.resolveName("/left_hand/joint_states");
  //ROS_INFO(" Waiting for left hand joint state on topic %s", topic_1.c_str());
  sensor_msgs::JointState::ConstPtr CurrentState_Hand_ConstPtr; 
  //CurrentState_Hand_ConstPtr =ros::topic::waitForMessage<sensor_msgs::JointState>(topic_1, nh_, ros::Duration(3.0));
  while(!CurrentState_Hand_ConstPtr) {
    //ROS_INFO("Try an acquiring Of Hand Joint State");
    CurrentState_Hand_ConstPtr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic_1, nh_, ros::Duration(3.0));
  }
  //else {ROS_INFO("Joint State acquired");}

  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Sinergy Joint Infomation   ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  
  msg_data_acquired_now.hand_synergy_joint_state_position=CurrentState_Hand_ConstPtr->position[28]; 
  msg_data_acquired_now.hand_synergy_joint_state_velocity=CurrentState_Hand_ConstPtr->velocity[28];
  msg_data_acquired_now.hand_synergy_joint_state_effort=CurrentState_Hand_ConstPtr->effort[28];
  
  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Error  Sinergy Joint  /////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  std::string topic_2 = nh_.resolveName("/left_hand/joint_trajectory_controller/state");
  //ROS_INFO(" Waiting for left Hand_Synergy_state %s", topic_2.c_str());
  Hand_Synergy_state_ConstPtr=ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(topic_2, nh_, ros::Duration(3.0));
  while(!Hand_Synergy_state_ConstPtr) {
    Hand_Synergy_state_ConstPtr=ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(topic_2, nh_, ros::Duration(3.0));
    }
    //ROS_ERROR("Empty Hand_Synergy_state!");
  // Completing the topic
  msg_data_acquired_now.hand_error_position=Hand_Synergy_state_ConstPtr->error.positions[0];
  
 
    
  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// TF information            ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  
  //// acquiring tf Pose of left hand
  //ROS_INFO("Starting acquiring tf pose left hand "); 
  ros::Time now1 = ros::Time(0);
  // left_hand_index_distal_link
  listener_.waitForTransform("left_arm_7_link","left_hand_index_distal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_index_distal_link",now1, wrist_2_left_hand_index_distal_link_stamped);
  //left_hand_index_knuckle_link
  listener_.waitForTransform("left_arm_7_link","left_hand_index_knuckle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_index_knuckle_link",now1, wrist_2_left_hand_index_knuckle_link_stamped);
  //left_hand_index_middle_link 
  listener_.waitForTransform("left_arm_7_link","left_hand_index_middle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_index_middle_link",now1, wrist_2_left_hand_index_middle_link_stamped);  
  //left_hand_index_proximal_link
  listener_.waitForTransform("left_arm_7_link","left_hand_index_proximal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_index_proximal_link",now1, wrist_2_left_hand_index_proximal_link_stamped); 

  //left_hand_little_distal_link   
  listener_.waitForTransform("left_arm_7_link","left_hand_little_distal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_little_distal_link",now1, wrist_2_left_hand_little_distal_link_stamped); 
  //left_hand_little_knuckle_link
  listener_.waitForTransform("left_arm_7_link","left_hand_little_knuckle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_little_knuckle_link",now1, wrist_2_left_hand_little_knuckle_link_stamped); 
  //left_hand_little_middle_link
  listener_.waitForTransform("left_arm_7_link","left_hand_little_middle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_little_middle_link",now1, wrist_2_left_hand_little_middle_link_stamped); 
  //left_hand_little_proximal_link  
  listener_.waitForTransform("left_arm_7_link","left_hand_little_proximal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_little_proximal_link",now1, wrist_2_left_hand_little_proximal_link_stamped);     
  
  //left_hand_middle_distal_link  
  listener_.waitForTransform("left_arm_7_link","left_hand_middle_distal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_middle_distal_link",now1, wrist_2_left_hand_middle_distal_link_stamped); 
  //left_hand_middle_knuckle_link
  listener_.waitForTransform("left_arm_7_link","left_hand_middle_knuckle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_middle_knuckle_link",now1, wrist_2_left_hand_middle_knuckle_link_stamped);
  //left_hand_middle_middle_link 
  listener_.waitForTransform("left_arm_7_link","left_hand_middle_middle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_middle_middle_link",now1, wrist_2_left_hand_middle_middle_link_stamped); 
  //left_hand_middle_proximal_link 
  listener_.waitForTransform("left_arm_7_link","left_hand_middle_proximal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_middle_proximal_link",now1, wrist_2_left_hand_middle_proximal_link_stamped);

  //left_hand_ring_distal_link     
  listener_.waitForTransform("left_arm_7_link","left_hand_ring_distal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_ring_distal_link",now1, wrist_2_left_hand_ring_distal_link_stamped);
  //left_hand_ring_knuckle_link    
  listener_.waitForTransform("left_arm_7_link","left_hand_ring_knuckle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_ring_knuckle_link",now1, wrist_2_left_hand_ring_knuckle_link_stamped);  
  //left_hand_ring_middle_link     
  listener_.waitForTransform("left_arm_7_link","left_hand_ring_middle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_ring_middle_link",now1, wrist_2_left_hand_ring_middle_link_stamped);    
  //left_hand_ring_proximal_link    
  listener_.waitForTransform("left_arm_7_link","left_hand_ring_proximal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_ring_proximal_link",now1, wrist_2_left_hand_ring_proximal_link_stamped);    
   
  //left_hand_thumb_distal_link  
  listener_.waitForTransform("left_arm_7_link","left_hand_thumb_distal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_thumb_distal_link",now1, wrist_2_left_hand_thumb_distal_link_stamped);    
  //left_hand_thumb_knuckle_link  
  listener_.waitForTransform("left_arm_7_link","left_hand_thumb_knuckle_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_thumb_knuckle_link",now1, wrist_2_left_hand_thumb_knuckle_link_stamped);    
  //left_hand_thumb_proximal_link
  listener_.waitForTransform("left_arm_7_link","left_hand_thumb_proximal_link",now1, ros::Duration(1));
  listener_.lookupTransform("left_arm_7_link","left_hand_thumb_proximal_link",now1, wrist_2_left_hand_thumb_proximal_link_stamped);    
  
  //////////////////// SENDING TF /////////////////////////////////////////////////////////////////////
  // left_hand_index_distal_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_index_distal_link_stamped,left_hand_index_distal_link_msgs);  
  msg_data_acquired_now.hand_index_distal_link=left_hand_index_distal_link_msgs;
  // left_hand_index_knuckle_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_index_knuckle_link_stamped,left_hand_index_knuckle_link_msgs);  
  msg_data_acquired_now.hand_index_knuckle_link=left_hand_index_knuckle_link_msgs;
  //wrist_2_left_hand_index_middle_link_msg     
  tf::transformStampedTFToMsg(wrist_2_left_hand_index_middle_link_stamped,left_hand_index_middle_link_msgs);  
  msg_data_acquired_now.hand_index_middle_link=left_hand_index_middle_link_msgs;
  //wrist_2_left_hand_index_proximal_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_index_proximal_link_stamped,left_hand_index_proximal_link_msgs);  
  msg_data_acquired_now.hand_index_proximal_link=left_hand_index_proximal_link_msgs;
  
  //left_hand_little_distal_link_msg   
  tf::transformStampedTFToMsg(wrist_2_left_hand_little_distal_link_stamped,left_hand_little_distal_link_msgs);  
  msg_data_acquired_now.hand_little_distal_link=left_hand_little_distal_link_msgs;
  //left_hand_little_knuckle_link_msg  
  tf::transformStampedTFToMsg(wrist_2_left_hand_little_knuckle_link_stamped,left_hand_little_knuckle_link_msgs);  
  msg_data_acquired_now.hand_little_knuckle_link=left_hand_little_knuckle_link_msgs;
  //left_hand_little_middle_link_msg  
  tf::transformStampedTFToMsg(wrist_2_left_hand_little_middle_link_stamped,left_hand_little_middle_link_msgs);  
  msg_data_acquired_now.hand_little_middle_link=left_hand_little_middle_link_msgs;
  //left_hand_little_proximal_link_msg  
  tf::transformStampedTFToMsg(wrist_2_left_hand_little_proximal_link_stamped,left_hand_little_proximal_link_msgs);  
  msg_data_acquired_now.hand_little_proximal_link=left_hand_little_proximal_link_msgs;

  //left_hand_middle_distal_link_msg  
  tf::transformStampedTFToMsg(wrist_2_left_hand_middle_distal_link_stamped,left_hand_middle_distal_link_msgs);  
  msg_data_acquired_now.hand_middle_distal_link=left_hand_middle_distal_link_msgs;
  //left_hand_middle_knuckle_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_middle_knuckle_link_stamped,left_hand_middle_knuckle_link_msgs);  
  msg_data_acquired_now.hand_middle_knuckle_link=left_hand_middle_knuckle_link_msgs;
  //left_hand_middle_middle_link_msg 
  tf::transformStampedTFToMsg(wrist_2_left_hand_middle_middle_link_stamped,left_hand_middle_middle_link_msgs);  
  msg_data_acquired_now.hand_middle_middle_link=left_hand_middle_middle_link_msgs;
  //left_hand_middle_proximal_link_msg 
  tf::transformStampedTFToMsg(wrist_2_left_hand_middle_proximal_link_stamped,left_hand_middle_proximal_link_msgs);  
  msg_data_acquired_now.hand_middle_proximal_link=left_hand_middle_proximal_link_msgs;

  //left_hand_ring_distal_link_msg     
  tf::transformStampedTFToMsg(wrist_2_left_hand_ring_distal_link_stamped,left_hand_ring_distal_link_msgs);  
  msg_data_acquired_now.hand_ring_distal_link=left_hand_ring_distal_link_msgs;
  //left_hand_ring_knuckle_link_msg    
  tf::transformStampedTFToMsg(wrist_2_left_hand_ring_knuckle_link_stamped,left_hand_ring_knuckle_link_msgs);  
  msg_data_acquired_now.hand_ring_knuckle_link=left_hand_ring_knuckle_link_msgs;
  //wrist_2_left_hand_ring_middle_link_msg     
  tf::transformStampedTFToMsg(wrist_2_left_hand_ring_middle_link_stamped,left_hand_ring_middle_link_msgs);  
  msg_data_acquired_now.hand_ring_middle_link=left_hand_ring_middle_link_msgs;
  //left_hand_ring_proximal_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_ring_proximal_link_stamped,left_hand_ring_proximal_link_msgs);  
  msg_data_acquired_now.hand_ring_proximal_link=left_hand_ring_proximal_link_msgs;
  
  //left_hand_thumb_distal_link_msg   
  tf::transformStampedTFToMsg(wrist_2_left_hand_thumb_distal_link_stamped,left_hand_thumb_distal_link_msgs);  
  msg_data_acquired_now.hand_thumb_distal_link=left_hand_thumb_distal_link_msgs;
  //left_hand_thumb_knuckle_link_msg  
  tf::transformStampedTFToMsg(wrist_2_left_hand_thumb_knuckle_link_stamped,left_hand_thumb_knuckle_link_msgs);  
  msg_data_acquired_now.hand_thumb_knuckle_link=left_hand_thumb_knuckle_link_msgs;
  //left_hand_thumb_proximal_link_msg
  tf::transformStampedTFToMsg(wrist_2_left_hand_thumb_proximal_link_stamped,left_hand_thumb_proximal_link_msgs);  
  msg_data_acquired_now.hand_thumb_proximal_link=left_hand_thumb_proximal_link_msgs; 
  


   /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Wrench information         ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  //std::string topic_3 = nh_.resolveName("/left/ft_sensor_topic");
  std::string topic_3 = nh_.resolveName("/left_ft_sensor/left/force_torque_sensor_filtered");

  //ROS_INFO(" Waiting for WrenchStamped on left arm 7 link  %s", topic_3.c_str()); 
  left_arm_wrench_stamped_ConstPtr=ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_3, nh_, ros::Duration(3.0));
  if(!left_arm_wrench_stamped_ConstPtr) {
    ROS_INFO("Empty WrenchStamped!");
    while(!left_arm_wrench_stamped_ConstPtr){
     left_arm_wrench_stamped_ConstPtr=ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_3, nh_, ros::Duration(3.0));
    }
  }

  listener_.waitForTransform("vito_anchor","left_measure",ros::Time(0), ros::Duration(1));
  listener_.lookupTransform("vito_anchor","left_measure",ros::Time(0), vito_anchor_2_left_gamma_measure);
  tf::transformStampedTFToMsg(vito_anchor_2_left_gamma_measure,vito_anchor_2_left_gamma_measure_msgs);  
  msg_data_acquired_now.vito_anchor_2_arm_7_link=vito_anchor_2_left_gamma_measure_msgs; 

  // Wrench on the left arm 7 link
  msg_data_acquired_now.arm_wrench_stamped.header=left_arm_wrench_stamped_ConstPtr->header;
  msg_data_acquired_now.arm_wrench_stamped.wrench=left_arm_wrench_stamped_ConstPtr->wrench;
  

  this->msg_data_acquired_=msg_data_acquired_now;
  //ROS_INFO("Data_Acquiring_srv Done !!!");    
  return;
}


void Data_Logger_server::publishDataState()
{
	data_acquiring_server();
	pub_data_states_.publish(msg_data_acquired_);
	return;
}




}//namespace grasp_estimator





int main(int argc, char **argv)
{
	ros::init(argc, argv, "Data_Logger_server");
    grasp_estimator::DataAcquired::ConstPtr msg_data_acquired;//
	ros::NodeHandle nh;
    //tf::TransformStamped msgs_tf;
	grasp_estimator::Data_Logger_server data_logger_publisher(nh);
    
    ROS_INFO("Data_Logger_server is Here !!!");

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		//Tf_acquiring(msgs_tf); msg_data_acquired_.
        //grasp_estimator::Data_Logger_server::data_acquiring_server(msg_data_acquired);
        //Data_Logger_server::data_acquiring_server;
		data_logger_publisher.publishDataState();

		ros::spinOnce();
		//ROS_INFO("Data Published !");
		loop_rate.sleep();
	}

	return 0;
}
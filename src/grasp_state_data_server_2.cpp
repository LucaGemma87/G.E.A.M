

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

#define Hz 25

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
  
  // Subscriber
  ros::Subscriber sub_hand_state_;
  ros::Subscriber sub_sinergy_hand_state_;
  ros::Subscriber sub_error_sinergy_hand_state_;
  ros::Subscriber sub_ft_sensor_state_;



	grasp_estimator::DataAcquired msg_data_acquired_;

  sensor_msgs::JointState CurrentState_Hand_now,CurrentSinergyState_Hand_now;
  control_msgs::JointTrajectoryControllerState CurrentErrorSinergyState_Hand_now; 

  geometry_msgs::WrenchStamped ft_sensor_measure_now;


public:
	// callback functions
      void data_acquiring_server();
      void sinergy_state(const sensor_msgs::JointState::ConstPtr &CurrentSinergyState_Hand);
      void error_sinergy_state(const control_msgs::JointTrajectoryControllerState::ConstPtr &CurrentErrorSinergyState_Hand);
      void glove_state(const sensor_msgs::JointState::ConstPtr &CurrentState_Hand);
      void ft_sensor_state(const geometry_msgs::WrenchStamped::ConstPtr &ft_sensor_measure);
      void publishDataState();


	// constructor
	Data_Logger_server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
    CurrentSinergyState_Hand_now.name.resize(1);
    CurrentSinergyState_Hand_now.position.resize(1);
    CurrentSinergyState_Hand_now.velocity.resize(1);
    CurrentSinergyState_Hand_now.effort.resize(1);

    CurrentState_Hand_now.name.resize(33);
    CurrentState_Hand_now.position.resize(33);
    CurrentState_Hand_now.velocity.resize(33);
    CurrentState_Hand_now.effort.resize(33);

  
   CurrentErrorSinergyState_Hand_now.desired.positions.resize(1);
   CurrentErrorSinergyState_Hand_now.desired.velocities.resize(1);
   CurrentErrorSinergyState_Hand_now.desired.accelerations.resize(1);
   CurrentErrorSinergyState_Hand_now.desired.effort.resize(1);
 
  CurrentErrorSinergyState_Hand_now.actual.positions.resize(1);
  CurrentErrorSinergyState_Hand_now.actual.velocities.resize(1);
  CurrentErrorSinergyState_Hand_now.actual.accelerations.resize(1);
  CurrentErrorSinergyState_Hand_now.actual.effort.resize(1);
  
  CurrentErrorSinergyState_Hand_now.error.positions.resize(1);
  CurrentErrorSinergyState_Hand_now.error.velocities.resize(1);
  CurrentErrorSinergyState_Hand_now.error.accelerations.resize(1);
  CurrentErrorSinergyState_Hand_now.error.effort.resize(1);

        
		// advertise topics
		pub_data_states_ = nh_.advertise<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"), 5);
	
		    sub_hand_state_ = nh_.subscribe<sensor_msgs::JointState>(nh_.resolveName("/left_hand/imu_glove/joint_states"),10,
                                                                                       &Data_Logger_server::glove_state,
                                                                                       this);
       sub_sinergy_hand_state_ = nh_.subscribe<sensor_msgs::JointState>(nh_.resolveName("/left_hand/joint_states"),10,
                                                                                      &Data_Logger_server::sinergy_state,
                                                                                      this);
       sub_error_sinergy_hand_state_ = nh_.subscribe<control_msgs::JointTrajectoryControllerState>(nh_.resolveName("/left_hand/joint_trajectory_controller/state"),10,
                                                                                      &Data_Logger_server::error_sinergy_state,
                                                                                      this);
       sub_ft_sensor_state_ = nh_.subscribe<geometry_msgs::WrenchStamped>(nh_.resolveName("/left_ft_sensor/left/force_torque_sensor_filtered"),10,
                                                                                      &Data_Logger_server::ft_sensor_state,
                                                                                      this);

	}

	//! Empty stub
	~Data_Logger_server() {}

};


void Data_Logger_server::sinergy_state(const sensor_msgs::JointState::ConstPtr &CurrentSinergyState_Hand)
{
 this->CurrentSinergyState_Hand_now = *CurrentSinergyState_Hand;
 //ROS_INFO("Sinergy acquired");
}

void Data_Logger_server::error_sinergy_state(const control_msgs::JointTrajectoryControllerState::ConstPtr &CurrentErrorSinergyState_Hand)
{
 this->CurrentErrorSinergyState_Hand_now = *CurrentErrorSinergyState_Hand;
 //ROS_INFO("Error Sinergy acquired"); 
}

void Data_Logger_server::glove_state(const sensor_msgs::JointState::ConstPtr &CurrentState_Hand)
{
 this->CurrentState_Hand_now = *CurrentState_Hand;
 ROS_INFO("Glove acquired");
}

void Data_Logger_server::ft_sensor_state(const geometry_msgs::WrenchStamped::ConstPtr &ft_sensor_measure)
{
  ft_sensor_measure_now = *ft_sensor_measure;
  //ROS_INFO("FT sensor acquired");
}

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
  
  tf::StampedTransform vito_anchor_2_left_arm_7_link;
  geometry_msgs::TransformStamped vito_anchor_2_left_arm_7_link_msgs;

  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Hand  Joint States    ////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 

  
  //msg_data_acquired_now.hand_joint_curr_state=CurrentState_Hand_now;
  

  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Sinergy Joint Infomation   ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  //ROS_INFO("Sinergy Joint = %g", CurrentState_Hand_ConstPtr->position[28]); 
  //msg_data_acquired_now.hand_synergy_joint_state_position=CurrentSinergyState_Hand_now.position[0]; 
  //msg_data_acquired_now.hand_synergy_joint_state_velocity=CurrentSinergyState_Hand_now.velocity[0];
  //msg_data_acquired_now.hand_synergy_joint_state_effort=CurrentSinergyState_Hand_now.effort[0];
  

  //ROS_INFO("CurrentState_Hand_ConstPtr->position[28];");
  /////////////////////////////////////////////////////////////////////////////////
  ///////////////// Error  Sinergy Joint  /////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////// 
  
  
  //msg_data_acquired_now.hand_error_position=CurrentErrorSinergyState_Hand_now.error.positions[0];
  
 
    
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
  // //std::string topic_3 = nh_.resolveName("/left/ft_sensor_topic");
  // std::string topic_3 = nh_.resolveName("/left_ft_sensor/left/force_torque_sensor_filtered");

  // // //ROS_INFO(" Waiting for WrenchStamped on left arm 7 link  %s", topic_3.c_str()); 
  // left_arm_wrench_stamped_ConstPtr=ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_3, nh_, ros::Duration(3.0));
  // if(!left_arm_wrench_stamped_ConstPtr) {
  //   ROS_INFO("Empty WrenchStamped!");
  //   while(!left_arm_wrench_stamped_ConstPtr){
  //    left_arm_wrench_stamped_ConstPtr=ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_3, nh_, ros::Duration(3.0));
  //   }
  // }
  //   // Wrench on the ft sensor
  msg_data_acquired_now.arm_wrench_stamped=ft_sensor_measure_now;
  

  listener_.waitForTransform("left_measure","vito_anchor",ros::Time(0), ros::Duration(1));
  listener_.lookupTransform("left_measure","vito_anchor",ros::Time(0), vito_anchor_2_left_gamma_measure);
  tf::transformStampedTFToMsg(vito_anchor_2_left_gamma_measure,vito_anchor_2_left_gamma_measure_msgs);  
  msg_data_acquired_now.vito_anchor_2_left_measure=vito_anchor_2_left_gamma_measure_msgs; 

  listener_.waitForTransform("vito_anchor","left_arm_7_link",ros::Time(0), ros::Duration(1));
  listener_.lookupTransform("vito_anchor","left_arm_7_link",ros::Time(0), vito_anchor_2_left_arm_7_link);
  tf::transformStampedTFToMsg(vito_anchor_2_left_arm_7_link,vito_anchor_2_left_arm_7_link_msgs);  
  msg_data_acquired_now.vito_anchor_2_arm_7_link=vito_anchor_2_left_arm_7_link_msgs;


  msg_data_acquired_now.time_now=ros::Time::now();

  this->msg_data_acquired_=msg_data_acquired_now;
  ROS_INFO("Data_Acquiring_srv Done !!!");    
  return;
}


void Data_Logger_server::publishDataState()
{
	
	pub_data_states_.publish(msg_data_acquired_);
	ROS_INFO("Data Published !!!");
  //return;
}




}//namespace grasp_estimator





int main(int argc, char **argv)
{
	ros::init(argc, argv, "Data_Logger_server");
    grasp_estimator::DataAcquired::ConstPtr msg_data_acquired;//
	ros::NodeHandle nh;
    //tf::TransformStamped msgs_tf;
     ros::AsyncSpinner spinner(4); 
   spinner.start();
	grasp_estimator::Data_Logger_server data_logger_publisher(nh);
    
    ROS_INFO("Data_Logger_server is Here !!!");

	ros::Rate loop_rate(Hz);

	while (ros::ok())
	{
		//Tf_acquiring(msgs_tf); msg_data_acquired_.
        //grasp_estimator::Data_Logger_server::data_acquiring_server(msg_data_acquired);
        //Data_Logger_server::data_acquiring_server;
    data_logger_publisher.data_acquiring_server();
		data_logger_publisher.publishDataState();

		//ros::spinOnce();
    
		//ROS_INFO("Data Published !");
		//

    loop_rate.sleep();
	}
  spinner.stop();
	return 0;
}
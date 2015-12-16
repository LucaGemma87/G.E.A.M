

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
#include "grasp_estimator/DataAcquired.h"

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

	// subscribers
    ros::Subscriber actual_joint_hand_state_;

    ros::Subscriber actual_wrench_state_;

   
    ros::Subscriber tf_state_;
    // message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_sim_;
    // message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_imu_;
	// message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_ft_measurement_;
	// message_filters::Subscriber<geometry_msgs::PoseStamped> sub_hand_object_relative_pose_;

	// sync-er
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> SyncPolicy_;
    // message_filters::Synchronizer<SyncPolicy_> sensor_sync_;

	// publishers
	ros::Publisher pub_data_states_;

	// grasp state message
	const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_;
	const geometry_msgs::WrenchStamped::ConstPtr msgs_wrench_;
	const sensor_msgs::JointState::ConstPtr msgs_joint_;
	const geometry_msgs::TransformStamped::ConstPtr tf_msg_;
    tf::StampedTransform msgs_tf;



public:
	// callback functions
  
    void Hand_joint_acquiring(const sensor_msgs::JointState::ConstPtr &msgs_joint_);
    void Wrench_acquiring(const geometry_msgs::WrenchStamped::ConstPtr  &msgs_wrench_);
    void Tf_acquiring(const geometry_msgs::TransformStamped::ConstPtr &tf_msg_);
    void publishDataState();

	// constructor
	Data_Logger_server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
        
        actual_joint_hand_state_ = nh_.subscribe<sensor_msgs::JointState>(nh_.resolveName("/right_hand/joint_states"),
                                                                                     10,
                                                                                     &Data_Logger_server::Hand_joint_acquiring,
                                                                                     this);

        actual_wrench_state_ = nh_.subscribe<geometry_msgs::WrenchStamped>(nh_.resolveName("/right_arm_7_link_ft_sensor_topic"),10,&Data_Logger_server::Wrench_acquiring,this);


        tf_state_ = nh_.subscribe<tf::StampedTransform>(nh_.resolveName("/tf"),10,&Data_Logger_server::Tf_acquiring,this);
		// advertise topics
		pub_data_states_ = nh_.advertise<grasp_estimator::DataAcquired>(nh_.resolveName("/right_hand/data_logger_server_"), 10);
	
		
	}

	//! Empty stub
	~Data_Logger_server() {}

};

// the grouped callback function
void Data_Logger_server::Tf_acquiring(const geometry_msgs::TransformStamped::ConstPtr &tf_msg_);
{ 
  grasp_estimator::DataAcquired data_now;
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
  
  
  return;
}

// put inside message hand joint state
 void Data_Logger_server::Hand_joint_acquiring(const sensor_msgs::JointState::ConstPtr &msgs_joint_)
 {
  //msg_data_acquired_.right_hand_joint_curr_state.header=msgs_joint->header;
  // msg_data_acquired->right_hand_joint_curr_state.name=msgs_joint->name;
  // msg_data_acquired->right_hand_joint_curr_state.position=msgs_joint->position;
  // msg_data_acquired->right_hand_joint_curr_state.position=msgs_joint->position;
  // msg_data_acquired->right_hand_joint_curr_state.velocity=msgs_joint->velocity;
  // msg_data_acquired->right_hand_joint_curr_state.effort=msgs_joint->effort;
  return;	
 }

void Data_Logger_server::Wrench_acquiring(const geometry_msgs::WrenchStamped::ConstPtr  &msgs_wrench_)
{

	return;
}

void Data_Logger_server::publishDataState()
{
	pub_data_states_.publish(msg_data_acquired_);
	return;
}




}//namespace grasp_estimator





int main(int argc, char **argv)
{
	ros::init(argc, argv, "Data_Logger_server");

	ros::NodeHandle nh;
  tf::TransformStamped msgs_tf;
	grasp_estimator::Data_Logger_server data_logger_publisher(nh);
    
    ROS_INFO("Data_Logger_server is Here !!!");

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//Tf_acquiring(msgs_tf);
        
		data_logger_publisher.publishDataState();

		ros::spinOnce();
		ROS_INFO("Data Published !");
		loop_rate.sleep();
	}

	return 0;
}
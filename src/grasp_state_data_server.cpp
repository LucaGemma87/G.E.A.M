

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
	//const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_;
	grasp_estimator::DataAcquired msg_data_acquired_;
	//const geometry_msgs::WrenchStamped::ConstPtr msgs_wrench_;
	//const sensor_msgs::JointState::ConstPtr msgs_joint_;
	//const geometry_msgs::TransformStamped::ConstPtr tf_msg_;
  //  tf::StampedTransform msgs_tf;



public:
	// callback functions
      void data_acquiring_server();
      //const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
      //const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
//    void Hand_joint_acquiring(const sensor_msgs::JointState::ConstPtr &msgs_joint_);
//    void Wrench_acquiring(const geometry_msgs::WrenchStamped::ConstPtr  &msgs_wrench_);
//    void Tf_acquiring(const geometry_msgs::TransformStamped::ConstPtr &tf_msg_);
      void publishDataState();

	// constructor
	Data_Logger_server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
        
        
		// advertise topics
		pub_data_states_ = nh_.advertise<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"), 10);
	
		
	}

	//! Empty stub
	~Data_Logger_server() {}

};
//const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
void Data_Logger_server::data_acquiring_server()
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
  grasp_estimator::DataAcquired msg_data_acquired_now;
  std::string service_name("left_hand_arm_data_logger_srv");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  //if (!nh.ok()) exit(0);  && nh.ok()

  
  grasp_estimator::DataAcquiring Data_Acquiring_srv;
  
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0))  )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  //if (!nh.ok()) exit(0);  && nh.ok()
  
 

  
  Data_Acquiring_srv.request.activation=(int)1;
     

  //ROS_INFO("test_data_acquiring  waiting for service on topic data_logger_srv");
  
  if (!ros::service::call(service_name, Data_Acquiring_srv))
  {
    ROS_ERROR("Call to data_logger_srv service failed");
    exit(0);
  }
  //ROS_INFO("data_logger_srv  receivint results for service on topic data_logger_srv");
  if (Data_Acquiring_srv.response.result != Data_Acquiring_srv.response.SUCCESS)
    {
     ROS_ERROR("Data_Acquiring_srv returned error %d", Data_Acquiring_srv.response.result);
     exit(0);
    }
  else 
  { 
    //ROS_INFO("Receving result");
    //int result;
    
    //result= Data_Acquiring_srv.response.result;
    
    //ROS_INFO("Result: %d", (int)result);
    
    // check joint state
    //ROS_INFO(" Data_Acquiring_srv.data.hand_joint_curr_state.position[0] %f ",(float)Data_Acquiring_srv.response.data.hand_joint_curr_state.position[0]);
    
    // check right hand index distal link
    // tf::StampedTransform wrist_2_right_hand_index_distal_link_stamped;
    // transformStampedMsgToTF(Data_Acquiring_srv.response.data.hand_index_distal_link,wrist_2_right_hand_index_distal_link_stamped);
    // tf::Vector3 position = wrist_2_right_hand_index_distal_link_stamped.getOrigin();
    // tf::Quaternion quaternion = wrist_2_right_hand_index_distal_link_stamped.getRotation();
    // ROS_INFO("right_hand_index_distal_link position: %f , %f , %f",(float)position[0],(float)position[1],(float)position[2]);
    // ROS_INFO("right_hand_index_distal_link quaternion: %f , %f , %f, %f",(float)quaternion[0],(float)quaternion[1],(float)quaternion[2],(float)quaternion[3]);
    
    // ROS_INFO("Motor right hand synergy position %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_position);
    // ROS_INFO("Motor right hand synergy velocity %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_velocity);
    // ROS_INFO("Motor right hand synergy effort %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_effort);
    // ROS_INFO("Motor right hand error position %f", (float)Data_Acquiring_srv.response.data.hand_error_position);
    // ROS_INFO("Wrench stamped on right arm 7 link: Force [%f,%f,%f] Torque [%f,%f,%f]", (float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.x,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.y,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.z,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.x,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.y,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.z);

    msg_data_acquired_now=Data_Acquiring_srv.response.data;
    msg_data_acquired_=msg_data_acquired_now;
   
    //msg_data_acquired_->hand_joint_curr_state=Data_Acquiring_srv.response.hand_joint_curr_state;
    ROS_INFO("Data_Acquiring_srv Done !!!");    
  }

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

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//Tf_acquiring(msgs_tf); msg_data_acquired_.
        //grasp_estimator::Data_Logger_server::data_acquiring_server(msg_data_acquired);
        //Data_Logger_server::data_acquiring_server;
		data_logger_publisher.publishDataState();

		ros::spinOnce();
		ROS_INFO("Data Published !");
		loop_rate.sleep();
	}

	return 0;
}
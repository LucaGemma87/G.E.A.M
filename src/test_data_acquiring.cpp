
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



/*! Simply pings the cylinder fitting service and prints out the result.*/
int main(int argc, char **argv)
{  


  ros::init(argc, argv, "test_data_acquiring");
  ros::NodeHandle nh;

  std::string service_name("left_hand_arm_data_logger_srv");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  
  grasp_estimator::DataAcquiring Data_Acquiring_srv;
  
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);
  
 

  
  Data_Acquiring_srv.request.activation=(int)1;
     

  ROS_INFO("test_data_acquiring  waiting for service on topic data_logger_srv");
  
  if (!ros::service::call(service_name, Data_Acquiring_srv))
  {
    ROS_ERROR("Call to data_logger_srv service failed");
    exit(0);
  }
  ROS_INFO("data_logger_srv  receivint results for service on topic data_logger_srv");
  if (Data_Acquiring_srv.response.result != Data_Acquiring_srv.response.SUCCESS)
    {
     ROS_ERROR("Data_Acquiring_srv returned error %d", Data_Acquiring_srv.response.result);
     exit(0);
    }
  else 
  { 
    ROS_INFO("Receving result");
    int result;
    
    result= Data_Acquiring_srv.response.result;
    
    ROS_INFO("Result: %d", (int)result);
    
    // check joint state
    ROS_INFO(" Data_Acquiring_srv.data.hand_joint_curr_state.position[0] %f ",(float)Data_Acquiring_srv.response.data.hand_joint_curr_state.position[0]);
    
    // check right hand index distal link
    tf::StampedTransform wrist_2_right_hand_index_distal_link_stamped;
    transformStampedMsgToTF(Data_Acquiring_srv.response.data.hand_index_distal_link,wrist_2_right_hand_index_distal_link_stamped);
    tf::Vector3 position = wrist_2_right_hand_index_distal_link_stamped.getOrigin();
    tf::Quaternion quaternion = wrist_2_right_hand_index_distal_link_stamped.getRotation();
    ROS_INFO("right_hand_index_distal_link position: %f , %f , %f",(float)position[0],(float)position[1],(float)position[2]);
    ROS_INFO("right_hand_index_distal_link quaternion: %f , %f , %f, %f",(float)quaternion[0],(float)quaternion[1],(float)quaternion[2],(float)quaternion[3]);
    
    ROS_INFO("Motor right hand synergy position %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_position);
    ROS_INFO("Motor right hand synergy velocity %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_velocity);
    ROS_INFO("Motor right hand synergy effort %f",(float)Data_Acquiring_srv.response.data.hand_synergy_joint_state_effort);
    ROS_INFO("Motor right hand error position %f", (float)Data_Acquiring_srv.response.data.hand_error_position);
    ROS_INFO("Wrench stamped on right arm 7 link: Force [%f,%f,%f] Torque [%f,%f,%f]", (float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.x,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.y,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.force.z,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.x,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.y,(float)Data_Acquiring_srv.response.data.arm_wrench_stamped.wrench.torque.z);

    
    
    
    ROS_INFO("Data_Acquiring_srv Done !!!");    
  }
 return true;
};

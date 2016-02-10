

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
#include "grasp_estimator/WeightEstimated.h"

// ROS headers
#include <ros/ros.h>

// ROS Control
#include "control_msgs/JointTrajectoryControllerState.h"

//Message filters

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Gnu Fitting
#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_multiset.h>

// Moveit Header
#include <moveit/move_group_interface/move_group.h>

#define N 25

namespace grasp_estimator
{

class Weight_Estimator_Server
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
	ros::Publisher pub_weight_states_;

	ros::Publisher pub_cog_;
    // subscribers
    ros::Subscriber msg_grasp_state_now_;

	grasp_estimator::DataAcquired msg_data_acquired_;

	grasp_estimator::DataAcquired msg_data_acquired_recent;
    
    grasp_estimator::WeightEstimated weight_estimate_now;
  
    grasp_estimator::DataAcquired msg_data_acquired_array[N];
    
    //


public:
	// callback functions
      void weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now);
      void Hand_Calibration();
      void Object_Estimation();
      void publishDataState();
      static int center_f(const gsl_vector *x, void *data, gsl_vector *f);
    
      int index; 

      bool calibration;

      double weight_measured[N];
      double mass_measured[N];

      double weight_measured_now;
      double mass_measured_now;

      double hand_weight;
      double hand_mass; 

      tf::Vector3 hand_gravity_center;

	// constructor
	Weight_Estimator_Server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
                     
        msg_grasp_state_now_ = nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),5,
                                                                                     &Weight_Estimator_Server::weight_estimation_server,
                                                                                     this);
        
		// advertise topics
		pub_weight_states_ = nh_.advertise<grasp_estimator::WeightEstimated>(nh_.resolveName("/left_hand/mass_estimator_server_"), 5);
	
		pub_cog_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/cog_server"),5);


	}

	//! Empty stub
	~Weight_Estimator_Server() {}

};

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GSL ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

 struct data {
  size_t n;

  double * force_x;
  double * force_y;
  double * force_z;

  double * torque_x;
  double * torque_y;
  double * torque_z;
 };

int Weight_Estimator_Server::center_f(const gsl_vector *x, void *data, gsl_vector *f)
{ 
  size_t n_data = ((struct data *)data)->n;

  double *f_x = ((struct data *)data)->force_x;
  double *f_y = ((struct data *)data)->force_y;
  double *f_z = ((struct data *)data)->force_z;
  double *tau_x = ((struct data *)data)->torque_x;
  double *tau_y = ((struct data *)data)->torque_y;
  double *tau_z = ((struct data *)data)->torque_z;

  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);

  ROS_INFO("Force : %lf, %lf, %lf",f_x[0],f_y[0],f_z[0]);
  ROS_INFO("Torque : %lf, %lf, %lf",tau_x[0],tau_y[0],tau_z[0]);

  size_t i;

  // f1=f(3)*data(2)-f(2)*data(3)-tau(1);
  // f2=f(1)*data(3)-f(3)*data(1)-tau(2);
  // f3=f(2)*data(1)-f(1)*data(2)-tau(3);
  // vals=((f1)^2+(f2)^2+(f3)^2)^2;

  for (i = 0; i < n_data; i++)
  {
    double y1=f_z[i]*y_0-f_y[i]*z_0-tau_x[i];
    double y2=f_x[i]*z_0-f_z[i]*x_0-tau_y[i];
    double y3=f_y[i]*x_0-f_x[i]*y_0-tau_z[i];
    double y= pow((pow( y1,2)+pow(y2 ,2)+pow(y3 ,2)),2);
     //double y= pow( y1,2)+pow(y2 ,2)+pow(y3 ,2);
     gsl_vector_set(f,i, y);
  }	

}


void Weight_Estimator_Server::weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now)
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_	
  //ROS_INFO("Acquiring message");
 
	this->msg_data_acquired_recent=*msg_data_acquired_now;
  if(this->index==N){this->index=0;}
  this->msg_data_acquired_array[this->index]=this->msg_data_acquired_recent; 
  //ROS_INFO("Hand Joint Error = %g",(double)this->msg_data_acquired_recent.hand_error_position);
  if((double)this->msg_data_acquired_recent.hand_error_position<0.001){
   tf::StampedTransform vito_anchor_2_left_ft_sensor;
   transformStampedMsgToTF(this->msg_data_acquired_recent.vito_anchor_2_left_measure,vito_anchor_2_left_ft_sensor);
   tf::Vector3 position = vito_anchor_2_left_ft_sensor.getOrigin();
   tf::Quaternion quaternion = vito_anchor_2_left_ft_sensor.getRotation();
   tf::Matrix3x3 ft_rotation_matrix(quaternion);
   tf::Vector3 force(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z);
   tf::Vector3 torque(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.z);
   tf::Vector3 gravity(0,0,-9.81);
   tf::Vector3 gravity_ft_frame=ft_rotation_matrix.transpose()*gravity;
   force=ft_rotation_matrix.transpose()*force;
   this->mass_measured[this->index]=(sqrt(pow(force[0]/gravity_ft_frame[0],2)+pow(force[1]/gravity_ft_frame[1],2)+pow(force[2]/gravity_ft_frame[2],2)));
   this->weight_measured[this->index]=(sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)));
   this->mass_measured_now=this->mass_measured[this->index];
   this->weight_measured_now=this->weight_measured[this->index];
   ROS_INFO("weight_measured now = %g",this->weight_measured_now); 
   ROS_INFO("mass_measured now  = %g",this->mass_measured_now);
   this->index=this->index+1;
  }
  else{
   ROS_INFO("Waiting for hand joint error position less than 0.001");
  } 
}


void Weight_Estimator_Server::Hand_Calibration()
{ 
  //ROS_INFO("Start Calibration");
  double weight_hand_home,mass_hand_home;
  
  
  if(this->index==N)
  { 
    ROS_INFO("Estimating first point !");
    for (int i = 0; i <N; ++i)
    {
     weight_hand_home=+this->weight_measured[i];
     mass_hand_home=+this->mass_measured[i];
    }
    weight_hand_home=weight_hand_home/N;
    mass_hand_home=mass_hand_home/N;
    ROS_INFO("weight_hand_home  = %g",(double)weight_hand_home);
    ROS_INFO("mass_hand_home  = %g",(double)mass_hand_home);
    this->calibration=true; 
    this->index=0;
    ROS_INFO("Calibration first point : DONE !");
  }


  
 }
void Weight_Estimator_Server::Object_Estimation()
{
   ROS_INFO("Estimating Object");

}


void Weight_Estimator_Server::publishDataState()
{ 
  ROS_INFO("pub");
  // moveit::planning_interface::MoveGroup dual_hand_arm("dual_hand_arm");
  // double weight_measured;
  // double mass_measured;  
     
  //       if(this->index<N&&msg_data_acquired_recent.hand_synergy_joint_state_position<=0.16)
  //       {  
  //         ROS_INFO("Calibrating Hand");
  //         this->index=this->index+1;
  //         this->hand_weight+=weight_measured;
  //         this->hand_mass+=mass_measured;
  //         weight_estimate_now.weight=weight_measured;
  //         weight_estimate_now.time_now=ros::Time::now();
  //         if(this->index==N)
  //         { 
           
  //          this->hand_weight=this->hand_weight/(N);
  //          ROS_INFO("Calibration DONE");
  //          this->hand_mass=this->hand_mass/(N);
  //          weight_estimate_now.weight=this->hand_weight;
  //          weight_estimate_now.time_now=ros::Time::now();
           
  //          this->index=this->index+1;
         
  //          ROS_INFO("Hand Weight = %f",(float)this->hand_weight);
  //          ROS_INFO("Hand Mass = %f",(float)this->hand_mass);
           

  //             ROS_INFO("Estimating cog DONE");

  //         }
         
  //       }


  //       if(this->index>=N)
  //       {
  //        ROS_INFO("Put Object on the hand"); 
  //        this->index=this->index+1; 
  //        double weight_object = fabs(weight_measured-this->hand_weight);
  //        ROS_INFO("Diff weight hand %f",(float)weight_object) ;
  //        if(abs(weight_object)<=0.05*(this->hand_weight))
  //        { 
  //         // gravity_center[0]=(1/this->hand_weight)*center[0];
  //         // gravity_center[1]=(1/this->hand_weight)*center[1];
  //         // gravity_center[2]=(1/this->hand_weight)*center[2];
  //         weight_estimate_now.weight=weight_measured;

  //         ROS_INFO("No object in the hand. weight of the hand: %f",(float)weight_measured); 
  //         moveit::planning_interface::MoveGroup left_hand("left_hand");
  //          left_hand.setNamedTarget("left_hand_preshape");

  //             // call the moves
  //           if( !(left_hand.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
  //           {
  //                 ROS_ERROR("An error occured during Graspig Object");
  //           } 
  //         //ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);
  //        }
  //        else
  //        { 
  //          this->index=this->index+1;
  //          ROS_INFO("Object in the hand: weight without the hand = %f",(float)weight_object);
  //          weight_estimate_now.weight=weight_object;
  //          weight_estimate_now.time_now=ros::Time::now();
           
           

  //             // configure hand move
  //          moveit::planning_interface::MoveGroup left_hand("left_hand");
  //          left_hand.setNamedTarget("left_hand_90_close");

  //             // call the moves
  //           if( !(left_hand.move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
  //           {
  //                 ROS_ERROR("An error occured during Graspig Object");
  //           }
  //          // gravity_center[0]=(1/weight_measured)*center[0];
  //          // gravity_center[1]=(1/weight_measured)*center[1];
  //          // gravity_center[2]=(1/weight_measured)*center[2];
         
  //          //ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);


  //          weight_estimate_now.act=(int)1;
  //          weight_estimate_now.time_now=ros::Time::now();
  //         }
  //       }

  //         // geometry_msgs::PointStamped cog_msgs;
  //         // cog_msgs.point.x=gravity_center[0];
  //         // cog_msgs.point.y=gravity_center[1];
  //         // cog_msgs.point.z=gravity_center[2];
  //         // cog_msgs.header.seq=1;
  //         // cog_msgs.header.stamp=ros::Time::now();
  //         // cog_msgs.header.frame_id="left_measure";
  //         // pub_cog_.publish(cog_msgs);
  //         //ROS_INFO("Estimating cog DONE");
        
       
        pub_weight_states_.publish(weight_estimate_now);
      //}

    }

}//namespace grasp_estimator

int main(int argc, char **argv)
{ 
  ROS_INFO("Weight_Estimator_Server is Here !!!");
  
	ros::init(argc, argv, "Weight_Estimator_Server");
    //grasp_estimator::WeightEstimated::ConstPtr weight_estimate_now;
	ros::NodeHandle nh;
    //tf::TransformStamped msgs_tf;
  
  grasp_estimator::Weight_Estimator_Server Weight_Estimator_Server(nh);

  Weight_Estimator_Server.index = 0;
  Weight_Estimator_Server.hand_weight=0;
  sleep(2.0);
  
  ros::AsyncSpinner spinner(4);  
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
    if(Weight_Estimator_Server.calibration==false){
      Weight_Estimator_Server.Hand_Calibration();
		}
    else{
     Weight_Estimator_Server.Object_Estimation(); 
     Weight_Estimator_Server.publishDataState();
    }
    

		//ROS_INFO("msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x: %f ",(float)msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x); 
        //
		//ros::spinOnce();
    spinner.start();

    //if(Weight_Estimator_Server.index==N-1){interrupt=1;} 
      

		//ROS_INFO("Data Published !");
		loop_rate.sleep();
	}
  Weight_Estimator_Server.calibration=false;
  spinner.stop();

	return 0;
}

    
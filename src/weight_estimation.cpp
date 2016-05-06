

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
#include "grasp_estimator/GraspShape.h"
// ROS headers
#include <ros/ros.h>

// ROS Control
#include "control_msgs/JointTrajectoryControllerState.h"

//Message filters

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Gnu Fitting
// #include <stdlib.h>
// #include <stdio.h>
// #include <gsl/gsl_rng.h>
// #include <gsl/gsl_randist.h>
// #include <gsl/gsl_vector.h>
// #include <gsl/gsl_blas.h>
// #include <gsl/gsl_multifit_nlin.h>
// #include <gsl/gsl_multifit.h>
// #include <gsl/gsl_multiset.h>

// Moveit Header
#include <moveit/move_group_interface/move_group.h>

// marker
#include <visualization_msgs/Marker.h>

#define N 25
//#define N 25
#define left_hand_name "left_hand"
#define dual_hand_arm_name "left_hand_arm"
#define head_name "head"
#define Hz 100
#define N2 8
#define error_thresh 0.001
#define weight_threshold 9.822 // 1.0 Newton circa 100 gr
#define mass_threshold  0.050
#define Hand_threshold 6.80
#define arm_state_num 6
#define N_cog 2
#define N_close 3
#define gravity 9.822
//for simulation
//#define mass_hand_def 2.50// Kg
//#define weight_hand_def  7.50 // Newton
// for experiment
// #define mass_hand_def 0.06// Kg
// #define weight_hand_def  0.314644 // Newton
// open hand COG Xcog = -0.00270399, Ycog = -0.0062375, Zcog = 0.121482
// close hand COG  Xcog = -0.0176698, Ycog = -0.00718917, Zcog = 0.0602982
#define max_state_close_hand 6
#define count_no_object_max 10

#include <std_srvs/Empty.h>

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

	ros::Publisher pub_hand_cog_;
  ros::Publisher pub_object_hand_cog_;
    // subscribers
    ros::Subscriber msg_grasp_state_now_;
    
    ros::Subscriber msg_grasp_shape_now_;

	grasp_estimator::DataAcquired msg_data_acquired_;

	grasp_estimator::DataAcquired msg_data_acquired_recent;
    
    grasp_estimator::WeightEstimated weight_estimate_now;
  
    grasp_estimator::DataAcquired msg_data_acquired_array[N];
    

    grasp_estimator::DataAcquired msg_data_acquired_cog_array[N_cog];
    
    grasp_estimator::GraspShape grasp_shape_now;

      /// declaration of service servers
    ros::ServiceServer srvServer_Start_;
    ros::ServiceServer srvServer_Stop_;
    
    ros::ServiceServer init_exp_;
    
public:
	// callback functions
      void weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now);
      void grasp_estimation_server(const grasp_estimator::GraspShape::ConstPtr &grasp_shape);
  // server function    
      void Hand_Calibration();
      //void Object_Estimation();
      void publishDataState();
      void Hand_Control();
      void Hand_Control_single_task();
      void Arm_Control();
      void Arm_exp_position();
      void hand_cog_calibration();
      void Arm_Control_Moving();
      bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool init_exp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      void init_exp();
      void Head_Control();

      int index;

      int index_calibration; 

      int hand_close_state;

      int hand_close_state_des;

      int arm_state;

      int arm_state_desidered;

      int close_90_state;

      int count_no_object;

      bool calibration;

      bool cog_calibration;
      bool cog_object_calibration;
      
      bool glove_calib;

      bool active_move;

      bool arm_action_move;

      double weight_measured[N];
      double mass_measured[N];

      double weight_measured_now;
      double mass_measured_now;

      double hand_weight[N2];
      double hand_mass[N2]; 

      double weight_object;
      double mass_object;
      double density_object;
      
      double mass_hand_def; 
      double weight_hand_def;  
      bool found_object;

      tf::Vector3 hand_gravity_center;

      tf::Vector3 hand_object_gravity_center;
  
      moveit::planning_interface::MoveGroup *left_hand_ = new moveit::planning_interface::MoveGroup(left_hand_name); 
      moveit::planning_interface::MoveGroup *dual_hand_arm_ = new moveit::planning_interface::MoveGroup(dual_hand_arm_name);
      //moveit::planning_interface::MoveGroup *head_ = new moveit::planning_interface::MoveGroup(head_name);
      
      std::string hand_pose_name[N2];
      std::string dual_arm_pose_name[arm_state_num];

      bool run_state;
          

    // constructor
    Weight_Estimator_Server(ros::NodeHandle nh) : 
    nh_(nh), 
    priv_nh_("~")
	{
                     
   msg_grasp_state_now_ = nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),1,
                                                                                     &Weight_Estimator_Server::weight_estimation_server,
                                                                                     this);
   
   // msg_grasp_shape_now_ = nh_.subscribe<grasp_estimator::GraspShape>(nh_.resolveName("/left_hand/grasp_shape_estimator_server_"),1,&Weight_Estimator_Server::grasp_estimation_server, this);
        
	  srvServer_Start_ = nh_.advertiseService("/Weight_Estimator_Server/start", &Weight_Estimator_Server::srvCallback_Start,this);
    srvServer_Stop_ = nh_.advertiseService("/Weight_Estimator_Server/stop", &Weight_Estimator_Server::srvCallback_Stop, this);
 
   init_exp_= nh_.advertiseService("/Weight_Estimator_Server/init_exp", &Weight_Estimator_Server::init_exp, this);
 

  	// advertise topics
		pub_weight_states_ = nh_.advertise<grasp_estimator::WeightEstimated>(nh_.resolveName("/left_hand/mass_estimator_server_"), 1);
	
		//pub_hand_cog_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/cog_hand_server_"),5);

    pub_hand_cog_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("/left_hand/cog_hand_server_"),1);

    //pub_object_hand_cog_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/cog_object_hand_server_"),5);
  
    //pub_object_hand_cog_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("/left_hand/cog_object_hand_server_"),5);
    //left_hand_ = new moveit::planning_interface::MoveGroup(left_hand_name);  
    
    glove_calib=false;
    run_state=false;
    found_object=false;
	}

	//! Empty stub
	~Weight_Estimator_Server() 
  { 
    delete left_hand_;
    delete dual_hand_arm_;
    //delete head_;
  }

};

  bool Weight_Estimator_Server::srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("Starting  Weight_Estimator_Server");
//   arm_action_move=true;
//   arm_state_desidered=0;
//   Arm_Control();
     mass_hand_def = this->mass_measured_now;
     weight_hand_def  = this->weight_measured_now;
      run_state = true;
      return true;
    }


    bool Weight_Estimator_Server::srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    { 
      	
    //  	dual_hand_arm_->setPlannerId("PRMstarkConfigDefault");
  		// ROS_INFO_STREAM("Arm state desidered:  dual_hand_arm_exp");
  		// dual_hand_arm_->setNamedTarget("left_hand_arm_exp");
 			//       // call the move
  		// if( !(dual_hand_arm_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
  	 //    {
 		 //  ROS_ERROR("An error occured during Moving Arm");
  	 //    }
      
    //   sleep(2.0);
    //   sleep(2.0);
      run_state = false;
      this->active_move=true;
      this->hand_close_state_des=0;
      this->Hand_Control_single_task();
      this->index=0;this->active_move=false;
     
      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
      this->close_90_state=0;
      ROS_INFO("Stopping Weight_Estimator_Server");
      
      found_object=false;
      return true;
    }

bool Weight_Estimator_Server::init_exp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::string tare_service_name("/left_ft_sensor/left/tare");
  std::string start_calibration_service_name("/left_hand/start_glove_calibration"); 
  std::string next_move_service_name("/left_hand/next_orientation");
  std::string set_world_reference_service_name("/left_hand/set_world_reference");
  //std_srvs::Empty::Request req_;
  std_srvs::Empty srv;
  
  if(glove_calib==false)
  {   	      
  arm_action_move=true; 
  arm_state_desidered=5;
  Arm_Control();
  sleep(2.0);
  
  if (!ros::service::call(start_calibration_service_name,srv))
   {
     ROS_ERROR("Call to IMU calibration service start failed");
     exit(0);
    }
  else 
    {
     ROS_INFO("Call to IMU calibration service  DONE");
    }
    
  arm_action_move=true; 
  arm_state_desidered=2;
  Arm_Control();
  sleep(2.0);
   //sleep(1.0); 
   if (!ros::service::call(next_move_service_name, srv))
    {
     ROS_ERROR("Call to next move service  failed");
     exit(0);
    }
    else 
    {
       ROS_INFO("Call to  next move service  DONE");
    }
   //sleep(1.0); 
   if (!ros::service::call(set_world_reference_service_name, srv))
    {
     ROS_ERROR("Call to get world reference service  failed");
     exit(0);
    }
   else 
    {
       ROS_INFO("Call to set world reference service  DONE");
    }
  glove_calib=true;
  }
    if (!ros::service::call(tare_service_name, srv))
    {
     ROS_ERROR("Call to FT Sensor tare service  failed");
     exit(0);
    }
   else 
    {
      ROS_INFO("Call to FT Sensor tare service  DONE");

     
     mass_hand_def = 0.0;
     weight_hand_def  = 0.0;

     ROS_INFO("OFF set hand DONE "); 
     
    }
  
  
  

ROS_INFO("Init Done");  

  return true;
}

 void Weight_Estimator_Server::init_exp()
{
  std::string tare_service_name("/left_ft_sensor/left/tare");
  std::string start_calibration_service_name("/left_hand/start_glove_calibration"); 
  std::string next_move_service_name("/left_hand/next_orientation");
  std::string set_world_reference_service_name("/left_hand/set_world_reference");
  //std_srvs::Empty::Request req_;
  std_srvs::Empty srv;
  
  if(glove_calib==false)
  {           
  arm_action_move=true; 
  arm_state_desidered=0;
  Arm_Control();
  sleep(5.0);
  
  if (!ros::service::call(start_calibration_service_name,srv))
   {
     ROS_ERROR("Call to IMU calibration service start failed");
     exit(0);
    }
  else 
    {
     ROS_INFO("Call to IMU calibration service  DONE");
    }
    
  arm_action_move=true; 
  arm_state_desidered=0;
  Arm_Control();
  sleep(5.0);
   //sleep(1.0); 
   if (!ros::service::call(next_move_service_name, srv))
    {
     ROS_ERROR("Call to next move service  failed");
     exit(0);
    }
    else 
    {
       ROS_INFO("Call to  next move service  DONE");
    }
   //sleep(1.0); 
   if (!ros::service::call(set_world_reference_service_name, srv))
    {
     ROS_ERROR("Call to get world reference service  failed");
     exit(0);
    }
   else 
    {
       ROS_INFO("Call to set world reference service  DONE");
    }
  glove_calib=true;
  }
    if (!ros::service::call(tare_service_name, srv))
    {
     ROS_ERROR("Call to FT Sensor tare service  failed");
     exit(0);
    }
   else 
    {
      ROS_INFO("Call to FT Sensor tare service  DONE");

     
     mass_hand_def = 0.0;
     weight_hand_def  = 0.0;

     ROS_INFO("OFF set hand DONE "); 
     
    }
  
  
  

ROS_INFO("Init Done");  
}


void Weight_Estimator_Server::weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now)
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_	

 if(this->index==N){this->index=0;}
 if(found_object==false)
 {
   std::string tare_service_name("/left_ft_sensor/left/tare");
       std_srvs::Empty srv;
      if (!ros::service::call(tare_service_name, srv))
     {
      ROS_ERROR("Call to FT Sensor tare service  failed");
      exit(0);
   }
 }
  this->msg_data_acquired_recent=*msg_data_acquired_now;
  this->weight_estimate_now.time_now=ros::Time::now();
  //ROS_INFO("Hand Joint Error = %g",(double)this->msg_data_acquired_recent.hand_error_position);
   //ROS_INFO("hand_synergy_joint_state_position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position);
     this->msg_data_acquired_array[this->index]=this->msg_data_acquired_recent;  
      //ROS_INFO("Acquiring message %d",this->index); 
       tf::StampedTransform vito_anchor_2_left_ft_sensor;
       transformStampedMsgToTF(this->msg_data_acquired_recent.vito_anchor_2_left_measure,vito_anchor_2_left_ft_sensor);
       tf::Vector3 position = vito_anchor_2_left_ft_sensor.getOrigin();
       tf::Quaternion quaternion = vito_anchor_2_left_ft_sensor.getRotation();
       tf::Matrix3x3 ft_rotation_matrix(quaternion);
       tf::Vector3 force(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z);
       tf::Vector3 torque(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.z);
       tf::Vector3 gravity_vec(0,0,-gravity);
       tf::Vector3 gravity_ft_frame=ft_rotation_matrix*gravity_vec;
       force=ft_rotation_matrix*force;
       //this->mass_measured[this-
       //>index]=(sqrt(pow(force[0]/gravity_ft_frame[0],2)+pow(force[1]/gravity_ft_frame[1],2)+pow(force[2]/gravity_ft_frame[2],2)//));
        this->mass_measured[this->index]=((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))/9.822);
       this->weight_measured[this->index]=(sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)));
       this->mass_measured_now=this->mass_measured[this->index];
       this->weight_measured_now=this->weight_measured[this->index];
      
        //ROS_INFO("weight_measured now = %g",this->weight_measured_now); 
        ROS_INFO("mass_measured now  = %g",this->mass_measured_now);
       this->index=this->index+1; 
//        double diff_weight(this->weight_measured_now-hand_weight[this->hand_close_state]);
//        double diff_mass(this->mass_measured_now-this->hand_mass[this->hand_close_state]);
       double diff_weight((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))-(double)weight_hand_def);
       double diff_mass((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))/9.822-(double)mass_hand_def);
       //ROS_INFO("weight object estimated now : %g",diff_weight);
       ROS_INFO("Mass object estimated now : %g",diff_mass);
         if(diff_mass>=mass_threshold) //&& diff_weight>=weight_threshold
         {
           //ROS_INFO("Object Detected in  the hand");
           this->weight_object=diff_weight;
	   
           //ROS_INFO("Estimating Object Weight= %g",this->weight_object);
           this->mass_object=diff_mass;
           //this->mass_object=this->mass_measured_now-Hand_threshold/9.81;
           ROS_INFO("<--------- Found  Object ------------>");
	      ROS_INFO("Estimating Object mass= %g",this->mass_object);
           this->weight_estimate_now.weight=diff_weight;
	   this->weight_estimate_now.mass=diff_mass;
      this->weight_estimate_now.density=0.0;
	   // this->density_object=(diff_mass/this->grasp_shape_now.v);
	   //this->weight_estimate_now.density=this->density_object;
	   //ROS_INFO("Object_density %g",this->density_object);
	   //ROS_INFO("<---------   End  Object ------------>");
	   this->active_move=true;
           this->count_no_object=0;
           found_object=true;
          }
          else
          {
	    if(count_no_object<count_no_object_max&&this->index>0)
	    {
// 	       ROS_INFO("<------- Remeber Found  Object ---------->");
	   this->weight_estimate_now.weight=this->weight_measured[this->index-1]-weight_hand_def;
	   this->weight_estimate_now.mass=this->mass_measured[this->index-1]-mass_hand_def;
	   this->density_object=(this->weight_estimate_now.mass/this->grasp_shape_now.v);
     found_object=true;
	     // ROS_INFO("<---------   End  Object ------------>");
	    }
	    else
	    {
	     ROS_INFO("No Object Detected in the hand");
// 	    ROS_INFO("<--------- NO Found  Object ------------>");
              found_object=false;
            this->weight_object=0.0;
            this->mass_object=0.0;
	    this->density_object=0.0;
	     this->weight_estimate_now.weight=0.0;
	     this->weight_estimate_now.mass=0.0;
	     this->weight_estimate_now.density=0.0;
            this->count_no_object=this->count_no_object+1;
            //ROS_INFO("No Object on the hand");
              this->active_move=true;
	   //ROS_INFO("<------- END NO Found  Object --------->");
            if(this->weight_estimate_now.hand_stopped==true)
	    {
	      this->active_move=false;
	    }
	    }
	    if(count_no_object==count_no_object_max){this->active_move=true; this->count_no_object=0;
// 	      this->arm_action_move=true; 
// 	      this->arm_state_desidered=0;
//                this->Arm_Control();
// 	       this->arm_action_move = false;
	    }  
	      //this->cog_object_calibration=false;
// //              if(this->arm_state!=0&&this->cog_calibration==true && count_no_object==3){
// //              this->arm_action_move=true;

// //              this->cog_object_calibration=false;
	    
            //}
        }    
         //geometry_msgs::PointStamped position_hand_msgs,position_object_hand_msgs;
         
         // if(this->cog_calibration==true)
         // {
         //    position_hand_msgs.point.x= (double)this->hand_gravity_center.x();
         //    position_hand_msgs.point.y= (double)this->hand_gravity_center.y();
         //    position_hand_msgs.point.z= (double)this->hand_gravity_center.z();
         //    position_hand_msgs.header.seq=0;
         //    position_hand_msgs.header.stamp=ros::Time::now();
         //    position_hand_msgs.header.frame_id="left_measure";
         //    pub_hand_cog_.publish(position_hand_msgs);
         //  }
         //  if(this->hand_object_gravity_center.x()!=0.0 && this->hand_object_gravity_center.y()!=0.0 && this->hand_object_gravity_center.z()!=0.0)
         //  { 
         //    position_object_hand_msgs.point.x= (double)this->hand_object_gravity_center.x();
         //    position_object_hand_msgs.point.y= (double)this->hand_object_gravity_center.y();
         //    position_object_hand_msgs.point.z= (double)this->hand_object_gravity_center.z();
         //    position_object_hand_msgs.header.seq=1;
         //    position_object_hand_msgs.header.stamp=ros::Time::now();
         //    position_object_hand_msgs.header.frame_id="left_measure";
         //    pub_object_hand_cog_.publish(position_object_hand_msgs);
         //  } 

 //if(this->cog_calibration==true)
//   if(this->hand_gravity_center.x()!=0.0 && this->hand_gravity_center.y()!=0.0 && this->hand_gravity_center.z()!=0.0)
//  {
//   //ROS_INFO("Pub hand cog ");
//  visualization_msgs::Marker position_hand_msgs;
//  position_hand_msgs.header.frame_id = "left_measure";
//  position_hand_msgs.header.stamp = ros::Time();
//  position_hand_msgs.ns = "Weight_Estimator_Server";
//  position_hand_msgs.id = 0;
//  position_hand_msgs.type = visualization_msgs::Marker::SPHERE;
//  position_hand_msgs.action = visualization_msgs::Marker::ADD;
//  position_hand_msgs.pose.position.x = (double)this->hand_gravity_center.x();
//  position_hand_msgs.pose.position.y = (double)this->hand_gravity_center.y();
//  position_hand_msgs.pose.position.z = (double)this->hand_gravity_center.z();
//  position_hand_msgs.pose.orientation.x = 0.0;
//  position_hand_msgs.pose.orientation.y = 0.0;
//  position_hand_msgs.pose.orientation.z = 0.0;
//  position_hand_msgs.pose.orientation.w = 1.0;
//  position_hand_msgs.scale.x = 2*0.05;
//  position_hand_msgs.scale.y = 2*0.05;
//  position_hand_msgs.scale.z = 2*0.05;
//  position_hand_msgs.color.a = 1.0; // Don't forget to set the alpha!
//  position_hand_msgs.color.r = 0.5f;
//  position_hand_msgs.color.g = 0.5f;
//  position_hand_msgs.color.b = 0.0f;
//  position_hand_msgs.lifetime = ros::Duration(0.2);
//  pub_hand_cog_.publish(position_hand_msgs);
//  }

}

void Weight_Estimator_Server::grasp_estimation_server(const grasp_estimator::GraspShape::ConstPtr &grasp_shape)
{
 grasp_shape_now = *grasp_shape;
}


void Weight_Estimator_Server::Hand_Control_single_task()
{
  if(this->active_move==true){
   ROS_INFO("Hand state desidered %d",this->hand_close_state_des);
     this->left_hand_->setPlanningTime((double)5);
   this->left_hand_->setGoalPositionTolerance((double)0.25);
   this->left_hand_->setNamedTarget(this->hand_pose_name[this->hand_close_state_des]);
      // call the moves
   //
   if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
   {
   ROS_ERROR("An error occured during Moving Hand");
   }
   else
   {
     ROS_INFO("Left hand moved to :");
     ROS_INFO_STREAM(this->hand_pose_name[this->hand_close_state_des]);  
     this->active_move=true;
     this->hand_close_state=this->hand_close_state_des;
    }    
  }
}

// void Weight_Estimator_Server::Head_Control()
// {
//    this->head_->setNamedTarget("head_home_exp");
//       // call the moves
// 
//    if( !(this->head_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
//    {
//    ROS_ERROR("An error occured during Moving Head");
//    }
//    else
//    {
//      ROS_INFO("Head moved to :");
//      ROS_INFO_STREAM("head_home_exp");  
//     }    
//   
// }

// hand calibration works well with 25 data recorded
// void Weight_Estimator_Server::Hand_Calibration()
// { 
//   //ROS_INFO("Index_calibration %d",this->index_calibration);
//   //ROS_INFO("Index_calibration %d",this->index_count);
//   //ROS_INFO("Close state %d",this->hand_close_state);
//   double weight_hand,mass_hand;
//   if(this->active_move==false && this->index==N && this->index_calibration==this->hand_close_state)// && this->msg_data_acquired_recent.hand_error_position<=error_thresh
//   { 
//    ROS_INFO("Hand Synergy Joint Position calibration= %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//    ROS_INFO("Estimating %d point !",this->index_calibration);
//    for (int i = 0; i <N; ++i)
//    {
//     weight_hand=weight_hand+this->weight_measured[i];
//     mass_hand=mass_hand+this->mass_measured[i];
//    }
//    weight_hand=weight_hand/N;
//    mass_hand=mass_hand/N;
//        ROS_INFO("weight_hand  = %g",(double)weight_hand);
//        ROS_INFO("mass_hand  = %g",(double)mass_hand);
//        this->hand_weight[this->index_calibration]=weight_hand;
//        this->hand_mass[this->index_calibration]=mass_hand;
//        ROS_INFO("Calibration %d point : DONE !\n",this->index_calibration);
//        //this->index=0;
//        this->index_calibration=this->index_calibration+1;
//        this->active_move=true;  
//    }
//    else
//    {
//    //this->index=0;
//    this->active_move=false; 
//    }
// 
//   if(this->active_move==true){
//   //ROS_INFO("Close state %d",this->hand_close_state);  
//   switch(this->index_calibration) 
//   {
//     case 0 : 
//     { 
//       if(this->index==N)
//       {   
//        //ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//           ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//           this->left_hand_->setNamedTarget("left_hand_home");
//           // call the moves
//           if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//           {
//            ROS_ERROR("An error occured during Calibrating hand");
//           } 
//           else{ROS_INFO("Hand Moved to home position");
//           this->active_move=false;
//           this->hand_close_state=this->index_calibration;
//           this->index=0;}
//         } 
//     }
//      case 1 : 
//      {
//       if(this->index==N){
//        ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position ); 
//        this->left_hand_->setNamedTarget("left_hand_preshape");
//         // call the moves
//        
//        if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//        {
//          ROS_ERROR("An error occured during Calibrating hand");
//        }
//       else{ROS_INFO("Hand  moved to preshape position");this->active_move=false;this->hand_close_state=this->index_calibration;
//      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//       }
//      }
//     case 2 :
//     {
//      if(this->index==N){ 
//      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//      this->left_hand_->setNamedTarget("left_hand_25_close");
//      // call the moves
// 
//      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//      {
//        ROS_ERROR("An error occured during Calibrating hand");
//      } 
//      else{ROS_INFO("Hand  moved  to 25 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//    ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     } 
//     case 3 :
//     {
//      if(this->index==N){
//      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//       this->left_hand_->setNamedTarget("left_hand_56_close");
//      // call the moves
// 
//       if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//       {
//         ROS_ERROR("An error occured during Calibrating hand");
//       }
//        else{ROS_INFO("Hand  moved  to 56 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//             ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     }
//     case 4 :
//     { if(this->index==N){
//      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position);  
//      this->left_hand_->setNamedTarget("left_hand_70_close");
//      // call the moves
// 
//      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//      {
//        ROS_ERROR("An error occured during Calibrating hand");
//      } 
//       else{ROS_INFO("Hand  moved  to 70 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     }
//     case 5 :
//     { 
//       if(this->index==N){
//         ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//      this->left_hand_->setNamedTarget("left_hand_80_close");
//      // call the moves
// 
//      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)   && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//      {
//        ROS_ERROR("An error occured during Calibrating hand");
//      }
//       else{ROS_INFO("Hand  moved  to 80 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//       ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     } 
//     case 6 : 
//     { 
//      if(this->index==N){ 
//       ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//      this->left_hand_->setNamedTarget("left_hand_90_close");
//      // call the moves
// 
//      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)    && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//      {
//        ROS_ERROR("An error occured during Calibrating hand");
//      }
//       else{ROS_INFO("Hand  moved  to 90 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     }
//     case 7 : 
//     { 
//       if(this->index==N){
//         ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
//      this->left_hand_->setNamedTarget("left_hand_95_close");
//      // call the moves
// 
//      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  && this->msg_data_acquired_recent.hand_error_position<=error_thresh)
//      {
//        ROS_ERROR("An error occured during Calibrating hand");
//      }
//       else{ROS_INFO("Hand  moved  to 95 close position");this->active_move=false;this->hand_close_state=this->index_calibration;
//     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); }
//      }
//     } 
//     
//     case 8 : 
//     { 
//      if(this->index==N)
//      {
//        ROS_INFO("Calibration weight DONE %g, %g, %g, %g, %g, %g, %g, %g",this->hand_weight[0],this->hand_weight[1],this->hand_weight[2],
//        this->hand_weight[3],this->hand_weight[4],this->hand_weight[5],this->hand_weight[6],this->hand_weight[7]);
//        ROS_INFO("Calibration mass DONE %g, %g, %g, %g, %g, %g, %g, %g",this->hand_mass[0],this->hand_mass[1],this->hand_mass[2],
//        this->hand_mass[3],this->hand_mass[4],this->hand_mass[5],this->hand_mass[6],this->hand_mass[7]);
//        this->calibration=true;
//        this->active_move=true;
//        this->arm_action_move=true;
//        //this->hand_close_state=this->index_calibration-1;
//        this->hand_close_state_des=0;
//        this->Hand_Control_single_task();
//        
//        
//      }    
//     }  
//    } 
//   }
// }

void Weight_Estimator_Server::Hand_Control()
{
 int i; 
  
 if(run_state==true){ 
  if(this->active_move==true&&this->mass_object>mass_threshold && this->msg_data_acquired_recent.hand_error_position < error_thresh)
  { 
    i=this->hand_close_state+3;
    
    if(i==max_state_close_hand+2){i=max_state_close_hand;
        //ROS_INFO("Hand is in 95 Close Position: No need planning");
        i=max_state_close_hand;this->close_90_state=this->close_90_state+1;
	if(this->close_90_state== N_close){this->active_move=false;}
        if(this->close_90_state>N_close){this->close_90_state=0;this->weight_estimate_now.hand_stopped=true;}
      }
    else
    {
         this->hand_close_state_des=i;
         this->Hand_Control_single_task();
         this->index=0;this->active_move=false;
         ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
         this->close_90_state=0;

     }
  }
   if(this->active_move==true&&this->mass_object<mass_threshold)
   { 
     //i=this->hand_close_state-1;
     i=0;
     if(this->close_90_state=0)
     {
       //ROS_INFO("Hand is in Home position. No need for planning");
       this->hand_close_state=0;
       // if(this->arm_state!=0)
       // { 
       //   this->arm_action_move=true;
       //   this->arm_state_desidered=0;
       //   this->Arm_Control();
       //   this->cog_object_calibration=false;
       //  } 
      }
     else
     {
      this->hand_close_state_des=i;
      this->Hand_Control_single_task();
      this->index=0;this->active_move=false;
      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
      this->close_90_state=0;
     }
   }
  }
  // else
  // {

  // }
}

void Weight_Estimator_Server::Arm_Control()
{   if(this->arm_action_move==true){

   ROS_INFO("Arm state desidered %d",this->arm_state_desidered);
   this->dual_hand_arm_->setNamedTarget(this->dual_arm_pose_name[this->arm_state_desidered]);
      // call the moves

   if( !(this->dual_hand_arm_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
   {
   ROS_ERROR("An error occured during Moving Arm");
   }
   else
   {
     ROS_INFO("Dual arm moved to :");
     ROS_INFO_STREAM(this->dual_arm_pose_name[this->arm_state_desidered]);  
     this->arm_action_move=false;
     this->arm_state=this->arm_state_desidered;
     ROS_INFO("Arm state %d",this->arm_state);
   }    
  }
}


void Weight_Estimator_Server::Arm_Control_Moving()
{
  if(this->arm_action_move==true){
   this->arm_state_desidered=this->arm_state+1;
   if(this->arm_state_desidered==5){this->arm_state_desidered=0;}
   ROS_INFO("Arm state desidered %d",this->arm_state_desidered);
   this->dual_hand_arm_->setNamedTarget(this->dual_arm_pose_name[this->arm_state_desidered]);
      // call the moves

   if( !(this->dual_hand_arm_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
   {
   ROS_ERROR("An error occured during Moving Arm");
   }
   else
   {
     ROS_INFO("Dual arm moved to :");
     ROS_INFO_STREAM(this->dual_arm_pose_name[this->arm_state_desidered]);  
     this->arm_action_move=false;
     this->arm_state=this->arm_state_desidered;
     ROS_INFO("Arm state %d",this->arm_state);
   }    
  }
}



// void Weight_Estimator_Server::hand_cog_calibration()
// { 
//   this->active_move=false;
//   double Fx1,Fy1,Fz1,Tx1,Ty1,Tz1,Fx2,Fy2,Fz2,Tx2,Ty2,Tz2,weight_object;
//   geometry_msgs::PointStamped position_msgs;
  
//   // this->hand_close_state_des=7; 
//   // this->Hand_Control_single_task();


//   this->arm_action_move=true;
//   this->arm_state_desidered=3;
//   this->Arm_Control();
//   this->arm_action_move=false;
//   sleep(3.0);
//   // if(this->cog_calibration=true && this->weight_object==0)
//   // {
//   //  this->arm_action_move=true;
//   //  this->arm_state_desidered=0;
//   //  this->Arm_Control();
//   //  this->arm_action_move=false;
//   //  //exit(0);
//   // }

//   if(this->arm_state==3&&this->arm_action_move==false)
//   {

//    ROS_INFO("Measuring %d for estimating cog",this->arm_state);

// //     for (int i = 0; i <N; ++i)
// //     {
// //       Fx1=Fx1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.x;
// //       Fy1=Fy1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.y;
// //       Fz1=Fz1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.z;
// //       Tx1=Tx1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.x;
// //       Ty1=Ty1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.y;
// //       Tz1=Tz1+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.z;
// //      }
// // 
// //     Fx1=-Fx1/N;
// //     Fy1=-Fy1/N;
// //     Fz1=-Fz1/N;
// //     Tx1=-Tx1/N;
// //     Ty1=-Ty1/N;
// //     Tz1=-Tz1/N;


//     Fx1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x;
//     Fy1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y;
//     Fz1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z;
//     Tx1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x;
//     Ty1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.y;
//     Tz1=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.z;

//    ROS_INFO("Fx1 = %g, Fy1 = %g, Fz1 = %g,Tx1 = %g,Ty1 = %g,Tz1 = %g.",Fx1,Fy1,Fz1,Tx1,Ty1,Tz1);
//    this->arm_action_move=true;
//   }


//   this->arm_state_desidered=1;
//   this->Arm_Control();
//   this->arm_action_move=false;
//   sleep(3.0);

//   // if(this->cog_calibration=true && this->weight_object==0)
//   // {
//   //  this->arm_action_move=true;
//   //  this->arm_state_desidered=0;
//   //  this->Arm_Control();
//   //  this->arm_action_move=false;
//   //  //exit(0);
//   // }

//   ROS_INFO("Measuring %d for estimating cog",this->arm_state);
//   if(this->arm_state==1&&this->arm_action_move==false)
//   {
   
// //      for (int i = 0; i <N; ++i)
// //    {
// //       Fx2=Fx2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.x;
// //       Fy2=Fy2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.y;
// //       Fz2=Fz2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.z;
// //       Tx2=Tx2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.x;
// //       Ty2=Ty2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.y;
// //       Tz2=Tz2+this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.z;
// //        //weight_object= weight_object+this->weight_measured[i];
// //        mass_object= mass_object+this->mass_measured[i];
// //     }
// // 
// //     Fx2=-Fx2/N;
// //     Fy2=-Fy2/N;
// //     Fz2=-Fz2/N;
// //     Tx2=-Tx2/N;
// //     Ty2=-Ty2/N;
// //     Tz2=-Tz2/N;
// //    weight_object=weight_object/N;
// //     mass_object=mass_object/N;
  
//     Fx2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x;
//     Fy2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y;
//     Fz2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z;
//    Tx2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x;
//     Ty2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.y;
//     Tz2=-this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.z;
//    ROS_INFO("Fx2 = %g, Fy2 = %g, Fz2 = %g,Tx2 = %g,Ty2 = %g,Tz2 = %g.",Fx2,Fy2,Fz2,Tx2,Ty2,Tz2);
   
//    // Fz1*y_0-Fy1*z_0=Tx1;
//    // Fx1*z_0-Fz1*x_0=Ty1;
//    // Fy2*x_0-Fx2*y_0=Tz2;

//    tf::Vector3 Torque(Tx1,Ty1,Tz2);
//    tf::Vector3 cog;
//    //tf::Matrix3x3 measure_matrix(0,Fz1,-Fy1, -Fz1,0,Fx1, Fy2,-Fx2,0);
//    tf::Matrix3x3 measure_matrix(0,Fz1,-Fy1, -Fz1,0,Fx1, Fy2,-Fx2,0);
//     // tf::Matrix3x3 transp_matrix(measure_matrix.transpose());
//     // tf::Matrix3x3 help_matrix(transp_matrix*measure_matrix);
//     // cog=help_matrix.inverse()*transp_matrix*Torque/(fabs(this->weight_measured_now)); // 
//   //cog=measure_matrix.inverse()*Torque; 
//   //cog=measure_matrix.inverse()*Torque/(this->weight_measured_now);
//   //cog=measure_matrix.inverse()*Torque/(this->mass_measured_now); // working not very well
//   cog=measure_matrix.inverse()*Torque/(fabs(this->mass_measured_now));   
  
  
//    //cog=measure_matrix.inverse()*Torque;
//    //cog=measure_matrix.inverse()*Torque/(mass_object);
//    ROS_INFO("Cog : Xcog = %g, Ycog = %g, Zcog = %g", (double)cog.x(), (double)cog.y(), (double)cog.z());
   
// //       if(this->cog_calibration==false) {
//     this->hand_gravity_center=cog;
//     this->cog_calibration=true;
// //    }
// //    else{
// //      this->hand_object_gravity_center=cog;
// //      this->cog_object_calibration=true;
// //    }
//   // this->active_move=true;
//   this->arm_state_desidered=0;
//   this->Arm_Control();
//   this->arm_action_move=false;
//    sleep(3.0);

//  this->cog_calibration=false;


//  }
  

// //this->active_move=true;  
// }


void Weight_Estimator_Server::publishDataState()
{ 
  //weight_estimate_now.calibration=this->calibration;
  pub_weight_states_.publish(weight_estimate_now);
}

}//namespace grasp_estimator




int main(int argc, char **argv)
{ 
  ROS_INFO("Weight_Estimator_Server is Here !!!");
  
	ros::init(argc, argv, "Weight_Estimator_Server");
    //grasp_estimator::WeightEstimated::ConstPtr weight_estimate_now;
	ros::NodeHandle nh;
    //tf::TransformStamped msgs_tf;
  //sleep(5.0);
  ros::Rate loop_rate(Hz);
  ros::AsyncSpinner spinner(8);
  grasp_estimator::Weight_Estimator_Server Weight_Estimator_Server(nh);
  //sleep(2.0); 
  Weight_Estimator_Server.index = 0;
  Weight_Estimator_Server.index_calibration = 0;
  //Weight_Estimator_Server.calibration=false;
  // to test only cog directly
  Weight_Estimator_Server.calibration=true;
  
  Weight_Estimator_Server.hand_close_state=0;

  Weight_Estimator_Server.hand_pose_name[0]="left_hand_home";
  Weight_Estimator_Server.hand_pose_name[1]="left_hand_preshape";
  Weight_Estimator_Server.hand_pose_name[2]="left_hand_25_close";
  Weight_Estimator_Server.hand_pose_name[3]="left_hand_60_close";
  Weight_Estimator_Server.hand_pose_name[4]="left_hand_70_close";
  Weight_Estimator_Server.hand_pose_name[5]="left_hand_80_close";
  Weight_Estimator_Server.hand_pose_name[6]="left_hand_90_close";
  Weight_Estimator_Server.hand_pose_name[7]="left_hand_95_close";
  Weight_Estimator_Server.dual_arm_pose_name[0]="left_hand_arm_exp";
  Weight_Estimator_Server.dual_arm_pose_name[1]="left_hand_arm_exp_1";
  Weight_Estimator_Server.dual_arm_pose_name[2]="left_hand_arm_exp_2";
  Weight_Estimator_Server.dual_arm_pose_name[3]="left_hand_arm_exp_3";
  Weight_Estimator_Server.dual_arm_pose_name[4]="left_hand_arm_exp_4";
  Weight_Estimator_Server.dual_arm_pose_name[5]="left_hand_arm_exp_5";
  Weight_Estimator_Server.arm_action_move=false;
  Weight_Estimator_Server.cog_calibration=false;
  Weight_Estimator_Server.close_90_state=0;
  Weight_Estimator_Server.cog_object_calibration=false;
  //Weight_Estimator_Server.hand_object_gravity_center.x=0;
  Weight_Estimator_Server.hand_object_gravity_center.setValue(0.0,0.0,0.0);
  Weight_Estimator_Server.hand_gravity_center.setValue(0.0,0.0,0.0);
 


  spinner.start();

  Weight_Estimator_Server.init_exp();
  sleep(5.0); 

    

  
  // Weight_Estimator_Server.Head_Control();
  //  sleep(5.0);
   
   //Weight_Estimator_Server.arm_action_move=true;
//    Weight_Estimator_Server.arm_state_desidered=0;
//    Weight_Estimator_Server.Arm_Control();
   //sleep(5.0); 
   Weight_Estimator_Server.active_move=true;
//    Weight_Estimator_Server.hand_close_state_des=0;
  
//    Weight_Estimator_Server.Hand_Control_single_task();
   //sleep(5.0);
   //Weight_Estimator_Server.active_move=true;
  
  while (ros::ok())
	 {
        Weight_Estimator_Server.Hand_Control();
//  	if(Weight_Estimator_Server.cog_calibration==false && Weight_Estimator_Server.close_90_state==N_close && Weight_Estimator_Server.mass_object!=0.0){Weight_Estimator_Server.hand_cog_calibration();}
       
//         if(Weight_Estimator_Server.close_90_state==N_close && Weight_Estimator_Server.mass_object!=0.0)
// 	{ Weight_Estimator_Server.arm_action_move=true;
// 	  Weight_Estimator_Server.Arm_Control_Moving();}
  
        Weight_Estimator_Server.publishDataState();
	 	loop_rate.sleep();
        }
  
//    Weight_Estimator_Server.arm_state_desidered=5;
//    Weight_Estimator_Server.Arm_Control();
//    Weight_Estimator_Server.calibration=false;
//    Weight_Estimator_Server.cog_calibration=false;
//    Weight_Estimator_Server.cog_object_calibration=false;
  spinner.stop();
	return 0;
}

    
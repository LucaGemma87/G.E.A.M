

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
#define Hz 10
#define N2 8
#define error_thresh 0.001
#define weight_threshold 9.822 // 1.0 Newton circa 100 gr
#define mass_threshold  0.070
#define Hand_threshold 6.80
#define arm_state_num 6
#define N_cog 2
#define N_close 3
#define gravity 9.822

#define alpha_ 15.0
#define alpha2_ 400.0
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

  ros::Publisher pub_hand_des_state_;
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

      double hand_des_state;

      tf::Vector3 hand_gravity_center;

      tf::Vector3 hand_object_gravity_center;
  
     
      moveit::planning_interface::MoveGroup *dual_hand_arm_ = new moveit::planning_interface::MoveGroup(dual_hand_arm_name);
      
      
     
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
 


     pub_hand_des_state_=nh_.advertise<trajectory_msgs::JointTrajectory>(nh_.resolveName("/left_hand/joint_trajectory_controller/command"),1);

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
    hand_des_state=0.0;
    index=0;
	}

	//! Empty stub
	~Weight_Estimator_Server() 
  { 
    delete dual_hand_arm_;
    //delete head_;
  }

};

  bool Weight_Estimator_Server::srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("Starting  Weight_Estimator_Server");
      std::string tare_service_name("/left_ft_sensor/left/tare");
      std_srvs::Empty srv;
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
      mass_hand_def = 0.0;
     weight_hand_def  = 0.0;
      run_state = true;
      return true;
    }


    bool Weight_Estimator_Server::srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    { 
      run_state = false;
      found_object=false;

      hand_des_state=0.0;
      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
      
      ROS_INFO("Stopping Weight_Estimator_Server");
      
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
  //arm_state_desidered=1;
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
  sleep(5.0);

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
}



void Weight_Estimator_Server::weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now)
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_	

 if(this->index==N){this->index=0;}
         if(found_object==false&&this->index==0)
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
       this->mass_measured_now=((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))/9.822);
       this->weight_measured_now=this->weight_measured[this->index];
      
        //ROS_INFO("weight_measured now = %g",this->weight_measured_now); 
        //ROS_INFO("mass_measured now  = %g",this->mass_measured_now);
       this->index=this->index+1; 
//        double diff_weight(this->weight_measured_now-hand_weight[this->hand_close_state]);
//        double diff_mass(this->mass_measured_now-this->hand_mass[this->hand_close_state]);
       double diff_weight((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))-(double)weight_hand_def);
       double diff_mass((sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)))/9.822-(double)mass_hand_def);
       //ROS_INFO("weight object estimated now : %g",diff_weight);
       //ROS_INFO("Mass object estimated now : %g",diff_mass);
         if(diff_mass>=mass_threshold) //&& diff_weight>=weight_threshold
         {
           //ROS_INFO("Object Detected in  the hand");
           this->weight_object=diff_weight;
	   
           //ROS_INFO("Estimating Object Weight= %g",this->weight_object);
           this->mass_object=diff_mass;
           //this->mass_object=this->mass_measured_now-Hand_threshold/9.81;
           //ROS_INFO("<--------- Found  Object ------------>");
	      //ROS_INFO("Estimating Object mass= %g",this->mass_object);
           this->weight_estimate_now.weight=this->weight_measured_now;
	   this->weight_estimate_now.mass=this->mass_measured_now;
      this->weight_estimate_now.density=0.0;
	   // this->density_object=(diff_mass/this->grasp_shape_now.v);
	   //this->weight_estimate_now.density=this->density_object;
	   //ROS_INFO("Object_density %g",this->density_object);
	   //ROS_INFO("<---------   End  Object ------------>");

          }
          else   
	    {
	     //ROS_INFO("No Object Detected in the hand");
// 	    ROS_INFO("<--------- NO Found  Object ------------>");

            this->weight_object=0.0;
            this->mass_object=0.0;
	    this->density_object=0.0;
	     this->weight_estimate_now.weight=0.0;
	     this->weight_estimate_now.mass=0.0;
	     this->weight_estimate_now.density=0.0;
            this->count_no_object=this->count_no_object+1;
            //ROS_INFO("No Object on the hand");
        
	   //ROS_INFO("<------- END NO Found  Object --------->");
 
     }
}

void Weight_Estimator_Server::grasp_estimation_server(const grasp_estimator::GraspShape::ConstPtr &grasp_shape)
{
 grasp_shape_now = *grasp_shape;



}



void Weight_Estimator_Server::Hand_Control()
{
   
 if(run_state==true && mass_measured_now>0)
 {
  double alpha=alpha_;
  double alpha_2=alpha2_; 
  if(mass_measured_now>mass_threshold )
  { double joint_sinergy_error(msg_data_acquired_recent.hand_error_position);
    //if(fabs(joint_sinergy_error)<0.16) {joint_sinergy_error=0.0;}
    ROS_INFO("Current error sinergy position= %g",joint_sinergy_error); 
    double rep(joint_sinergy_error / (6*(1 + exp( -alpha_2 * joint_sinergy_error)))); 
    ROS_INFO("Repulsive sinergy joint %g",(double)rep);
    hand_des_state = 1 / (1 + exp( -alpha *  mass_measured_now)) - rep;
    if(hand_des_state<0){hand_des_state=0.0;} 
    ROS_INFO("Hand desidered state 0  %g",(double)hand_des_state);
    ROS_INFO("Estimating Object mass= %g",mass_measured_now);
    if(hand_des_state>1.0){hand_des_state=1.0;}
    if (hand_des_state > 0.70)
    {
         found_object=true;
         this->weight_estimate_now.hand_stopped=true;
         ROS_INFO("Hand desidered state 1  %g",(double)hand_des_state);
    }
  else
  {
    hand_des_state=0.0;
    found_object=false;
    this->weight_estimate_now.hand_stopped=false;
       ROS_INFO("Hand desidered state 2 %g",(double)hand_des_state);
  }
 }
 else

 {
  hand_des_state=0.0;
  found_object=false;
  this->weight_estimate_now.hand_stopped=false;
     ROS_INFO("Hand desidered state 3 %g",(double)hand_des_state);
 }  
}
trajectory_msgs::JointTrajectory hand_command_state;
hand_command_state.header.stamp = ros::Time::now();
// manca header frame id

hand_command_state.joint_names.push_back("left_hand_synergy_joint");
trajectory_msgs::JointTrajectoryPoint Point;
Point.positions.push_back(hand_des_state);
Point.time_from_start=ros::Duration(1.5);
hand_command_state.points.push_back(Point);

pub_hand_des_state_.publish(hand_command_state);
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
  // arm_action_move=true; 
  // arm_state_desidered=0;
  // Arm_Control();
  sleep(0.5);
  
  if (!ros::service::call(start_calibration_service_name,srv))
   {
     ROS_ERROR("Call to IMU calibration service start failed");
     exit(0);
    }
  else 
    {
     ROS_INFO("Call to IMU calibration service  DONE");
    }
    
  // arm_action_move=true; 
  // arm_state_desidered=0;
  // Arm_Control();
  sleep(0.5);
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
  sleep(0.5);

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
  ros::AsyncSpinner spinner(4);
  grasp_estimator::Weight_Estimator_Server Weight_Estimator_Server(nh);
  //sleep(2.0); 
  Weight_Estimator_Server.index = 0;
  Weight_Estimator_Server.index_calibration = 0;
  //Weight_Estimator_Server.calibration=false;
  // to test only cog directly
  Weight_Estimator_Server.calibration=true;
  
  Weight_Estimator_Server.hand_close_state=0;

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
  
 Weight_Estimator_Server.active_move=true;

  while (ros::ok())
	 {
        Weight_Estimator_Server.Hand_Control();
 
    Weight_Estimator_Server.publishDataState();
	 	loop_rate.sleep();
  }
  spinner.stop();
	return 0;
}

    
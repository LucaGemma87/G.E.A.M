

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
//#define N 25
#define left_hand_name "left_hand"
#define dual_hand_arm_name "dual_hand_arm"
#define Hz 50
#define N2 9
#define error_thresh 0.001
#define weight_threshold 2 // 2 Newton circa 200 gr
#define Hand_threshold 6.80
#define arm_state_num 5
#define N_cog 3


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
    

    grasp_estimator::DataAcquired msg_data_acquired_cog_array[N_cog];
    


public:
	// callback functions
      void weight_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now);
  // server function    
      void Hand_Calibration();
      void Object_Estimation();
      void publishDataState();
      void Hand_Close_State();
      void Hand_Control();
      void Arm_Control();
      void Arm_exp_position();
      void hand_cog_calibration();
      static int center_f(const gsl_vector *x, void *data, gsl_vector *f);

      int index;

      int index_calibration; 

      int hand_close_state;

      int arm_state;

      bool calibration;

      bool cog_calibration;

      bool active_move;

      bool arm_action_move;

      double weight_measured[N];
      double mass_measured[N];

      double weight_measured_now;
      double mass_measured_now;

      double hand_weight[N2-1];
      double hand_mass[N2-1]; 

      double weight_object;
      double mass_object;

      tf::Vector3 hand_gravity_center;
  
      moveit::planning_interface::MoveGroup *left_hand_ = new moveit::planning_interface::MoveGroup(left_hand_name); 
      moveit::planning_interface::MoveGroup *dual_hand_arm_ = new moveit::planning_interface::MoveGroup(dual_hand_arm_name);
      std::string hand_pose_name[N2-1];
      std::string dual_arm_pose_name[arm_state_num];
          

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
  

    //left_hand_ = new moveit::planning_interface::MoveGroup(left_hand_name);  
    
	}

	//! Empty stub
	~Weight_Estimator_Server() 
  {
   delete left_hand_;
  }

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
  
 if(this->index==N){this->index=0;}
  this->msg_data_acquired_recent=*msg_data_acquired_now;
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
       tf::Vector3 gravity(0,0,-9.81);
       tf::Vector3 gravity_ft_frame=ft_rotation_matrix.transpose()*gravity;
       force=ft_rotation_matrix.transpose()*force;
       this->mass_measured[this->index]=(sqrt(pow(force[0]/gravity_ft_frame[0],2)+pow(force[1]/gravity_ft_frame[1],2)+pow(force[2]/gravity_ft_frame[2],2)));
       this->weight_measured[this->index]=(sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)));
       this->mass_measured_now=this->mass_measured[this->index];
       this->weight_measured_now=this->weight_measured[this->index];
       //ROS_INFO("weight_measured now = %g",this->weight_measured_now); 
       //ROS_INFO("mass_measured now  = %g",this->mass_measured_now);
       this->index=this->index+1; 
       
}

// hand calibration works well with 25 data recorded
void Weight_Estimator_Server::Hand_Calibration()
{ 
  //ROS_INFO("Index_calibration %d",this->index_calibration);
  //ROS_INFO("Index_calibration %d",this->index_count);
  //ROS_INFO("Close state %d",this->hand_close_state);
  double weight_hand,mass_hand;
  if(this->active_move==false && this->index==N && this->index_calibration==this->hand_close_state && this->msg_data_acquired_recent.hand_error_position<=error_thresh)//&& this->msg_data_acquired_recent.hand_error_position<=error_thresh
  { 
   ROS_INFO("Estimating %d point !",this->index_calibration);
   for (int i = 0; i <N; ++i)
   {
    weight_hand=weight_hand+this->weight_measured[i];
    mass_hand=mass_hand+this->mass_measured[i];
   }
   weight_hand=weight_hand/N;
   mass_hand=mass_hand/N;
       ROS_INFO("weight_hand  = %g",(double)weight_hand);
       ROS_INFO("mass_hand  = %g",(double)mass_hand);
       this->hand_weight[this->index_calibration]=weight_hand;
       this->hand_mass[this->index_calibration]=mass_hand;
       ROS_INFO("Calibration %d point : DONE !\n",this->index_calibration);
       //this->index=0;
       this->index_calibration=this->index_calibration+1;
       this->active_move=true;  
   }
   else
   {
   //this->index=0;
   this->active_move=false; 
   }

  if(this->active_move==true){
  //ROS_INFO("Close state %d",this->hand_close_state);  
  switch(this->index_calibration) 
  {
    case 0 : 
    { 
      if(this->index==N)
      {
       //ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
          ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
          this->left_hand_->setNamedTarget("left_hand_home");
          // call the moves
          if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
          {
           ROS_ERROR("An error occured during Calibrating hand");
          } 
          else{ROS_INFO("Hand Moved to home position");
          this->active_move=false;
          this->hand_close_state=this->index_calibration;
          this->index=0;}
        } 
    }
     case 1 : 
     {
      if(this->index==N){
       ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
       this->left_hand_->setNamedTarget("left_hand_preshape");
        // call the moves
       
       if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
       {
         ROS_ERROR("An error occured during Calibrating hand");
       }
      else{ROS_INFO("Hand  moved to preshape position");this->active_move=false;this->hand_close_state=this->index_calibration;}
      }
     }
    case 2 :
    {
     if(this->index==N){ 
     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
     this->left_hand_->setNamedTarget("left_hand_25_close");
     // call the moves

     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
     {
       ROS_ERROR("An error occured during Calibrating hand");
     } 
     else{ROS_INFO("Hand  moved  to 25 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    } 
    case 3 :
    {
     if(this->index==N){
     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
      this->left_hand_->setNamedTarget("left_hand_56_close");
     // call the moves

      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
      {
        ROS_ERROR("An error occured during Calibrating hand");
      }
       else{ROS_INFO("Hand  moved  to 56 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    }
    case 4 :
    { if(this->index==N){
     ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position);  
     this->left_hand_->setNamedTarget("left_hand_70_close");
     // call the moves

     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
     {
       ROS_ERROR("An error occured during Calibrating hand");
     } 
      else{ROS_INFO("Hand  moved  to 70 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    }
    case 5 :
    { 
      if(this->index==N){
        ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
     this->left_hand_->setNamedTarget("left_hand_80_close");
     // call the moves

     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
     {
       ROS_ERROR("An error occured during Calibrating hand");
     }
      else{ROS_INFO("Hand  moved  to 80 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    } 
    case 6 : 
    { 
     if(this->index==N){ 
      ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
     this->left_hand_->setNamedTarget("left_hand_90_close");
     // call the moves

     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)   )
     {
       ROS_ERROR("An error occured during Calibrating hand");
     }
      else{ROS_INFO("Hand  moved  to 90 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    }
    case 7 : 
    { 
      if(this->index==N){
        ROS_INFO("Hand Synergy Joint Position = %g",(double)this->msg_data_acquired_recent.hand_synergy_joint_state_position); 
     this->left_hand_->setNamedTarget("left_hand_99_close");
     // call the moves

     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
     {
       ROS_ERROR("An error occured during Calibrating hand");
     }
      else{ROS_INFO("Hand  moved  to 99 close position");this->active_move=false;this->hand_close_state=this->index_calibration;}
     }
    } 
    
    case 8 : 
    { 
     if(this->index==N){
      this->left_hand_->setNamedTarget("left_hand_home");
      // call the moves

      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)  )
      {
        ROS_ERROR("An error occured during Calibrating hand");
      }
      else{ROS_INFO("Hand Moved to Home position");this->active_move=false;}
    
      ROS_INFO("Calibration DONE %g, %g, %g, %g, %g, %g, %g, %g",this->hand_weight[0],this->hand_weight[1],this->hand_weight[2],
        this->hand_weight[3],this->hand_weight[4],this->hand_weight[5],this->hand_weight[6],this->hand_weight[7]);
      this->calibration=true;
      this->active_move=true;
      this->arm_action_move=true;
      this->hand_close_state=0;
     }    
    }  
   } 
  }
}

void Weight_Estimator_Server::Hand_Control()
{
 int i; 
  
  if(this->active_move==true&&this->weight_object>0.1)
  { 
    i=this->hand_close_state+1; 
    if(i==N2-2){ROS_INFO("Hand is in 90 Close Position: No need planning");i=N2-3;}
    else
    {  
     this->left_hand_->setNamedTarget(this->hand_pose_name[i]);
     // call the moves
     if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) && fabs(this->msg_data_acquired_recent.hand_error_position)<error_thresh)
     {
       ROS_ERROR("An error occured during Moving hand");
      }
      else{ROS_INFO("Hand moved to: ");ROS_INFO_STREAM(this->hand_pose_name[i];);
         this->index=0;this->active_move=false;this->hand_close_state=i;}
     }
  }
   if(this->active_move==true&&this->weight_object<0.1)
   { 
     i=this->hand_close_state-1;
     if(i==-1){ROS_INFO("Hand is in Home position. No need for planning");this->hand_close_state=0;}
     else
     {
      this->left_hand_->setNamedTarget(this->hand_pose_name[i]);
      // call the moves
      if( !(this->left_hand_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS)&& fabs(this->msg_data_acquired_recent.hand_error_position)<error_thresh)
      {
       ROS_ERROR("An error occured during Moving hand");
      }
      else{ROS_INFO("Hand moved to: ");ROS_INFO_STREAM(this->hand_pose_name[i]);this->index=0;this->active_move=false;this->hand_close_state=i;}
   }
  }
 
}

void Weight_Estimator_Server::Arm_exp_position()
{  
  this->dual_hand_arm_->setNamedTarget(this->dual_arm_pose_name[0]);
      // call the moves
 if( !(this->dual_hand_arm_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
  {
  ROS_ERROR("An error occured during Moving Arm");
  }
  else
  {
   ROS_INFO("Moved Dual arm to Experimental Position !");  
   this->arm_state=0;
   this->index=0;
  }  
}

void Weight_Estimator_Server::Arm_Control()
{
  if(this->arm_action_move==true){
   ROS_INFO("Arm state %d",this->arm_state);
   int i(this->arm_state+1); 
   if(i>arm_state_num){i=0;}
   this->dual_hand_arm_->setNamedTarget(this->dual_arm_pose_name[i]);
      // call the moves

   if( !(this->dual_hand_arm_->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
   {
   ROS_ERROR("An error occured during Moving Arm");
   }
   else
   {
     ROS_INFO("Dual arm moved to !");
     ROS_INFO_STREAM(this->dual_arm_pose_name[i]);  
     this->arm_state=i;
     this->arm_action_move==true;
    }    
  }

  // if(this->hand_close_state==0&&this->arm_action_move==true){
  //  this->Arm_exp_position(); 
  //}

}


// void Weight_Estimator_Server::Hand_Close_State()
// {
//  int hand_state(this->hand_close_state);
//  //ROS_INFO("Hand hand_synergy_joint_state_position = %g" ,this->msg_data_acquired_recent.hand_synergy_joint_state_position);
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.1409) {hand_state=0;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.15&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.2499) {hand_state=1;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.24991&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.5599) {hand_state=2;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.55991&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.6999) {hand_state=3;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.69991&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.7999) {hand_state=4;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.79991&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.8999) {hand_state=5;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.89991&&this->msg_data_acquired_recent.hand_synergy_joint_state_position<0.9899) {hand_state=6;}
//  if(this->msg_data_acquired_recent.hand_synergy_joint_state_position>=0.99){hand_state=7;} 
//  this->hand_close_state=hand_state;
//  //this->active_move=true;
// }


void Weight_Estimator_Server::Object_Estimation()
{
   //if(this->msg_data_acquired_recent.hand_synergy_joint_state_position)
   
   double diff_weight(this->weight_measured_now-hand_weight[this->hand_close_state]);
   //double diff_weight(this->weight_measured_now-Hand_threshold);
   ROS_INFO("Diff Weight= %g",diff_weight);
   if(diff_weight>weight_threshold)
   {
    ROS_INFO("Object Detected on the hand");
    this->weight_object=fabs(diff_weight);
    ROS_INFO("Estimating Object Weight= %g",this->weight_object);
    this->mass_object=this->mass_measured_now-hand_mass[this->hand_close_state];
    //this->mass_object=this->mass_measured_now-Hand_threshold/9.81;
    ROS_INFO("Estimating Object mass= %g",this->mass_object);
    this->active_move=true;
   }
   else
   {
    this->weight_object=0;
    this->mass_object=0;
    ROS_INFO("No Object on the hand");
    this->active_move=true;
    if(this->arm_state!=0)
    {
      this->Arm_exp_position();
    }
   }
}

void Weight_Estimator_Server::hand_cog_calibration()
{ 
  if(this->arm_state==0&&this->index==N-1)
  {
   ROS_INFO("Measuring %d for estimating cog",this->arm_state);
   this->msg_data_acquired_cog_array[this->arm_state]=this->msg_data_acquired_recent;
  }
  if(this->arm_state==1&&this->index==N-1)
  {
   ROS_INFO("Measuring %d for estimating cog",this->arm_state);
   this->msg_data_acquired_cog_array[this->arm_state]=this->msg_data_acquired_recent;
  }
  if(this->arm_state==2&&this->index==N-1)
  {
   ROS_INFO("Measuring %d for estimating cog",this->arm_state);
   this->msg_data_acquired_cog_array[this->arm_state]=this->msg_data_acquired_recent;
  }
  if(this->arm_state==3&&this->index==N-1)
  {
   ROS_INFO("Measuring %d for estimating cog",this->arm_state); 
   this->msg_data_acquired_cog_array[this->arm_state]=this->msg_data_acquired_recent;
  } 
   //Done something
   if(this->arm_state==4&&this->index==N-1)
  {
   ROS_INFO("Measuring %d for estimating cog",this->arm_state); 
   this->msg_data_acquired_cog_array[this->arm_state]=this->msg_data_acquired_recent;

   ROS_INFO("Hand Cog Estimated ! ! !");
   this->cog_calibration=true;
  }
  
 
 
}


void Weight_Estimator_Server::publishDataState()
{ 
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
  //ros::Rate loop_rate(Hz);
  ros::AsyncSpinner spinner(4);
  grasp_estimator::Weight_Estimator_Server Weight_Estimator_Server(nh);
  //sleep(2.0); 
  Weight_Estimator_Server.index = 0;
  Weight_Estimator_Server.index_calibration = 0;
  Weight_Estimator_Server.calibration=false;
  Weight_Estimator_Server.active_move=true;
  Weight_Estimator_Server.hand_close_state=0;
  Weight_Estimator_Server.hand_pose_name[0]="left_hand_home";
  Weight_Estimator_Server.hand_pose_name[1]="left_hand_preshape";
  Weight_Estimator_Server.hand_pose_name[2]="left_hand_25_close";
  Weight_Estimator_Server.hand_pose_name[3]="left_hand_56_close";
  Weight_Estimator_Server.hand_pose_name[4]="left_hand_70_close";
  Weight_Estimator_Server.hand_pose_name[5]="left_hand_80_close";
  Weight_Estimator_Server.hand_pose_name[6]="left_hand_90_close";
  Weight_Estimator_Server.hand_pose_name[7]="left_hand_99_close";
  Weight_Estimator_Server.dual_arm_pose_name[0]="dual_hand_arm_exp";
  Weight_Estimator_Server.dual_arm_pose_name[1]="dual_hand_arm_exp_1";
  Weight_Estimator_Server.dual_arm_pose_name[2]="dual_hand_arm_exp_2";
  Weight_Estimator_Server.dual_arm_pose_name[3]="dual_hand_arm_exp_3";
  Weight_Estimator_Server.dual_arm_pose_name[4]="dual_hand_arm_exp_4";
  Weight_Estimator_Server.arm_action_move=false;
  
	
  
  spinner.start();
  Weight_Estimator_Server.Arm_exp_position();
  
  sleep(2.0); 

  
  while (ros::ok())
	 {
    //spinner.start(); 
       //Weight_Estimator_Server.Hand_Close_State();
      if(Weight_Estimator_Server.calibration==false) // || Weight_Estimator_Server.cog_calibration==false
      {
     
       Weight_Estimator_Server.Hand_Calibration();
      // if(Weight_Estimator_Server.calibration==true&&Weight_Estimator_Server.cog_calibration==false) {
      //   Weight_Estimator_Server.Arm_Control();
      //   Weight_Estimator_Server.hand_cog_calibration();
      // }
    } 
    else
    { 
     Weight_Estimator_Server.Object_Estimation(); 
     Weight_Estimator_Server.Hand_Control();
     //Weight_Estimator_Server.Arm_Control();
    

    }
     Weight_Estimator_Server.publishDataState();
	 	//loop_rate.sleep();
	}
  
  spinner.stop();
  Weight_Estimator_Server.calibration=false;
	return 0;
}

    
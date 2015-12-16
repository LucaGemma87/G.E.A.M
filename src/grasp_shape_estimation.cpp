

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
#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_multiset.h>

// marker
#include <visualization_msgs/Marker.h>


//#include <gsl/gsl_multifit.h>

/* number of data points to fit */
#define N 19
#define pi 3.14

namespace grasp_estimator
{

class Grasp_Shape_Estimator_Server
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
	ros::Publisher pub_shape_grasp_states_;

	ros::Publisher pub_hand_link_position_;

  // marker pubblisher
  
  ros::Publisher marker_pub_;

    // subscribers
    ros::Subscriber msg_grasp_state_now_;
   
    //msgs definition
	grasp_estimator::DataAcquired msg_data_acquired_;
    
  grasp_estimator::GraspShape grasp_shape_estimate_now;
 

public:
	// callback functions
      void grasp_shape_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now);
      void publishDataState();
      //static int expb_f(const gsl_vector * x, void *data, gsl_vector * f);
      static int sphere_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      static int cylinder_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      //static int expb_df(const gsl_vector * x, void *data,gsl_matrix * J);
	// constructor
	Grasp_Shape_Estimator_Server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
        
        msg_grasp_state_now_ = nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),5,
                                                                                     &Grasp_Shape_Estimator_Server::grasp_shape_estimation_server,
                                                                                     this);
        
		// advertise topics
		pub_shape_grasp_states_ = nh_.advertise<grasp_estimator::GraspShape>(nh_.resolveName("/left_hand/grasp_shape_estimator_server_"), 5);
	
		pub_hand_link_position_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/hand_link_server"), 19);
	  
    marker_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName( "/markers_out"), 1);
  }

	//! Empty stub
	~Grasp_Shape_Estimator_Server() {}

};


/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GSL ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// struct data {
//  size_t n;
//  double * y;
// };

struct data_3d {
 //size_t n_x,n_y,n_z;
 size_t n_theta;
 double  min_x;
 double  min_y;
 double  min_z;
 double  max_x;
 double  max_y;
 double  max_z;
 double * x;
 double * y;
 double * z;
};

      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// SPHERE FITTING /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sphere (x-x_O)^2+(y-y_O)^2+(z-z_O)^2=r^2

int Grasp_Shape_Estimator_Server::sphere_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta;
 
  // double x_min = ((struct data_3d *)data_3d)->min_x;
  // double y_min = ((struct data_3d *)data_3d)->min_y;
  // double z_min = ((struct data_3d *)data_3d)->min_z;

  // double x_max = ((struct data_3d *)data_3d)->max_x;
  // double y_max = ((struct data_3d *)data_3d)->max_y;
  // double z_max = ((struct data_3d *)data_3d)->max_z;

  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  
  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r = gsl_vector_get(x, 3);

   size_t i,j;

   //ROS_INFO("Initializing DONE");  

   for (i = 0; i < n_theta; i++)
     {
       //for (j = 0; i < n_phi; j++)
        //{
          
         double theta_i = 2*pi*i/N;
         double phi_i   = 2*pi*i/N;
        
         double xx = x_0 + r*sin(theta_i)*cos(phi_i);
         double yy = y_0 + r*sin(theta_i)*sin(phi_i);
         double zz = z_0 + r*cos(theta_i);
         
         //double Y = pow((xx-x_0),2)+pow((yy-y_0),2)+pow((zz-z_0),2)-pow(r,2);
         //ROS_INFO("Yijk %f",(float)Y);
         double Y = pow((xx-x_0),2)+pow((yy-y_0),2)+pow((zz-z_0),2);
          double y = (pow((x_data[i]-x_0),2)+pow((y_data[i]-y_0),2)+pow((z_data[i]-z_0),2));
         
         //ROS_INFO("y %f",(float)y);
         gsl_vector_set(f,i, Y -y);

         //double y = (pow((x_data[i]-x_0),2)+pow((y_data[i]-y_0),2)+pow((z_data[i]-z_0),2)-pow(r,2));
         //gsl_vector_set(f,i, y);
         //ROS_INFO("Iter: %f ",(float)(i*(j+1)*(k+1)));
       
        //}       
      }

   return GSL_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CYLINDER FITTING  ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Grasp_Shape_Estimator_Server::cylinder_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta;
  
 
  double x_min = ((struct data_3d *)data_3d)->min_x;
  double y_min = ((struct data_3d *)data_3d)->min_y;
  double z_min = ((struct data_3d *)data_3d)->min_z;

  double x_max = ((struct data_3d *)data_3d)->max_x;
  double y_max = ((struct data_3d *)data_3d)->max_y;
  double z_max = ((struct data_3d *)data_3d)->max_z;
  
  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  
  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r = gsl_vector_get(x, 3);
  //double h = gsl_vector_get(x, 4);
  
  double height = fabs(x_max-x_min);
  // double distance[3];
  // distance[0] = fabs(x_max-x_min);
  // distance[1] = fabs(y_max-y_min);
  // distance[2] = fabs(z_max-z_min);
  // height = distance[0];
  // for(int l = 1; l < 2; ++l)
  // {
  //  if((double)distance[l] > height){ height = (double)distance[l]; }
  // }

   size_t i;

   //ROS_INFO("Initializing DONE");  

   for (i = 0; i < n_theta; i++)
      { 
         double theta_i = 2*pi*i/N;
         double y2 = -z_0 + height*i/N;
         double xx = x_0 + r*cos(theta_i);
         double zz = z_0 + r*sin(theta_i);
         double yy = y_0 + y2;
         
         //double Y = pow((xx-x_0),2)+pow((zz-z_0),2)-pow(r,2);
         //ROS_INFO("Y %f",(float)Y); +pow(zz-h,2) +pow(yy-y_0,2)
         double Y = pow((xx-x_0),2)+pow((zz-z_0),2);
         double y = (pow((x_data[i]-x_0),2)+pow((z_data[i]-z_0),2));
         //ROS_INFO("y %f",(float)y); +pow(z_data[i]-z_0,2) +pow(y_data[i]-y_0,2)
         gsl_vector_set(f,i, Y-y);
         //double y = (pow((x_data[i]-x_0),2)+pow((z_data[i]-z_0),2)-pow(r,2));
         //gsl_vector_set(f,i, y);
         //ROS_INFO("Iter: %f ",(float)(i*(j+1)*(k+1)));      
       }

   return GSL_SUCCESS;
}

 


///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// End of GSL /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void Grasp_Shape_Estimator_Server::grasp_shape_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_now)
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
  ROS_INFO("Grasp Shape Estimator START");

  ROS_INFO("Grasp Shape Estimator GSl initialization");
  
  const gsl_multifit_fdfsolver_type *T = gsl_multifit_fdfsolver_lmsder;
  const gsl_multifit_fdfsolver_type *T_cyl = gsl_multifit_fdfsolver_lmsder;
  gsl_multifit_fdfsolver *s,*s_cyl;
  int status, info,status_cyl,info_cyl;
  size_t i;
  const size_t n = N;
  const size_t n_theta=N;  
  const size_t p = 4;
  const size_t p_cyl = 5;
  // init sphere
  gsl_matrix *J = gsl_matrix_alloc(n, p);
  gsl_matrix *covar = gsl_matrix_alloc (p, p);
  // init cylinder
  gsl_matrix *J_cyl = gsl_matrix_alloc(n, p_cyl);
  gsl_matrix *covar_cyl = gsl_matrix_alloc (p_cyl, p_cyl);


  double x_data[N],y_data[N],z_data[N], weights[N];
  double x_min,x_max,y_min,y_max,z_min,z_max;
  
  struct data_3d d_3d = { n_theta,x_min,x_max,y_min,y_max,z_min,z_max,x_data,y_data,z_data};
  gsl_multifit_function_fdf f,f_cyl;
  
  ROS_INFO("Grasp Shape Estimator receiving message");
  
  // Initializing variables	
  tf::StampedTransform hand_index_distal_link_stamped,    hand_index_knuckle_link_stamped,   hand_index_middle_link_stamped,     hand_index_proximal_link_stamped;
  tf::StampedTransform hand_little_distal_link_stamped,   hand_little_knuckle_link_stamped,  hand_little_middle_link_stamped,    hand_little_proximal_link_stamped;  
  tf::StampedTransform hand_middle_distal_link_stamped,   hand_middle_knuckle_link_stamped,  hand_middle_middle_link_stamped,    hand_middle_proximal_link_stamped; 
  tf::StampedTransform hand_ring_distal_link_stamped,     hand_ring_knuckle_link_stamped,    hand_ring_middle_link_stamped,      hand_ring_proximal_link_stamped;
  tf::StampedTransform hand_thumb_distal_link_stamped,    hand_thumb_knuckle_link_stamped,   hand_thumb_proximal_link_stamped;
  
  // Position msgs
  geometry_msgs::PointStamped position_msgs;

  // trasform msgs
  // index
  transformStampedMsgToTF(msg_data_acquired_now->hand_index_distal_link, hand_index_distal_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_index_knuckle_link,hand_index_knuckle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_index_middle_link,hand_index_middle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_index_proximal_link,hand_index_proximal_link_stamped);

  //little
  transformStampedMsgToTF(msg_data_acquired_now->hand_little_distal_link,hand_little_distal_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_little_knuckle_link,hand_little_knuckle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_little_middle_link,hand_little_middle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_little_proximal_link,hand_little_proximal_link_stamped);

  //middle
  transformStampedMsgToTF(msg_data_acquired_now->hand_middle_distal_link,hand_middle_distal_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_middle_knuckle_link,hand_middle_knuckle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_middle_middle_link,hand_middle_middle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_middle_proximal_link,hand_middle_proximal_link_stamped);

  //ring
  transformStampedMsgToTF(msg_data_acquired_now->hand_ring_distal_link, hand_ring_distal_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_ring_knuckle_link,hand_ring_knuckle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_ring_middle_link,hand_ring_middle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_ring_proximal_link,hand_ring_proximal_link_stamped);

  //thumb 
  transformStampedMsgToTF(msg_data_acquired_now->hand_thumb_distal_link,hand_thumb_distal_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_thumb_knuckle_link,hand_thumb_knuckle_link_stamped);
  transformStampedMsgToTF(msg_data_acquired_now->hand_thumb_proximal_link,hand_thumb_proximal_link_stamped);


  // get origin  
  // index  
  tf::Vector3 pos_id = hand_index_distal_link_stamped.getOrigin();
  tf::Vector3 pos_ik = hand_index_knuckle_link_stamped.getOrigin();
  tf::Vector3 pos_im = hand_index_middle_link_stamped.getOrigin();
  tf::Vector3 pos_ip = hand_index_proximal_link_stamped.getOrigin();
  

  x_data[0]=pos_id[0];
  x_data[1]=pos_ik[0];
  x_data[2]=pos_im[0];
  x_data[3]=pos_ip[0];
  
  y_data[0]=pos_id[1];
  y_data[1]=pos_ik[1];
  y_data[2]=pos_im[1];
  y_data[3]=pos_ip[1];
  
  z_data[0]=pos_id[2];
  z_data[1]=pos_ik[2];
  z_data[2]=pos_im[2];
  z_data[3]=pos_ip[2];
  
  // {
  //   // position_msgs[0].point.x=pos_id[0];
  //   // position_msgs[0].point.y=pos_id[1];
  //   // position_msgs[0].point.z=pos_id[2];
    
  //   // position_msgs[1].point.x=pos_ik[0];
  //   // position_msgs[1].point.y=pos_ik[1];
  //   // position_msgs[1].point.z=pos_ik[2];
    
  //   // position_msgs[2].point.x=pos_im[0];
  //   // position_msgs[2].point.y=pos_im[1];
  //   // position_msgs[2].point.z=pos_im[2];

  //   // position_msgs[3].point.x=pos_ip[0];
  //   // position_msgs[3].point.y=pos_ip[1];
  //   // position_msgs[3].point.z=pos_ip[2];
  // }
  
  //little ,   ,  ,    
  tf::Vector3 pos_ld = hand_little_distal_link_stamped.getOrigin();
  tf::Vector3 pos_lk = hand_little_knuckle_link_stamped.getOrigin();
  tf::Vector3 pos_lm = hand_little_middle_link_stamped.getOrigin();
  tf::Vector3 pos_lp = hand_little_proximal_link_stamped.getOrigin();

  x_data[4]=pos_ld[0];
  x_data[5]=pos_lk[0];
  x_data[6]=pos_lm[0];
  x_data[7]=pos_lp[0];
  
  y_data[4]=pos_ld[1];
  y_data[5]=pos_lk[1];
  y_data[6]=pos_lm[1];
  y_data[7]=pos_lp[1];
  
  z_data[4]=pos_ld[2];
  z_data[5]=pos_lk[2];
  z_data[6]=pos_lm[2];
  z_data[7]=pos_lp[2];
  
  // {
  //   // position_msgs[4].point.x=pos_ld[0];
  //   // position_msgs[4].point.y=pos_ld[1];
  //   // position_msgs[4].point.z=pos_ld[2];
    
  //   // position_msgs[5].point.x=pos_lk[0];
  //   // position_msgs[5].point.y=pos_lk[1];
  //   // position_msgs[5].point.z=pos_lk[2];
    
  //   // position_msgs[6].point.x=pos_lm[0];
  //   // position_msgs[6].point.y=pos_lm[1];
  //   // position_msgs[6].point.z=pos_lm[2];

  //   // position_msgs[7].point.x=pos_lp[0];
  //   // position_msgs[7].point.y=pos_lp[1];
  //   // position_msgs[7].point.z=pos_lp[2];
  // }
  
  //middle ,   ,  ,    ;
  tf::Vector3 pos_md = hand_middle_distal_link_stamped.getOrigin();
  tf::Vector3 pos_mk = hand_middle_knuckle_link_stamped.getOrigin();
  tf::Vector3 pos_mm = hand_middle_middle_link_stamped.getOrigin();
  tf::Vector3 pos_mp = hand_middle_proximal_link_stamped.getOrigin();

  x_data[8]=pos_md[0];
  x_data[9]=pos_mk[0];
  x_data[10]=pos_mm[0];
  x_data[11]=pos_mp[0];
  
  y_data[8]=pos_md[1];
  y_data[9]=pos_mk[1];
  y_data[10]=pos_mm[1];
  y_data[11]=pos_mp[1];
  
  z_data[8]=pos_md[2];
  z_data[9]=pos_mk[2];
  z_data[10]=pos_mm[2];
  z_data[11]=pos_mp[2];
  

  // { 
  //   // position_msgs[8].point.x=pos_md[0];
  //   // position_msgs[8].point.y=pos_md[1];
  //   // position_msgs[8].point.z=pos_md[2];
    
  //   // position_msgs[9].point.x=pos_mk[0];
  //   // position_msgs[9].point.y=pos_mk[1];
  //   // position_msgs[9].point.z=pos_mk[2];
    
  //   // position_msgs[10].point.x=pos_mm[0];
  //   // position_msgs[10].point.y=pos_mm[1];
  //   // position_msgs[10].point.z=pos_mm[2];

  //   // position_msgs[11].point.x=pos_mp[0];
  //   // position_msgs[11].point.y=pos_mp[1];
  //   // position_msgs[11].point.z=pos_mp[2];
  // }
  

  //ring  
  tf::Vector3 pos_rd = hand_ring_distal_link_stamped.getOrigin();
  tf::Vector3 pos_rk = hand_ring_knuckle_link_stamped.getOrigin();
  tf::Vector3 pos_rm = hand_ring_middle_link_stamped.getOrigin();
  tf::Vector3 pos_rp = hand_ring_proximal_link_stamped.getOrigin();

  x_data[12]=pos_rd[0];
  x_data[13]=pos_rk[0];
  x_data[14]=pos_rm[0];
  x_data[15]=pos_rp[0];
  
  y_data[12]=pos_rd[1];
  y_data[13]=pos_rk[1];
  y_data[14]=pos_rm[1];
  y_data[15]=pos_rp[1];
  
  z_data[12]=pos_rd[2];
  z_data[13]=pos_rk[2];
  z_data[14]=pos_rm[2];
  z_data[15]=pos_rp[2];
  

  // {
  //  // position_msgs[12].point.x=pos_rd[0];
  //  // position_msgs[12].point.y=pos_rd[1];
  //  // position_msgs[12].point.z=pos_rd[2];
   
  //  // position_msgs[13].point.x=pos_rk[0];
  //  // position_msgs[13].point.y=pos_rk[1];
  //  // position_msgs[13].point.z=pos_rk[2];
   
  //  // position_msgs[14].point.x=pos_rm[0];
  //  // position_msgs[14].point.y=pos_rm[1];
  //  // position_msgs[14].point.z=pos_rm[2];

  //  // position_msgs[15].point.x=pos_rp[0];
  //  // position_msgs[15].point.y=pos_rp[1];
  //  // position_msgs[15].point.z=pos_rp[2];
  // }


  //thumb 
  tf::Vector3 pos_td = hand_thumb_distal_link_stamped.getOrigin();
  tf::Vector3 pos_tk = hand_thumb_knuckle_link_stamped.getOrigin();
  tf::Vector3 pos_tp = hand_thumb_proximal_link_stamped.getOrigin();

  x_data[16]=pos_td[0];
  x_data[17]=pos_tk[0];
  x_data[18]=pos_tp[0];
 
  
  y_data[16]=pos_td[1];
  y_data[17]=pos_tk[1];
  y_data[18]=pos_tp[1];
 
  z_data[16]=pos_td[2];
  z_data[17]=pos_tk[2];
  z_data[18]=pos_tp[2];
  
  
  x_min=x_data[0];
  x_max=x_data[0];
  y_min=y_data[0];
  y_max=y_data[0];
  z_min=z_data[0];
  z_max=z_data[0];

  double center_x = x_data[0];
  double center_y = y_data[0];
  double center_z = z_data[0];
  for(int i = 1; i < N; ++i)
  {
   if((double)x_data[i] > x_max){ x_max = (double)x_data[i]; }
   if((double)x_data[i] < x_min){ x_min = (double)x_data[i]; }
  
   if((double)y_data[i] > y_max){ y_max = (double)y_data[i]; }
   if((double)y_data[i] < y_min){ y_min = (double)y_data[i]; }
  
   if((double)z_data[i] > z_max){ z_max = (double)z_data[i]; }
   if((double)z_data[i] < z_min){ z_min = (double)z_data[i]; }
  
   center_x += x_data[i];
   center_y += y_data[i];
   center_z += z_data[i];  
  }

  center_x=center_x/N;
  center_y=center_y/N;
  center_z=center_z/N;
  double height= fabs(y_max-y_min);

  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  //ROS_INFO(" Center_x: %f, center_y: %f, center_z: %f",(float)center_x,(float)center_y,(float)center_z);
  // {
  //  // position_msgs[16].point.x=pos_td[0];
  //  // position_msgs[16].point.y=pos_td[1];
  //  // position_msgs[16].point.z=pos_td[2];
  
  //  // position_msgs[17].point.x=pos_tk[0];
  //  // position_msgs[17].point.y=pos_tk[1];
  //  // position_msgs[17].point.z=pos_tk[2];
  
  //  // position_msgs[18].point.x=pos_tp[0];
  //  // position_msgs[18].point.y=pos_tp[1];
  //  // position_msgs[18].point.z=pos_tp[2];
  // } 
  // {
  //  // for( int i = 0; i < 19; i++ )
  //  //  {
  //  //      position_msgs[i].header.seq=i;
  //  //      position_msgs[i].header.stamp=ros::Time::now();
  //  //      position_msgs[i].header.frame_id="left_arm_7_link";
  //  //      pub_hand_link_position_.publish(position_msgs[i]);
  //  //  }
  // } 
   
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL continue SPHERE  ////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ROS_INFO("Fitting Sphere");
  double x_init[4] = { center_x, center_y, center_z, 0.05};
  gsl_vector_view x = gsl_vector_view_array(x_init, p);
  gsl_vector_view w = gsl_vector_view_array(weights, n);
  const gsl_rng_type * type;
  gsl_rng * r;
  gsl_vector *res_f;
  double chi, chi0;

  const double xtol = 1e-8;
  const double gtol = 1e-8;
  const double ftol = 0.0;

  gsl_rng_env_setup();

  type = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f.f =&sphere_f;

   f.df =NULL;
   //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
   f.n = n;
   f.p = p;
   f.params = &d_3d;

  /* This is the data to be fitted */
  ROS_INFO("Creating data");
  for (i = 0; i < n; i++)
    {
      weights[i] = 1.0 / (0.5 * 0.5);
    }; 
  


  //ROS_INFO("Data created");
  s = gsl_multifit_fdfsolver_alloc(T, n, p);
  //ROS_INFO("Alloc Done");
  /* initialize solver with starting point and weights */
  gsl_multifit_fdfsolver_wset(s, &f, &x.vector, &w.vector);
  //ROS_INFO("wset Done"); 
   /* compute initial residual norm */
  res_f = gsl_multifit_fdfsolver_residual(s);
  //ROS_INFO("gsl_multifit_fdfsolver_residual Done"); 
  chi0 = gsl_blas_dnrm2(res_f);

  /* solve the system with a maximum of 20 iterations */
  status = gsl_multifit_fdfsolver_driver(s, 40, xtol, gtol, ftol, &info);

  gsl_multifit_fdfsolver_jac(s, J);
  gsl_multifit_covar(J, 0.0, covar);

  /* compute final residual norm */
  chi = gsl_blas_dnrm2(res_f);

  #define FIT(i) gsl_vector_get(s->x, i)
  #define ERR(i) sqrt(gsl_matrix_get(covar,i,i))

  // ROS_INFO("summary from method '%s'\n",gsl_multifit_fdfsolver_name(s));
  // ROS_INFO( "number of iterations: %zu\n",
  //         gsl_multifit_fdfsolver_niter(s));
  // ROS_INFO( "function evaluations: %zu\n", f.nevalf);
  // ROS_INFO( "Jacobian evaluations: %zu\n", f.nevaldf);
  // ROS_INFO( "reason for stopping: %s\n",
  //         (info == 1) ? "small step size" : "small gradient");
  // ROS_INFO( "initial |f(x)| = %g\n", chi0);
  // ROS_INFO( "final   |f(x)| = %g\n", chi);
  // ROS_INFO( "ratio   = %g\n", chi0/chi);
  double ratio_inv=  chi/chi0;
  //ROS_INFO( "ratio_inv   = %g\n", chi/chi0);
  

  
  double dof = n - p;
  double c = GSL_MAX_DBL(1, chi / sqrt(dof)); 
  //ROS_INFO( "c = %g\n", c);
  //ROS_INFO("chi / sqrt(dof) %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  // ROS_INFO( "chisq/dof = %g\n",  pow(chi, 2.0) / dof);

   ROS_INFO ( "x_0      = %.5f +/- %.5f\n", FIT(0), c*ERR(0));
   ROS_INFO ( "y_0 = %.5f +/- %.5f\n", FIT(1), c*ERR(1));
   ROS_INFO ( "z_0      = %.5f +/- %.5f\n", FIT(2), c*ERR(2));
   ROS_INFO ( "r      = %.5f +/- %.5f\n", FIT(3), c*ERR(3));

  // ROS_INFO ( "status = %s\n", gsl_strerror (status));
  
  position_msgs.point.x=FIT(0);
  position_msgs.point.y=FIT(1);
  position_msgs.point.z=FIT(2);
  position_msgs.header.seq=1;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  
   
  // marker visualization
  // creating marker 
  visualization_msgs::Marker marker;
  marker.header.frame_id = "left_arm_7_link";
  marker.header.stamp = ros::Time();
  marker.ns = "Grasp_Shape_Estimator_Server";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = FIT(0);
  marker.pose.position.y = FIT(1);
  marker.pose.position.z = FIT(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2*fabs(FIT(3));
  marker.scale.y = 2*fabs(FIT(3));
  marker.scale.z = 2*fabs(FIT(3));
  marker.color.a = 0.2; // Don't forget to set the alpha!
  marker.color.r = (float)ratio_inv;
  marker.color.g = 1.0f-(float)ratio_inv;
  marker.color.b = 0.0f;
  marker.lifetime = ros::Duration(0.2);
  // publishing marker
  double r_sphere = FIT(3);
  if (r_sphere>0.04 && r_sphere<0.089 && (1.0 - ratio_inv) > 0.75)
  {  
  marker_pub_.publish( marker );
  pub_hand_link_position_.publish(position_msgs);
  }
  ROS_INFO("Fitting sphere DONE");

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL continue Cylinder  ////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  ROS_INFO("Fitting Cylinder");
  double x_init_cyl[5] = { center_x, center_y, center_z, 0.05,height};
  gsl_vector_view x_cyl = gsl_vector_view_array(x_init_cyl, p_cyl);
  gsl_vector_view w_cyl = gsl_vector_view_array(weights, n);
  const gsl_rng_type *type_cyl;
  gsl_rng *r_cyl;
  gsl_vector *res_f_cyl;
  double chi_cyl, chi0_cyl;

  gsl_rng_env_setup();

  type_cyl = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f_cyl.f =&cylinder_f;

   f_cyl.df =NULL;
   //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
   f_cyl.n = n;
   f_cyl.p = p_cyl;
   f_cyl.params = &d_3d;

  /* This is the data to be fitted */
  ROS_INFO("Creating data");
  for (i = 0; i < n; i++)
    {
      weights[i] = 1.0 / (0.5 * 0.5);
    }; 
  


  //ROS_INFO("Data created");
  s_cyl = gsl_multifit_fdfsolver_alloc(T_cyl, n, p_cyl);
  //ROS_INFO("Alloc Done");
  /* initialize solver with starting point and weights */
  gsl_multifit_fdfsolver_wset(s_cyl, &f_cyl, &x_cyl.vector, &w_cyl.vector);
  //ROS_INFO("wset Done"); 
   /* compute initial residual norm */
  res_f_cyl = gsl_multifit_fdfsolver_residual(s_cyl);
  //ROS_INFO("gsl_multifit_fdfsolver_residual Done"); 
  chi0_cyl = gsl_blas_dnrm2(res_f_cyl);

  /* solve the system with a maximum of 20 iterations */
  status_cyl = gsl_multifit_fdfsolver_driver(s_cyl, 40, xtol, gtol, ftol, &info_cyl);

  gsl_multifit_fdfsolver_jac(s_cyl, J_cyl);
  gsl_multifit_covar(J_cyl, 0.0, covar_cyl);

  /* compute final residual norm */
  chi = gsl_blas_dnrm2(res_f_cyl);

  #define FIT_cyl(i) gsl_vector_get(s_cyl->x, i)
  #define ERR_cyl(i) sqrt(gsl_matrix_get(covar_cyl,i,i))

  // ROS_INFO("summary from method '%s'\n",gsl_multifit_fdfsolver_name(s));
  // ROS_INFO( "number of iterations: %zu\n",
  //         gsl_multifit_fdfsolver_niter(s));
  // ROS_INFO( "function evaluations: %zu\n", f.nevalf);
  // ROS_INFO( "Jacobian evaluations: %zu\n", f.nevaldf);
   //ROS_INFO( "reason for stopping: %s\n",
    //       (info_cyl == 1) ? "small step size" : "small gradient");
  //ROS_INFO( "initial_cyl |f(x)| = %g\n", chi0_cyl);
   //ROS_INFO( "final_cyl   |f(x)| = %g\n", chi_cyl);
   //ROS_INFO( "ratio_cyl   = %g\n", chi0_cyl/chi_cyl);
  double ratio_inv_cyl=  chi_cyl/chi0_cyl;
  //ROS_INFO( "ratio_inv_cyl   = %g\n", chi_cyl/chi0_cyl);
  

  
  double dof_cyl = n - p_cyl;
  double c_cyl = GSL_MAX_DBL(1, chi_cyl / sqrt(dof_cyl)); 
  //ROS_INFO( "c = %g\n", c);
  //ROS_INFO("chi / sqrt(dof) %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  //ROS_INFO( "chisq_cyl/dof_cyl = %g\n",  pow(chi_cyl, 2.0) / dof_cyl);

  ROS_INFO ( "x_0      = %.5f +/- %.5f\n", FIT_cyl(0), c_cyl*ERR_cyl(0));
  ROS_INFO ( "y_0 = %.5f +/- %.5f\n", FIT_cyl(1), c_cyl*ERR_cyl(1));
  ROS_INFO ( "z_0      = %.5f +/- %.5f\n", FIT_cyl(2), c_cyl*ERR_cyl(2));
  ROS_INFO ( "r      = %.5f +/- %.5f\n", FIT_cyl(3), c_cyl*ERR_cyl(3));
  ROS_INFO ("height: %f",(float)height );
  //ROS_INFO ( "status_cylinder = %s\n", gsl_strerror (status_cyl));
  
  position_msgs.point.x=FIT_cyl(0);
  position_msgs.point.y=FIT_cyl(1);
  position_msgs.point.z=FIT_cyl(2);
  position_msgs.header.seq=2;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  
   
  // marker visualization
  // creating marker 

  
  visualization_msgs::Marker marker_2;
  marker_2.header.frame_id = "left_arm_7_link";
  marker_2.header.stamp = ros::Time();
  marker_2.ns = "Grasp_Shape_Estimator_Server";
  marker_2.id = 1;
  marker_2.type = visualization_msgs::Marker::CYLINDER;
  marker_2.action = visualization_msgs::Marker::ADD;
  marker_2.pose.position.x = FIT_cyl(0);
  marker_2.pose.position.y = FIT_cyl(1);
  marker_2.pose.position.z = FIT_cyl(2);
  //const tfScalar &yaw, const tfScalar &pitch, const tfScalar &roll
  tf::Quaternion q;
  q.setRPY(-pi/2,0,0);
  // marker_2.pose.orientation.x = 1.0;
  // marker_2.pose.orientation.y = 0.0;
  // marker_2.pose.orientation.z = 0.0;
  // marker_2.pose.orientation.w = 0.0;
  marker_2.pose.orientation.x = q.x();
  marker_2.pose.orientation.y = q.y();
  marker_2.pose.orientation.z = q.z();
  marker_2.pose.orientation.w = q.w();
  marker_2.scale.x = 2*fabs(FIT_cyl(3));
  marker_2.scale.y = 2*fabs(FIT_cyl(3));
  marker_2.scale.z = height;
  marker_2.color.a = 0.2; // Don't forget to set the alpha!
  // marker_2.color.r = (float)ratio_inv_cyl;
  // marker_2.color.g = 0.0f;
  // marker_2.color.b = 1.0f-(float)ratio_inv_cyl;

  marker_2.color.r = 0.0f;
  marker_2.color.g = 0.0f;
  marker_2.color.b = 1.0f;
  marker_2.lifetime = ros::Duration(0.2);


  
  // publishing marker
  double r_cylinder = FIT_cyl(3);

  if (r_cylinder>0.01&&r_cylinder<0.04 && (1.0 - ratio_inv) < 0.75){
  marker_pub_.publish(marker_2);
  pub_hand_link_position_.publish(position_msgs);
  }
  ROS_INFO("Fitting cylinder DONE");
  //gsl_multifit_fdfsolver_free (s);
  //gsl_matrix_free (covar);
  //gsl_matrix_free (J);
  //gsl_rng_free (r);
 
}


// publishing data
void Grasp_Shape_Estimator_Server::publishDataState()
{
	pub_shape_grasp_states_.publish(grasp_shape_estimate_now);
	return;
}

}//namespace grasp_estimator


// MAIN
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Grasp_Shape_Estimator_Server");
    
	ros::NodeHandle nh;
    	grasp_estimator::Grasp_Shape_Estimator_Server Grasp_Shape_Estimator_Server(nh);
    
    ROS_INFO("Grasp_Shape_Estimator_Server is Here !!!");
    //int index=(int)0;
	ros::Rate loop_rate(10);

	while (ros::ok())
	{

		Grasp_Shape_Estimator_Server.publishDataState();

		ros::spinOnce();
		//ROS_INFO("Data Published !");
		loop_rate.sleep();
	}

	return 0;
}
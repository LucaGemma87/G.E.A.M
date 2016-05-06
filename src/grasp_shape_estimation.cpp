

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
#include "grasp_estimator/WeightEstimated.h"
#include "grasp_estimator/GraspShape.h"

// ROS headers
#include <ros/ros.h>

// ROS Control
//#include "control_msgs/JointTrajectoryControllerState.h"

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
#include <std_srvs/Empty.h>

//#include <gsl/gsl_multifit.h>

/* number of data points to fit */
#define N 19
#define pi 3.14
#define Hz 20

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

  ros::Subscriber hand_move_sub;
   
    //msgs definition
  grasp_estimator::DataAcquired msg_data_acquired_;

  grasp_estimator::DataAcquired msg_data_acquired_now;
    
  grasp_estimator::GraspShape grasp_shape_estimate_now;

    /// declaration of service servers
    ros::ServiceServer srvServer_Start_;
    ros::ServiceServer srvServer_Stop_;
 

public:

      bool run_state;
      bool hand_stopped;
  // callback functions
      //double sign(double number);
      void grasp_shape_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_);
      void publishDataState();
      static int sphere_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      static int sphere_df(const gsl_vector * x, void *data_3d,gsl_matrix * J);
      static int cylinder_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      static int cylinder_df(const gsl_vector * x, void *data_3d,gsl_matrix * J);
      static int ellissoid_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      static int ellissoid_df(const gsl_vector * x, void *data_3d,gsl_matrix * J);
      static int plane_f(const gsl_vector *x, void *data_3d, gsl_vector *f);
      static int plane_df(const gsl_vector * x, void *data_3d,gsl_matrix * J);

      bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);  

        
      void take_hand_stopped(const grasp_estimator::WeightEstimated::ConstPtr &mass_now);

      //static int expb_df(const gsl_vector * x, void *data,gsl_matrix * J);
  // constructor
  Grasp_Shape_Estimator_Server(ros::NodeHandle nh) : 
    nh_(nh), 
        priv_nh_("~")
  {
        
        msg_grasp_state_now_ = nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),10,
                                                                                     &Grasp_Shape_Estimator_Server::grasp_shape_estimation_server,
                                                                                     this);

        hand_move_sub=nh_.subscribe(nh_.resolveName("/left_hand/mass_estimator_server_"),1,&Grasp_Shape_Estimator_Server::take_hand_stopped,this);
    srvServer_Start_ = nh_.advertiseService("/Grasp_Shape_Estimator_Server/start", &Grasp_Shape_Estimator_Server::srvCallback_Start,this);
    srvServer_Stop_ = nh_.advertiseService("/Grasp_Shape_Estimator_Server/stop", &Grasp_Shape_Estimator_Server::srvCallback_Stop, this);
    // advertise topics
    pub_shape_grasp_states_ = nh_.advertise<grasp_estimator::GraspShape>(nh_.resolveName("/left_hand/grasp_shape_estimator_server_"), 10);
  
     pub_hand_link_position_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/hand_link_server"), 10);
    
    marker_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName( "/left_hand/shape_markers"), 10);
   
   run_state=false; 
   hand_stopped=false;
  }

  //! Empty stub
  ~Grasp_Shape_Estimator_Server() {}

};


  bool Grasp_Shape_Estimator_Server::srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("Starting Grasp_Shape_Estimator_Server");
      run_state = true;
      
    }


   bool Grasp_Shape_Estimator_Server::srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("Stopping Grasp_Shape_Estimator_Server");
        run_state = false;
       
    }

  void Grasp_Shape_Estimator_Server::take_hand_stopped(const grasp_estimator::WeightEstimated::ConstPtr &mass_now)
{
   hand_stopped = mass_now->hand_stopped;
}

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

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// SIGN  //////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// double Grasp_Shape_Estimator_Server::sign(double number)
// { 
//   if(number!=0)
//   {
//    if(number>0){return  1.0;}
//    else{return -1.0;}
//   }
//   else{return 0.0;}
// }
      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// SPHERE FITTING /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sphere (x-x_O)^2+(y-y_O)^2+(z-z_O)^2=r^2

int Grasp_Shape_Estimator_Server::sphere_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta;
 
  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  //ROS_INFO("X_data : %lf, %lf, %lf",((struct data_3d *)data_3d)->x[0],((struct data_3d *)data_3d)->y[0],((struct data_3d *)data_3d)->z[0]);

  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r = gsl_vector_get(x, 3);

   size_t j;
   double y;
   //ROS_INFO("Initializing DONE");  

   for (j = 0; j < n_theta; j++)
     {
      
         y =(pow((x_data[j]-x_0),2)+pow((y_data[j]-y_0),2)+pow((z_data[j]-z_0),2))-pow(r,2);
         //ROS_INFO("y %f",(float)y);
         //gsl_vector_set(f,i, Y-y);
         gsl_vector_set(f,j, y);

         //double y = (pow((x_data[i]-x_0),2)+pow((y_data[i]-y_0),2)+pow((z_data[i]-z_0),2)-pow(r,2));
         //gsl_vector_set(f,i, y);
         //ROS_INFO("Iter: %f ",(float)(i*(j+1)*(k+1)));
       
        //}       
      }

   return GSL_SUCCESS;
}

int Grasp_Shape_Estimator_Server::sphere_df(const gsl_vector * x, void *data_3d,gsl_matrix * J)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta; 

  // double A = gsl_vector_get (x, 0);
  // double lambda = gsl_vector_get (x, 1);
  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r = gsl_vector_get(x, 3);
  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  
  // Y = (pow((x_data[i]-x_0),2)+pow((y_data[i]-y_0),2)+pow((z_data[i]-z_0),2))-r^2
  // size_t i;
  // for (i = 0; i < n_theta; i++)
  //    {   

  //        gsl_matrix_set (J, i, 0, (2*x_0-2*x_data[i])); 
  //        gsl_matrix_set (J, i, 1, (2*y_0-2*y_data[i]));
  //        gsl_matrix_set (J, i, 2, (2*z_0-2*z_data[i]));
  //        gsl_matrix_set (J, i, 3, -2*r);
  //    }
   size_t j;
   double dy1,dy2,dy3,dy4;
   //ROS_INFO("Initializing DONE");  

   for (j = 0; j < n_theta; j++)
     {

         dy1=(2*x_0-2*x_data[j]);
         dy2=(2*y_0-2*y_data[j]);
         dy3=(2*z_0-2*z_data[j]);
         dy4=(-2*r);

       gsl_matrix_set (J, j, 0, dy1); 
       gsl_matrix_set (J, j, 1, dy2);
       gsl_matrix_set (J, j, 2, dy3);
       gsl_matrix_set (J, j, 3, dy4);

     }
            
   return GSL_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CYLINDER FITTING  ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Grasp_Shape_Estimator_Server::cylinder_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta;
  
  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  
  double x_0 = gsl_vector_get(x, 0);
  double z_0 = gsl_vector_get(x, 1);
  double r1 = gsl_vector_get(x, 2);
  double r2 = gsl_vector_get(x, 3);
  

   size_t j;
   double y;
   //ROS_INFO("Initializing DONE");  
   for (j = 0; j < n_theta; j++)
     {
         
         //y=(pow((x_data[j]-x_0),2)+pow((z_data[j]-z_0),2))-pow(r,2);
         y=(pow((x_data[j]-x_0)/r1,2)+pow((z_data[j]-z_0)/r2,2))-1.0;
         
          //pow((y_data[i]-y_0),2)-height/2;
         //ROS_INFO("y %f",(float)y); +pow(z_data[i]-z_0,2) +pow(y_data[i]-y_0,2)
        
         gsl_vector_set(f,j, y);
         //double y = (pow((x_data[i]-x_0),2)+pow((z_data[i]-z_0),2)-pow(r,2));
         //gsl_vector_set(f,i, y);
         //ROS_INFO("Iter: %f ",(float)(i*(j+1)*(k+1)));      
       }

   return GSL_SUCCESS;
}

int Grasp_Shape_Estimator_Server::cylinder_df(const gsl_vector * x, void *data_3d,gsl_matrix * J)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta; 

  // double A = gsl_vector_get (x, 0);
  // double lambda = gsl_vector_get (x, 1);
  double x_0 = gsl_vector_get(x, 0);
  double z_0 = gsl_vector_get(x, 1);
  double r1 = gsl_vector_get(x, 2);
  double r2 = gsl_vector_get(x, 3);

  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  size_t j;
   //  double y = (pow((x_data[i]-x_0),2)+pow((z_data[i]-z_0),2))-pow(r,2);
  // for (i = 0; i < n_theta; i++)
  //    {   
  //        gsl_matrix_set (J, i, 0, (2*x_0-2*x_data[i])); 
  //        gsl_matrix_set (J, i, 1, 0 );
  //        gsl_matrix_set (J, i, 2, (2*z_0-2*z_data[i]));
  //        gsl_matrix_set (J, i, 3, -2*r);
  //    }
  double dy1,dy2,dy3,dy4;
  for (j = 0; j < n_theta; j++)
     {

        dy1= 1.0/(r1*r1)*(x_0*2.0-x_data[j]*2.0);
        dy2= 1.0/(r2*r2)*(z_0*2.0-z_data[j]*2.0);
        dy3= 1.0/(r1*r1*r1)*pow(x_0-x_data[j],2.0)*-2.0;
        dy4= 1.0/(r2*r2*r2)*pow(z_0-z_data[j],2.0)*-2.0; ;
         gsl_matrix_set (J, j, 0, dy1); 
         gsl_matrix_set (J, j, 1, dy2 );
         gsl_matrix_set (J, j, 2, dy3);
         gsl_matrix_set (J, j, 3, dy4);
     }
   return GSL_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// ELLISSOID FITTING /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sphere (x-x_O)^2+(y-y_O)^2+(z-z_O)^2=r^2

int Grasp_Shape_Estimator_Server::ellissoid_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
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
  //ROS_INFO("X_data : %lf, %lf, %lf",((struct data_3d *)data_3d)->x[0],((struct data_3d *)data_3d)->y[0],((struct data_3d *)data_3d)->z[0]);

  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r1 = gsl_vector_get(x, 3);
  double r2 = gsl_vector_get(x, 4);
  double r3 = gsl_vector_get(x, 5);


   //ROS_INFO("Initializing DONE");  

   size_t j;
   double y;
   //ROS_INFO("Initializing DONE");  
   for (j = 0; j < n_theta; j++)
     {


         y =pow((x_data[j]-x_0)/r1,2)+pow((y_data[j]-y_0)/r2,2)+pow((z_data[j]-z_0)/r3,2)-1.0;

         gsl_vector_set(f,j, y);
     }

   return GSL_SUCCESS;
}

int Grasp_Shape_Estimator_Server::ellissoid_df(const gsl_vector * x, void *data_3d,gsl_matrix * J)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta; 

  // double A = gsl_vector_get (x, 0);
  // double lambda = gsl_vector_get (x, 1);
  double x_0 = gsl_vector_get(x, 0);
  double y_0 = gsl_vector_get(x, 1);
  double z_0 = gsl_vector_get(x, 2);
  double r1 = gsl_vector_get(x, 3);
  double r2 = gsl_vector_get(x, 4);
  double r3 = gsl_vector_get(x, 5);

  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
 
  //double y = pow((x_data[i]-x_0)/r1,2)+pow((y_data[i]-y_0)/r2,2)+pow((z_data[i]-z_0)/r3,2)-1;
   size_t j;
   double dy1,dy2,dy3,dy4,dy5,dy6;
   //ROS_INFO("Initializing DONE");  
   for (j = 0; j < n_theta; j++)
     {

        dy1= 1.0/(r1*r1)*(x_0*2.0-x_data[j]*2.0);
        dy2= 1.0/(r2*r2)*(y_0*2.0-y_data[j]*2.0);
        dy3= 1.0/(r3*r3)*(z_0*2.0-z_data[j]*2.0);
        dy4= 1.0/(r1*r1*r1)*pow(x_0-x_data[j],2.0)*-2.0;
        dy5= 1.0/(r2*r2*r2)*pow(y_0-y_data[j],2.0)*-2.0;
        dy6= 1.0/(r3*r3*r3)*pow(z_0-z_data[j],2.0)*-2.0;
       
     
         gsl_matrix_set (J, j, 0, dy1); 
         gsl_matrix_set (J, j, 1, dy2);
         gsl_matrix_set (J, j, 2, dy3);
         gsl_matrix_set (J, j, 3, dy4);
         gsl_matrix_set (J, j, 4, dy5);
         gsl_matrix_set (J, j, 5, dy6);
     }
   return GSL_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// PLANE  FITTING /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int Grasp_Shape_Estimator_Server::plane_f(const gsl_vector *x, void *data_3d, gsl_vector *f)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta;
  
  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  

  double a = gsl_vector_get(x, 0);
  double b = gsl_vector_get(x, 1);
  double c = gsl_vector_get(x, 2);
  double d = gsl_vector_get(x, 3);
  double x_0 = gsl_vector_get(x, 4);
  double y_0 = gsl_vector_get(x, 5);
  double z_0 = gsl_vector_get(x, 6);

   size_t j;
   double y,y2;
   //ROS_INFO("Initializing DONE");  

   for (j = 0; j < n_theta; j++)
      { 
  
           y = sqrt(pow((a*x_data[j]+b*y_data[j]+c*z_data[j]+d),2))/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
           //y2 = fabs(a*(x_0)+b*(y_0)+c*(z_0)+d)/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
           y2 = sqrt(pow(x_data[j]-x_0,2)+pow(y_data[j]-y_0,2)+pow(z_data[j]-z_0,2));
         gsl_vector_set(f,j,y-y2);//-y2
         
      }

   return GSL_SUCCESS;

}

int Grasp_Shape_Estimator_Server::plane_df(const gsl_vector * x, void *data_3d,gsl_matrix * J)
{
  size_t n_theta = ((struct data_3d *)data_3d)->n_theta; 

  // double A = gsl_vector_get (x, 0);
  // double lambda = gsl_vector_get (x, 1);
  double a = gsl_vector_get(x, 0);
  double b = gsl_vector_get(x, 1);
  double c = gsl_vector_get(x, 2);
  double d = gsl_vector_get(x, 3);
  double x_0 = gsl_vector_get(x, 4);
  double y_0 = gsl_vector_get(x, 5);
  double z_0 = gsl_vector_get(x, 6);

  double *x_data = ((struct data_3d *)data_3d)->x;
  double *y_data = ((struct data_3d *)data_3d)->y;
  double *z_data = ((struct data_3d *)data_3d)->z;
  
  size_t j;
           // d=-(a*x_0+b*y_0+c*z_0);
           // y =pow(a*(x_data[j]-x_0)+b*(x_data[j]-y_0)+c*(x_data[j]-z_0)-d,2);
           // y2 =pow(x_data[j]-x_0,2)+pow(y_data[j]-y_0,2)+pow(z_data[j]-z_0,2); 
  double dy1,dy2,dy3,dy4,dy5,dy6,dy7;
   for (j = 0; j < n_theta; j++)
     {


        // dy1=x_data[j]+x_data[j+1]+x_0;
        // dy2=y_data[j]+y_data[j+1]+y_0;
        // dy3=z_data[j]+z_data[j+1]+z_0;

        dy1= -a*sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/pow(a*a+b*b+c*c,3.0/2.0)+
              x_data[j]*1.0/sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/sqrt(a*a+b*b+c*c)*(d+a*x_data[j]+
              b*y_data[j]+c*z_data[j]);
        
        dy2= -b*sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/pow(a*a+b*b+c*c,3.0/2.0)+
              y_data[j]*1.0/sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/sqrt(a*a+b*b+c*c)*(d+a*x_data[j]+
              b*y_data[j]+c*z_data[j]);
        

        dy3= -c*sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/pow(a*a+b*b+c*c,3.0/2.0)+
              z_data[j]*1.0/sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/sqrt(a*a+b*b+c*c)*(d+a*x_data[j]+
              b*y_data[j]+c*z_data[j]);

        dy4=1.0/sqrt(pow(d+a*x_data[j]+b*y_data[j]+c*z_data[j],2.0))*1.0/sqrt(a*a+b*b+c*c)*(d*2.0
            +a*x_data[j]*2.0+b*y_data[j]*2.0+c*z_data[j]*2.0)*(1.0/2.0);
        
        dy5=(x_0*2.0-x_data[j]*2.0)*1.0/sqrt(pow(x_0-x_data[j],2.0)+pow(y_0-y_data[j],2.0)+pow(z_0-z_data[j],2.0))*(-1.0/2.0);
        
        dy6=(y_0*2.0-y_data[j]*2.0)*1.0/sqrt(pow(x_0-x_data[j],2.0)+pow(y_0-y_data[j],2.0)+pow(z_0-z_data[j],2.0))*(-1.0/2.0);
        
        dy7= (z_0*2.0-z_data[j]*2.0)*1.0/sqrt(pow(x_0-x_data[j],2.0)+pow(y_0-y_data[j],2.0)+pow(z_0-z_data[j],2.0))*(-1.0/2.0);
      
         gsl_matrix_set (J, j, 0, dy1); 
         gsl_matrix_set (J, j, 1, dy2);
         gsl_matrix_set (J, j, 2, dy3);
         gsl_matrix_set (J, j, 3, dy4);
         gsl_matrix_set (J, j, 4, dy5);
         gsl_matrix_set (J, j, 5, dy6);
         gsl_matrix_set (J, j, 6, dy7);
         

     }
   return GSL_SUCCESS;

}

 


///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// End of GSL /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void Grasp_Shape_Estimator_Server::grasp_shape_estimation_server(const grasp_estimator::DataAcquired::ConstPtr &msg_data_acquired_)
{// const grasp_estimator::DataAcquired::ConstPtr msg_data_acquired_
 this->msg_data_acquired_now=*msg_data_acquired_;
 if( (this->msg_data_acquired_now.hand_index_distal_link.transform.translation.x!=0) &&
     (this->msg_data_acquired_now.hand_index_distal_link.transform.translation.y!=0) &&
     (this->msg_data_acquired_now.hand_index_distal_link.transform.translation.z!=0) &&
 
     (this->msg_data_acquired_now.hand_index_distal_link.transform.rotation.x!=0     )    &&
     (this->msg_data_acquired_now.hand_index_distal_link.transform.rotation.y!=0     )    &&
     (this->msg_data_acquired_now.hand_index_distal_link.transform.rotation.z!=0     )    &&
     (this->msg_data_acquired_now.hand_index_distal_link.transform.rotation.w!=0     )  )
                                                                                      

  { 
   if(run_state==true&&hand_stopped==true){ 
   grasp_estimator::GraspShape grasp_shape; 
  //ROS_INFO("<--------Grasp Shape Estimator START-------->\n");
 
  //ROS_INFO("Grasp Shape Estimator GSl initialization");
  
  const gsl_multifit_fdfsolver_type *T = gsl_multifit_fdfsolver_lmsder;
  const gsl_multifit_fdfsolver_type *T_cyl = gsl_multifit_fdfsolver_lmsder;
  const gsl_multifit_fdfsolver_type *T_ell = gsl_multifit_fdfsolver_lmsder;
  const gsl_multifit_fdfsolver_type *T_plan = gsl_multifit_fdfsolver_lmsder;
  gsl_multifit_fdfsolver *s,*s_cyl,*s_ell,*s_plan;
  int status, info,status_cyl,info_cyl,status_ell,info_ell,status_plan,info_plan;
  size_t i;
  const size_t n = N;
  const size_t n_theta=N;  
  const size_t p = 4;
  const size_t p_cyl = 4;
  const size_t p_ell = 6;
  const size_t p_plan = 7;
  // init sphere
  gsl_matrix *J = gsl_matrix_alloc(n, p);
  gsl_matrix *covar = gsl_matrix_alloc (p, p);
  // init cylinder
  gsl_matrix *J_cyl = gsl_matrix_alloc(n, p_cyl);
  gsl_matrix *covar_cyl = gsl_matrix_alloc (p_cyl, p_cyl);
  // init ellissoid
  gsl_matrix *J_ell = gsl_matrix_alloc(n, p_ell);
  gsl_matrix *covar_ell = gsl_matrix_alloc (p_ell, p_ell);
   // init plan
  gsl_matrix *J_plan = gsl_matrix_alloc(n, p_plan);
  gsl_matrix *covar_plan = gsl_matrix_alloc (p_plan, p_plan);


  double x_data[N],y_data[N],z_data[N], weights_sph[N], weights_cyl[N], weights_ell[N], weights_plan[N];
  double x_min,x_max,y_min,y_max,z_min,z_max;
  
  struct data_3d d_3d = { n_theta,x_min,x_max,y_min,y_max,z_min,z_max,x_data,y_data,z_data};
  gsl_multifit_function_fdf f,f_cyl,f_ell,f_plan;
  
  //ROS_INFO("Grasp Shape Estimator receiving message");
  
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
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_index_distal_link, hand_index_distal_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_index_knuckle_link,hand_index_knuckle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_index_middle_link,hand_index_middle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_index_proximal_link,hand_index_proximal_link_stamped);

  //little
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_little_distal_link,hand_little_distal_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_little_knuckle_link,hand_little_knuckle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_little_middle_link,hand_little_middle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_little_proximal_link,hand_little_proximal_link_stamped);

  //middle
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_middle_distal_link,hand_middle_distal_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_middle_knuckle_link,hand_middle_knuckle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_middle_middle_link,hand_middle_middle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_middle_proximal_link,hand_middle_proximal_link_stamped);

  //ring
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_ring_distal_link, hand_ring_distal_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_ring_knuckle_link,hand_ring_knuckle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_ring_middle_link,hand_ring_middle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_ring_proximal_link,hand_ring_proximal_link_stamped);

  //thumb 
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_thumb_distal_link,hand_thumb_distal_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_thumb_knuckle_link,hand_thumb_knuckle_link_stamped);
  transformStampedMsgToTF(this->msg_data_acquired_now.hand_thumb_proximal_link,hand_thumb_proximal_link_stamped);


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
  double radius=fabs(x_max-x_min)/2;

  //ROS_INFO("Bound x_min,x_max,y_min,y_max,z_min,z_max %f,%f,%f,%f,%f,%f ",(float)x_min,(float)x_max,(float)y_min,(float)y_max,(float)z_min,(float)z_max);
  //ROS_INFO(" Center_x: %f, center_y: %f, center_z: %f",(float)center_x,(float)center_y,(float)center_z);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL  SPHERE  ////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ROS_INFO("Fitting Sphere");
  double x_init[4] = { center_x, center_y, center_z, radius};
  gsl_vector_view x = gsl_vector_view_array(x_init, p);
  for (i = 0; i < n; i++)
    {
      weights_sph[i] = 1.0 / (0.00045*0.00045);
      // weights_sph[i] = 1.0 / (0.1*0.1);
    }; 
  gsl_vector_view w = gsl_vector_view_array(weights_sph, n);
  const gsl_rng_type * type;
  gsl_rng * r;
  gsl_vector *res_f;
  double chi, chi0;

  const double xtol = 1e-20;
  const double gtol = 1e-20;
  const double ftol = 1e-20;

  gsl_rng_env_setup();

  type = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f.f =&sphere_f;

   //f.df =NULL;
   f.df =&sphere_df;
   //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
   f.n = n;
   f.p = p;
   f.params = &d_3d;

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
  status = gsl_multifit_fdfsolver_driver(s, 2000, xtol, gtol, ftol, &info);

  gsl_multifit_fdfsolver_jac(s, J);
  gsl_multifit_covar(J, 0.0, covar);

  /* compute final residual norm */
  res_f = gsl_multifit_fdfsolver_residual(s);
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
  //ROS_INFO( "initial |f(x)| = %g\n", chi0);
  //ROS_INFO( "final   |f(x)| = %g\n", chi);
  // ROS_INFO( "ratio   = %g\n", chi0/chi);
  double ratio_inv=  chi/chi0;
  //ROS_INFO( "ratio_inv   = %g\n", chi/chi0);
  

  
  double dof = n - p;
  double c = GSL_MAX_DBL(1, chi / sqrt(dof)); 
  //ROS_INFO( "c_sph = %g\n", c);
  //ROS_INFO("Sphere dev_std  %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  double index_sph = pow(chi, 2.0) / dof;
  //ROS_INFO( "Index_sph = %g",  index_sph);

  // ROS_INFO ( "status_sph = %s\n", gsl_strerror (status));
  

  
   
  // marker visualization
  // creating marker 
  visualization_msgs::Marker marker_sph;
  marker_sph.header.frame_id = "left_arm_7_link";
  marker_sph.header.stamp = ros::Time();
  marker_sph.ns = "Grasp_Shape_Estimator_Server";
  marker_sph.id = 0;
  marker_sph.type = visualization_msgs::Marker::SPHERE;
  marker_sph.action = visualization_msgs::Marker::ADD;
  marker_sph.pose.position.x = FIT(0);
  marker_sph.pose.position.y = FIT(1);
  marker_sph.pose.position.z = FIT(2);
  marker_sph.pose.orientation.x = 0.0;
  marker_sph.pose.orientation.y = 0.0;
  marker_sph.pose.orientation.z = 0.0;
  marker_sph.pose.orientation.w = 1.0;
  marker_sph.scale.x = 2*fabs(FIT(3));
  marker_sph.scale.y = 2*fabs(FIT(3));
  marker_sph.scale.z = 2*fabs(FIT(3));
  marker_sph.color.a = 0.5; // Don't forget to set the alpha!
  marker_sph.color.r = (float)(pow(chi, 2.0) / dof);
  marker_sph.color.g = 1.0-(float)(pow(chi, 2.0) / dof);
  marker_sph.color.b = 0.0f;
  marker_sph.lifetime = ros::Duration(0.2);

  double v_sph((4/3)*pi*pow(fabs(FIT(3)),3));
  //ROS_INFO("Sphere volume = %f",(float)(v_sph));
  //ROS_INFO("Fitting sphere DONE \n");

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL  Cylinder  ////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  double height= fabs(y_max-y_min);
  //ROS_INFO("Fitting Cylinder");
  double x_init_cyl[4] = { center_x,center_z, radius,radius};
  gsl_vector_view x_cyl = gsl_vector_view_array(x_init_cyl, p_cyl);
    for (i = 0; i < n; i++)
    {
      weights_cyl[i] = 1.0 / (0.35*0.35);
      //weights_cyl[i] = 1.0 / (0.27*0.27);
      //weights_cyl[i] = 1.0 / (0.1*0.1);
    }; 
  gsl_vector_view w_cyl = gsl_vector_view_array(weights_cyl, n);
  const gsl_rng_type *type_cyl;
  gsl_rng *r_cyl;
  gsl_vector *res_f_cyl;
  double chi_cyl, chi0_cyl;

  gsl_rng_env_setup();

  type_cyl = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f_cyl.f =&cylinder_f;

   //f_cyl.df =NULL;
   f_cyl.df =&cylinder_df;
   //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
   f_cyl.n = n;
   f_cyl.p = p_cyl;
   f_cyl.params = &d_3d;


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
  status_cyl = gsl_multifit_fdfsolver_driver(s_cyl, 2000, xtol, gtol, ftol, &info_cyl);

  gsl_multifit_fdfsolver_jac(s_cyl, J_cyl);
  gsl_multifit_covar(J_cyl, 0.0, covar_cyl);

  /* compute final residual norm */
  res_f_cyl = gsl_multifit_fdfsolver_residual(s_cyl);
  chi_cyl = gsl_blas_dnrm2(res_f_cyl);

  #define FIT_cyl(i) gsl_vector_get(s_cyl->x, i)
  #define ERR_cyl(i) sqrt(gsl_matrix_get(covar_cyl,i,i))

  // ROS_INFO("summary from method '%s'\n",gsl_multifit_fdfsolver_name(s));
  // ROS_INFO( "number of iterations: %zu\n",
  //         gsl_multifit_fdfsolver_niter(s));
  // ROS_INFO( "function evaluations: %zu\n", f.nevalf);
  // ROS_INFO( "Jacobian evaluations: %zu\n", f.nevaldf);
  //  ROS_INFO( "reason for stopping: %s\n",
  //         (info_cyl == 1) ? "small step size" : "small gradient");
  // ROS_INFO( "initial_cyl |f(x)| = %g\n", chi0_cyl);
  //  ROS_INFO( "final_cyl   |f(x)| = %g\n", chi_cyl);
  //  ROS_INFO( "ratio_cyl   = %g\n", chi0_cyl/chi_cyl);
  double ratio_inv_cyl=  chi_cyl/chi0_cyl;
  //ROS_INFO( "ratio_inv_cyl   = %g\n", chi_cyl/chi0_cyl);
  

  
  double dof_cyl = n - p_cyl;
  double c_cyl = GSL_MAX_DBL(1, chi_cyl / sqrt(dof_cyl)); 
  //ROS_INFO( "c_cyl = %g\n", c_cyl);
  //ROS_INFO("chi / sqrt(dof) %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  //ROS_INFO( "chisq_cyl/dof_cyl = %g\n",  pow(chi_cyl, 2.0) / dof_cyl);
  double index_cyl = pow(chi_cyl, 2.0) / dof_cyl;
  //ROS_INFO( "Index_cyl = %g",  index_cyl);

  //ROS_INFO ("height: %f",(float)height );
  //ROS_INFO ( "status_cylinder = %s\n", gsl_strerror (status_cyl));
  

  
   
  // marker visualization
  // creating marker 

  
  visualization_msgs::Marker marker_cyl;
  marker_cyl.header.frame_id = "left_arm_7_link";
  marker_cyl.header.stamp = ros::Time();
  marker_cyl.ns = "Grasp_Shape_Estimator_Server";
  marker_cyl.id = 1;
  marker_cyl.type = visualization_msgs::Marker::CYLINDER;
  marker_cyl.action = visualization_msgs::Marker::ADD;
  marker_cyl.pose.position.x = FIT_cyl(0);
  marker_cyl.pose.position.y = center_y;
  marker_cyl.pose.position.z = FIT_cyl(1);
  //const tfScalar &yaw, const tfScalar &pitch, const tfScalar &roll
  tf::Quaternion q;
  q.setRPY(-pi/2,0,0);
  marker_cyl.pose.orientation.x = q.x();
  marker_cyl.pose.orientation.y = q.y();
  marker_cyl.pose.orientation.z = q.z();
  marker_cyl.pose.orientation.w = q.w();
  marker_cyl.scale.x = 2*fabs(FIT_cyl(2));
  marker_cyl.scale.y = 2*fabs(FIT_cyl(3));
  marker_cyl.scale.z =  height;
  marker_cyl.color.a =  0.5; // Don't forget to set the alpha!
  marker_cyl.color.r = 1.0-(float)(pow(chi_cyl, 2.0) / dof_cyl);
  marker_cyl.color.g = 0.0f;
  marker_cyl.color.b = (float)(pow(chi_cyl, 2.0) / dof_cyl);

  // marker_2.color.r = 0.0f;
  // marker_2.color.g = 0.0f;
  // marker_2.color.b = 1.0f;
  marker_cyl.lifetime = ros::Duration(0.2);


  
  // publishing marker
  double r_cylinder = FIT_cyl(3);

  double v_cyl(pi*fabs(FIT_cyl(2))*fabs(FIT_cyl(3))*height);
  //ROS_INFO("Cylinder volume = %f",(float)(v_cyl));
  //ROS_INFO("Fitting cylinder DONE \n");
  
 
  //ROS_INFO("Fitting cylinder DONE");
  //gsl_multifit_fdfsolver_free (s);
  //gsl_matrix_free (covar);
  //gsl_matrix_free (J);
  //gsl_rng_free (r);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL  ELLISSOID    ///////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ROS_INFO("Ellissoid fitting");
  double radius_1(radius),radius_2(radius),radius_3(radius);
  double x_init_ell[6] = { FIT_cyl(0), center_y, FIT_cyl(1), FIT_cyl(2),height/2,FIT_cyl(3)};
  gsl_vector_view x_ell = gsl_vector_view_array(x_init_ell, p_ell);
    for (i = 0; i < n; i++)
    {
      //weights_ell[i] = 1.0 / (0.11*0.11);
      weights_ell[i] = 1.0 / (0.2*0.2);
      
    }; 
  gsl_vector_view w_ell = gsl_vector_view_array(weights_ell, n);
  const gsl_rng_type * type_ell;
  gsl_rng * r_ell;
  gsl_vector *res_f_ell;
  double chi_ell, chi0_ell;

  const double xtol_ell = 1e-20;
  const double gtol_ell = 1e-20;
  const double ftol_ell = 1e-20;

  gsl_rng_env_setup();

  type_ell = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f_ell.f =&ellissoid_f;

   //f_ell.df =NULL;
   f_ell.df =&ellissoid_df;   /* set to NULL for finite-difference Jacobian */
   f_ell.n = n;
   f_ell.p = p_ell;
   f_ell.params = &d_3d;

  //ROS_INFO("Data created");
  s_ell = gsl_multifit_fdfsolver_alloc(T_ell, n, p_ell);
  //ROS_INFO("Alloc Done");
  
  /* initialize solver with starting point and weights */
  gsl_multifit_fdfsolver_wset(s_ell, &f_ell, &x_ell.vector, &w_ell.vector);
  //ROS_INFO("wset Done"); 
   /* compute initial residual norm */
  //ROS_INFO("Error Ellissoid fitting 1");
  res_f_ell = gsl_multifit_fdfsolver_residual(s_ell);
  //ROS_INFO("gsl_multifit_fdfsolver_residual Done"); 
  //ROS_INFO("Error Ellissoid fitting 2");
  chi0_ell = gsl_blas_dnrm2(res_f_ell);
  
  /* solve the system with a maximum of 20 iterations */
  status_ell = gsl_multifit_fdfsolver_driver(s_ell, 2000, xtol_ell, gtol_ell, ftol_ell, &info_ell);
  //ROS_INFO("Error Ellissoid fitting 3");
  gsl_multifit_fdfsolver_jac(s_ell, J_ell);
  gsl_multifit_covar(J_ell, 0.0, covar_ell);

  /* compute final residual norm */
  res_f_ell = gsl_multifit_fdfsolver_residual(s_ell);
  chi_ell = gsl_blas_dnrm2(res_f_ell);

  #define FIT_ell(i) gsl_vector_get(s_ell->x, i)
  #define ERR_ell(i) sqrt(gsl_matrix_get(covar_ell,i,i))

  // ROS_INFO("summary from method '%s'\n",gsl_multifit_fdfsolver_name(s));
  // ROS_INFO( "number of iterations: %zu\n",
  //         gsl_multifit_fdfsolver_niter(s));
  // ROS_INFO( "function evaluations: %zu\n", f.nevalf);
  // ROS_INFO( "Jacobian evaluations: %zu\n", f.nevaldf);
  // ROS_INFO( "reason for stopping: %s\n",
  //       (info_ell == 1)? "small step size" : "small gradient");
  // ROS_INFO( "initial |f(x)| = %g\n", chi0);
  // ROS_INFO( "final   |f(x)| = %g\n", chi);
  // ROS_INFO( "ratio   = %g\n", chi0/chi);
  double ratio_inv_ell=  chi_ell/chi0_ell;
  //ROS_INFO( "ratio_inv_ell   = %g\n", ratio_inv_ell);
  

  
  double dof_ell = n - p_ell;
  double c_ell = GSL_MAX_DBL(1, chi_ell / sqrt(dof_ell)); 
  //ROS_INFO( "c_ell = %g\n", c_ell);
  //ROS_INFO("chi / sqrt(dof) %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  // ROS_INFO( "chisq/dof = %g\n",  pow(chi, 2.0) / dof);
  double index_ell =  pow(chi_ell, 2.0) / dof_ell;
  //ROS_INFO( "Index_ell = %g",  index_ell);

  //ROS_INFO ( "status_ell = %s\n", gsl_strerror (status));
  

  
   
  // marker visualization
  // creating marker 
  visualization_msgs::Marker marker_ell;
  marker_ell.header.frame_id = "left_arm_7_link";
  marker_ell.header.stamp = ros::Time();
  marker_ell.ns = "Grasp_Shape_Estimator_Server";
  marker_ell.id = 2;
  marker_ell.type = visualization_msgs::Marker::SPHERE;
  marker_ell.action = visualization_msgs::Marker::ADD;
  marker_ell.pose.position.x = FIT_ell(0);
  marker_ell.pose.position.y = FIT_ell(1);
  marker_ell.pose.position.z = FIT_ell(2);
  marker_ell.pose.orientation.x = 0.0;
  marker_ell.pose.orientation.y = 0.0;
  marker_ell.pose.orientation.z = 0.0;
  marker_ell.pose.orientation.w = 1.0;
  marker_ell.scale.x = 2*fabs(FIT_ell(3));
  marker_ell.scale.y = 2*fabs(FIT_ell(4));
  marker_ell.scale.z = 2*fabs(FIT_ell(5)); //2*
  marker_ell.color.a = 0.5; // Don't forget to set the alpha!
  marker_ell.color.r = 0.0f;
  marker_ell.color.g = (float)(pow(chi_ell, 2.0)/dof_ell);
  marker_ell.color.b = 1-(float)(pow(chi_ell, 2.0)/dof_ell);
  marker_ell.lifetime = ros::Duration(0.2);
  // publishing marker
  //double r_sphere = FIT_ell(3);
  double r_1(FIT_ell(3)),r_2(FIT_ell(4)),r_3(FIT_ell(5));

  double v_ell((4/3)*pi*fabs(FIT_ell(3))*fabs(FIT_ell(4))*fabs(FIT_ell(5)));
  //ROS_INFO("Ellissoid volume = %f",(float)(v_ell));

  //ROS_INFO("Fitting Ellissoid DONE \n");

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////// GSL  PLANE   ///////  ////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  double a0(1.0),b0(1.0),c0(1.0),d0(-(a0*center_x+b0*center_y+c0*center_z));
  double x_init_plan[7] = { a0, b0, c0, d0,center_x,center_y,center_z};
  gsl_vector_view x_plan = gsl_vector_view_array(x_init_plan, p_plan); 
    for (i = 0; i < n; i++)
    {
      weights_plan[i] = 1.0 / (0.01*0.01);
    }; 
  gsl_vector_view w_plan = gsl_vector_view_array(weights_plan, n);
  const gsl_rng_type * type_plan;
  gsl_rng * r_plan;
  gsl_vector *res_f_plan;
  double chi_plan, chi0_plan;

  const double xtol_plan = 1e-20;
  const double gtol_plan = 1e-20;
  const double ftol_plan = 1e-20;

  gsl_rng_env_setup();

  type_plan = gsl_rng_default;
  //r = gsl_rng_alloc (type);

   //f.f =&expb_f;
   f_plan.f =&plane_f;
   //f_plan.df =NULL;
   f_plan.df =&plane_df;
   //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
   f_plan.n = n;
   f_plan.p = p_plan;
   f_plan.params = &d_3d;


  //ROS_INFO("Data created");
  s_plan = gsl_multifit_fdfsolver_alloc(T_plan, n, p_plan);
  //ROS_INFO("Alloc Done");
  /* initialize solver with starting point and weights */
  gsl_multifit_fdfsolver_wset(s_plan, &f_plan, &x_plan.vector, &w_plan.vector);
  //ROS_INFO("wset Done"); 
   /* compute initial residual norm */
  res_f_plan = gsl_multifit_fdfsolver_residual(s_plan);
  //ROS_INFO("gsl_multifit_fdfsolver_residual Done"); 
  chi0_plan = gsl_blas_dnrm2(res_f_plan);

  /* solve the system with a maximum of 20 iterations */
  status_plan = gsl_multifit_fdfsolver_driver(s_plan, 20000, xtol_plan, gtol_plan, ftol_plan, &info_plan);

  gsl_multifit_fdfsolver_jac(s_plan, J_plan);
  gsl_multifit_covar(J_plan, 0.0, covar_plan);

  /* compute final residual norm */
  res_f_plan = gsl_multifit_fdfsolver_residual(s_plan);
  chi_plan = gsl_blas_dnrm2(res_f_plan);

  #define FIT_plan(i) gsl_vector_get(s_plan->x, i)
  #define ERR_plan(i) sqrt(gsl_matrix_get(covar_plan,i,i))

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
  double ratio_inv_plan=  chi_plan/chi0_plan;
  //ROS_INFO( "ratio_inv_plan   = %g\n",1.0 - chi_plan/chi0_plan);
  

  
  double dof_plan = n - p_plan;
  double c_plan = GSL_MAX_DBL(1, chi_plan / sqrt(dof_plan)); 
  //ROS_INFO( "c_plan = %g\n", c_plan);
  //ROS_INFO("chi / sqrt(dof) %g\n",chi / sqrt(dof));
  // I_neg = (double)(chi / sqrt(dof));
  //ROS_INFO( "dev standard plan= %g\n",  pow(chi_plan, 2.0) / dof_plan);
  double index_plan = pow(chi_plan, 2.0) / dof_plan;
  //ROS_INFO( "Index_plan = %g",  index_plan);


  // ROS_INFO ( "status_sph = %s\n", gsl_strerror (status));
  double height_plan = fabs(z_max-z_min);
  double width_plan = fabs(y_max-y_min);
  double depth_plan = fabs(x_max-x_min);
   
   tf::Vector3 normal_vector(FIT_plan(0), FIT_plan(1),FIT_plan(2));
   tf::Vector3 x_axis(1,0,0);
   tf::Vector3 y_axis(0,1,0);
   tf::Vector3 z_axis(0,0,1);
   
   tfScalar roll(normal_vector.angle(x_axis));   
   tfScalar pitch(normal_vector.angle(y_axis));  
   tfScalar yaw(normal_vector.angle(z_axis));  

  //ROS_INFO("Roll = %f, Pitch = %f , Yaw = %f. ",(float)roll,(float)pitch,(float)yaw);

   tf::Quaternion q_plan,q_normal;
   q_normal.setRPY(roll,pitch,yaw);
   q_plan.setRPY(roll,pitch-pi/4,yaw-pi/4);
   
  // marker visualization
  // creating marker 
  visualization_msgs::Marker marker_box;
  marker_box.header.frame_id = "left_arm_7_link";
  marker_box.header.stamp = ros::Time();
  marker_box.ns = "Grasp_Shape_Estimator_Server";
  marker_box.id = 3;
  marker_box.type = visualization_msgs::Marker::CUBE;
  marker_box.action = visualization_msgs::Marker::ADD;
  marker_box.pose.position.x = FIT_plan(4);
  marker_box.pose.position.y = FIT_plan(5);
  marker_box.pose.position.z = FIT_plan(6);
  marker_box.pose.orientation.x = q_plan.x();
  marker_box.pose.orientation.y = q_plan.y();
  marker_box.pose.orientation.z = q_plan.z();
  marker_box.pose.orientation.w = q_plan.w();
  marker_box.scale.x = depth_plan;
  marker_box.scale.y = width_plan;
  marker_box.scale.z =height_plan;
  marker_box.color.a = 0.5; // Don't forget to set the alpha!
  marker_box.color.r = float(pow(chi_plan, 2.0) / dof_plan);
  marker_box.color.g = float(pow(chi_plan, 2.0) / dof_plan);
  marker_box.color.b = 1-(float)(pow(chi_plan, 2.0) / dof_plan);
  marker_box.lifetime = ros::Duration(0.2);
  // marker_pub_.publish(marker_box);
  // ROS_INFO( "Index Plan Fitting  = %g\n",1.0 - chi_plan/chi0_plan);
  // ROS_INFO("Fitting plane DONE"); 
 
  double v_box(height_plan*depth_plan*width_plan);
  //ROS_INFO("Box volume = %f",(float)(v_box));
  //ROS_INFO("Fitting Plane DONE \n");
  // Pubblishing marker
  
  bool b_sph,b_cyl,b_ell,b_box;
  // Sphere

  grasp_shape.time_now=ros::Time::now();
  double r_sphere = FIT(3);
  if( (v_sph<v_box)  && 
        (v_sph<v_cyl)   &&
        (v_sph<v_ell)   )
  {
  ROS_INFO("Grasp Shape: Sphere");  
  ROS_INFO("Sphere Center: %f, %f, %f",FIT(0),FIT(1),FIT(2));
  ROS_INFO("Sphere Radius: %f",FIT(3));
  ROS_INFO("Sphere Volume: %f\n",v_sph);
  ROS_INFO ("Sphere Fitting Result: ");
  ROS_INFO ( "x_0  = %.5f +/- %.5f", FIT(0), c*ERR(0));
  ROS_INFO ( "y_0  = %.5f +/- %.5f", FIT(1), c*ERR(1));
  ROS_INFO ( "z_0  = %.5f +/- %.5f", FIT(2), c*ERR(2));
  ROS_INFO ( "r    = %.5f +/- %.5f", FIT(3), c*ERR(3));
  ROS_INFO( "Index Reconstruction Sphere: %g\n", pow(chi, 2.0) / dof );//

  marker_pub_.publish( marker_sph );
  position_msgs.point.x=FIT(0);
  position_msgs.point.y=FIT(1);
  position_msgs.point.z=FIT(2);
  position_msgs.header.seq=0;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  pub_hand_link_position_.publish(position_msgs);
  
  b_sph=true;
  
  grasp_shape.x_0=FIT(0);
  grasp_shape.y_0=FIT(1);
  grasp_shape.z_0=FIT(2);
  grasp_shape.r_1=FIT(3);
  grasp_shape.v=v_sph;
  grasp_shape.I=pow(chi, 2.0) / dof;
  grasp_shape.grasp_shape="sphere";
  this->grasp_shape_estimate_now=grasp_shape;
  }
  

  //Ellissoid
  if( (v_ell<v_box) &&
        (v_ell<v_cyl) &&
        (v_ell<v_sph) )
  {
  ROS_INFO("Grasp Shape: ELLISSOID"); 
  ROS_INFO(" Ellissoid Center: %f, %f, %f",FIT_ell(0),FIT_ell(1),FIT_ell(2));
  ROS_INFO(" Ellissoid r1 = %f,r2 = %f ,r3 = %f",FIT_ell(3),FIT_ell(4),FIT_ell(5));
  ROS_INFO("Ellissoid Volume: %f",v_ell);
  ROS_INFO ("Ellissoid Fitting Result");
  ROS_INFO ( "x_0   = %.5f +/- %.5f", FIT_ell(0), c*ERR_ell(0));
  ROS_INFO ( "y_0   = %.5f +/- %.5f", FIT_ell(1), c*ERR_ell(1));
  ROS_INFO ( "z_0   = %.5f +/- %.5f", FIT_ell(2), c*ERR_ell(2));
  ROS_INFO ( "r1    = %.5f +/- %.5f", FIT_ell(3), c*ERR_ell(3));
  ROS_INFO ( "r2    = %.5f +/- %.5f", FIT_ell(4), c*ERR_ell(4));
  ROS_INFO ( "r3    = %.5f +/- %.5f", FIT_ell(5), c*ERR_ell(5));
  ROS_INFO( "Index Reconstruction Ellissoid: %g\n",  pow(chi_ell, 2.0) / dof_ell);
  
  marker_pub_.publish(marker_ell);
  position_msgs.point.x=FIT_ell(0);
  position_msgs.point.y=FIT_ell(1);
  position_msgs.point.z=FIT_ell(2);
  position_msgs.header.seq=1;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  pub_hand_link_position_.publish(position_msgs);
  
  b_ell=true;
  
  grasp_shape.x_0=FIT_ell(0);
  grasp_shape.y_0=FIT_ell(1);
  grasp_shape.z_0=FIT_ell(2);
  grasp_shape.r_1=FIT_ell(3);
  grasp_shape.r_2=FIT_ell(4);
  grasp_shape.r_3=FIT_ell(5);
  grasp_shape.v=v_ell;
  grasp_shape.I=pow(chi_ell, 2.0) / dof_ell;
  grasp_shape.grasp_shape="ellissoid";
  this->grasp_shape_estimate_now=grasp_shape;
  }

  //Cylinder 
  if( (v_cyl<v_box) &&
      (v_cyl<v_ell) &&
      (v_cyl<v_sph))     
  {
  ROS_INFO("Grasp Shape: Cylinder");
  ROS_INFO("Cylinder Center: %f, %f, %f", FIT_cyl(0),center_y, FIT_cyl(1));
  ROS_INFO("Cylender Radius_1 , Radius_2 and Height: %f, %f, %f",FIT_cyl(2),FIT_cyl(3),height);
  ROS_INFO("Cylinder Volume: %f",v_cyl);
  ROS_INFO ("Cylinder Fitting Result");
  ROS_INFO ( "x_0   = %.5f +/- %.5f", FIT_cyl(0), c_cyl*ERR_cyl(0));
  ROS_INFO ( "z_0   = %.5f +/- %.5f", FIT_cyl(1), c_cyl*ERR_cyl(1));
  ROS_INFO ( "r1    = %.5f +/- %.5f", FIT_cyl(2), c_cyl*ERR_cyl(2));
  ROS_INFO ( "r2    = %.5f +/- %.5f", FIT_cyl(3), c_cyl*ERR_cyl(3));
  ROS_INFO ("\n");
  ROS_INFO( "Index Reconstruction Cylinder: %g\n",  pow(chi_cyl, 2.0) / dof_cyl);
  marker_pub_.publish(marker_cyl);
  position_msgs.point.x=FIT_cyl(0);
  position_msgs.point.y=center_y;
  position_msgs.point.z=FIT_cyl(1);
  position_msgs.header.seq=2;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  pub_hand_link_position_.publish(position_msgs);
  
  b_cyl=true;
  
  grasp_shape.x_0=FIT_cyl(0);
  grasp_shape.y_0=center_y;
  grasp_shape.z_0=FIT_cyl(1);
  grasp_shape.r_1=FIT_cyl(2);
  grasp_shape.r_2=FIT_cyl(3);
  grasp_shape.h=height;
  grasp_shape.v=v_cyl;
  grasp_shape.I=pow(chi_cyl, 2.0) / dof_cyl;
  grasp_shape.grasp_shape="cylinder";
  this->grasp_shape_estimate_now=grasp_shape;
  }

  //Plane 
  if((v_box<v_cyl) &&
     (v_box<v_ell)&&
     (v_box<v_sph))    
  {
  ROS_INFO("Grasp Shape: Plane");
  //ROS_INFO("Plane Center: %f, %f, %f", center_x, center_y,center_z);
  ROS_INFO("Plane Center: %f, %f, %f", FIT_plan(4), FIT_plan(5),FIT_plan(6));
  ROS_INFO("Plane Coef: %f, %f, %f , %f",  FIT_plan(0), FIT_plan(1), FIT_plan(2), FIT_plan(3));
  ROS_INFO("Plane Width, Height and Depth: %f, %f, %f",width_plan,height_plan,depth_plan);
  ROS_INFO("Box Volume: %f\n",v_box);
  ROS_INFO ("Plane Fitting Result:"); 
  ROS_INFO ( "x_0   = %.5f +/- %.5f", FIT_plan(4), c_plan*ERR_plan(4));
  ROS_INFO ( "y_0   = %.5f +/- %.5f", FIT_plan(5), c_plan*ERR_plan(5));
  ROS_INFO ( "z_0   = %.5f +/- %.5f", FIT_plan(6), c_plan*ERR_plan(6));
  ROS_INFO ( "a     = %.5f +/- %.5f", FIT_plan(0), c_plan*ERR_plan(0));
  ROS_INFO ( "b     = %.5f +/- %.5f", FIT_plan(1), c_plan*ERR_plan(1));
  ROS_INFO ( "c     = %.5f +/- %.5f", FIT_plan(2), c_plan*ERR_plan(2));
  ROS_INFO ( "d     = %.5f +/- %.5f", FIT_plan(3), c_plan*ERR_plan(3));
  ROS_INFO( "Index Reconstruction: %g\n",  pow(chi_plan, 2.0) / dof_plan);
  
  marker_pub_.publish(marker_box);
  position_msgs.point.x= FIT_plan(4);
  position_msgs.point.y= FIT_plan(5);
  position_msgs.point.z= FIT_plan(6);
  position_msgs.header.seq=3;
  position_msgs.header.stamp=ros::Time::now();
  position_msgs.header.frame_id="left_arm_7_link";
  pub_hand_link_position_.publish(position_msgs);
  
  b_box=true;
  
  grasp_shape.x_0=FIT_plan(4);
  grasp_shape.y_0=FIT_plan(5);
  grasp_shape.z_0=FIT_plan(6);
  grasp_shape.a=FIT_cyl(0);
  grasp_shape.b=FIT_cyl(1);
  grasp_shape.c=FIT_cyl(2);
  grasp_shape.d=FIT_cyl(3);
  grasp_shape.w=width_plan;
  grasp_shape.h=height_plan;
  grasp_shape.de=depth_plan;
  grasp_shape.v=v_box;
  grasp_shape.I=pow(chi_plan, 2.0) / dof_plan;
  grasp_shape.grasp_shape="plane";
  this->grasp_shape_estimate_now=grasp_shape;
  }
  
  if((b_box||b_cyl||b_sph||b_ell)==false)
  {
    ROS_INFO("Shape Estimator Failed to fit Grasp");
  }  


  //ROS_INFO(">--------Grasp Shape Estimator END----------<\n");
  }
 }
 else{ROS_ERROR("No data");}
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
  ros::AsyncSpinner spinner(4); 
  grasp_estimator::Grasp_Shape_Estimator_Server Grasp_Shape_Estimator_Server(nh);
    
    ROS_INFO("Grasp_Shape_Estimator_Server is Here !!!");
    //int index=(int)0;
  //ros::Rate loop_rate(Hz);
  
  spinner.start();
  while (ros::ok())
  {

    Grasp_Shape_Estimator_Server.publishDataState();

    //ros::spinOnce();
    
    //ROS_INFO("Data Published !");
    //loop_rate.sleep();
  }
  spinner.stop();

  return 0;
}
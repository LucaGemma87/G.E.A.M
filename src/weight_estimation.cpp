

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

// Pseudo inverse
//#include <utils/pseudo_inversion.h>

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
      void publishDataState();
      static int center_f(const gsl_vector *x, void *data, gsl_vector *f);
    
      int index; 

      double hand_weight; 

      tf::Vector3 hand_gravity_center;

	// constructor
	Weight_Estimator_Server(ros::NodeHandle nh) : 
		nh_(nh), 
        priv_nh_("~")
	{
                     
        msg_grasp_state_now_ = nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),1,
                                                                                     &Weight_Estimator_Server::weight_estimation_server,
                                                                                     this);
        
		// advertise topics
		pub_weight_states_ = nh_.advertise<grasp_estimator::WeightEstimated>(nh_.resolveName("/left_hand/mass_estimator_server_"), 1);
	
		pub_cog_ = nh_.advertise<geometry_msgs::PointStamped>(nh_.resolveName("/left_hand/cog_server"), 1);


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
	 this->msg_data_acquired_recent=*msg_data_acquired_now; 
}

void Weight_Estimator_Server::publishDataState()
{ 
  double weight_measured;  
  if(msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x!=0&&msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y!=0&&msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z!=0&&msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x!=0&&msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y!=0&&msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z!=0)
  { 
    ROS_INFO("Calibrating Hand");
    if(msg_data_acquired_recent.hand_synergy_joint_state_position<=0.16)
      { 
        
      


           int index_2;
       if (this->index<=N-1)
        {
          index_2=this->index;
        } 
       else
        { 
         index_2=0;
        }

       this->msg_data_acquired_array[index_2]=this->msg_data_acquired_recent;
    
    //ROS_INFO("Index %d",this->index);
    tf::StampedTransform vito_anchor_2_left_ft_sensor;
    transformStampedMsgToTF(this->msg_data_acquired_recent.vito_anchor_2_arm_7_link,vito_anchor_2_left_ft_sensor);
    tf::Vector3 position = vito_anchor_2_left_ft_sensor.getOrigin();
    //ROS_INFO("Position: %f, %f, %f ",(float)position[0],(float)position[1],(float)position[2]); 
    tf::Quaternion quaternion = vito_anchor_2_left_ft_sensor.getRotation();
    tf::Matrix3x3 ft_rotation_matrix(quaternion);
    tf::Vector3 force(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.force.z);
    tf::Vector3 torque(this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.x,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.y,this->msg_data_acquired_recent.arm_wrench_stamped.wrench.torque.z);
    tf::Vector3 gravity(0,0,-9.81);
    tf::Vector3 gravity_ft_frame=ft_rotation_matrix.transpose()*gravity;
    //ROS_INFO("gravity_ft_frame %f %f %f",(float)gravity_ft_frame[0],(float)gravity_ft_frame[1],(float)gravity_ft_frame[2]);   
    //tf::Vector3 weight_3d=ft_rotation_matrix*force;
    //double gravity_norm(sqrt(pow(force[0],2)+pow(force[1],2)+pow(force[2],2)));
    weight_measured=(sqrt(pow(force[0]/gravity_ft_frame[0],2)+pow(force[1]/gravity_ft_frame[1],2)+pow(force[2]/gravity_ft_frame[2],2)));
    //double weight_measured(sqrt(pow(weight_3d[0],2)+pow(weight_3d[1],2)+pow(weight_3d[2],2)));
    //double weight_measured(abs(weight_3d[2]));
    if(this->index==0){ROS_INFO("Estimating Hand Weight");}
    //ROS_INFO("weight_measured %d = %f",(int)this->index,(float)weight_measured); 

    // | 0   -vz  vy|
    // | vz   0  -vx|
    // |-vy   vx  0 |
    // tf::Matrix3x3 skew_matrix_gravity(0, -gravity_ft_frame[2], gravity_ft_frame[1], gravity_ft_frame[2],0, -gravity_ft_frame[0], -gravity_ft_frame[1], gravity_ft_frame[0], 0);
    
    // // // questa sembra buona !!
    // //tf::Matrix3x3 skew_matrix_gravity(0, gravity_ft_frame[2], -gravity_ft_frame[1], -gravity_ft_frame[2],0, gravity_ft_frame[2], gravity_ft_frame[1], -gravity_ft_frame[2], 0);
    
    // tf::Matrix3x3 matrix_transpose(skew_matrix_gravity.transpose());
    
    // // tf::Vector3  mt_col_1(matrix_transpose.getColumn(0));
    // // ROS_INFO("mt_col_1 %f,%f,%f",(float)mt_col_1[0],(float)mt_col_1[1],(float)mt_col_1[2]);
   
    // // tf::Vector3  mt_col_2(matrix_transpose.getColumn(1));
    // // ROS_INFO("mt_col_2 %f,%f,%f",(float)mt_col_2[0],(float)mt_col_2[1],(float)mt_col_2[2]);

    // // tf::Vector3  mt_col_3(matrix_transpose.getColumn(2));
    // // ROS_INFO("mt_col_3 %f,%f,%f",(float)mt_col_3[0],(float)mt_col_3[1],(float)mt_col_3[2]);
   
    // tf::Matrix3x3 matrix_m(matrix_transpose*skew_matrix_gravity);

    // // tf::Vector3  mm_col_1(matrix_m.getColumn(0));
    // // ROS_INFO("mm_col_1 %f,%f,%f",(float)mm_col_1[0],(float)mm_col_1[1],(float)mm_col_1[2]);
   
    // // tf::Vector3  mm_col_2(matrix_m.getColumn(1));
    // // ROS_INFO("mm_col_2 %f,%f,%f",(float)mm_col_2[0],(float)mm_col_2[1],(float)mm_col_2[2]);

    // // tf::Vector3  mm_col_3(matrix_m.getColumn(2));
    // // ROS_INFO("mm_col_3 %f,%f,%f",(float)mm_col_3[0],(float)mm_col_3[1],(float)mm_col_3[2]);


    // tf::Matrix3x3 matrix_inverse(matrix_m.inverse());

    // double det_mat(matrix_inverse.determinant());

    // ROS_INFO("Det matrix = %f",(float)det_mat);
    
    // tf::Vector3 center(matrix_inverse*matrix_transpose*(torque));
    // ROS_INFO("Center: %f,%f,%f",center[0],center[1],center[2]);

    // tf::Vector3 gravity_center;

    if(this->index<N)
    {  
      
      this->index=this->index+1;
      this->hand_weight+=weight_measured;
      weight_estimate_now.weight=weight_measured;
      weight_estimate_now.time_now=ros::Time::now();
      

       // gravity_center[0]=(1/weight_measured)*center[0];
       // gravity_center[1]=(1/weight_measured)*center[1];
       // gravity_center[2]=(1/weight_measured)*center[2];
   

     // //gravity_center=ft_rotation_matrix*gravity_center;      
     // ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);
      
      // tf::Vector3 gravity_center_2_vito_anchor(ft_rotation_matrix.transpose()*gravity_center);
      // ROS_INFO("gravity_center_2_vito_anchor %f %f %f",(float)gravity_center_2_vito_anchor[0],(float)gravity_center_2_vito_anchor[1],(float)gravity_center_2_vito_anchor[2]);
      
      if(this->index==N)
      { 
       
       this->hand_weight=this->hand_weight/(N);
       weight_estimate_now.weight=this->hand_weight;
       weight_estimate_now.time_now=ros::Time::now();
       
       this->index=this->index+1;
       ROS_INFO("index %d",(int)(this->index));  
       ROS_INFO("Hand Weight = %f",(float)this->hand_weight);
       // gravity_center[0]=(1/this->hand_weight)*center[0];
       // gravity_center[1]=(1/this->hand_weight)*center[1];
       // gravity_center[2]=(1/this->hand_weight)*center[2];
      
       // this->hand_gravity_center=gravity_center;
      
       // ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);
      
       // tf::Vector3 gravity_center_2_vito_anchor(ft_rotation_matrix.transpose()*gravity_center);
       // ROS_INFO("gravity_center_2_vito_anchor %f %f %f",(float)gravity_center_2_vito_anchor[0],(float)gravity_center_2_vito_anchor[1],(float)gravity_center_2_vito_anchor[2]);
      
       weight_estimate_now.act=(int)1;
       weight_estimate_now.time_now=ros::Time::now();

       ROS_INFO("Try to solve the problem"); 
       ROS_INFO("Cog Estimator GSl initialization");
          const gsl_multifit_fdfsolver_type *T = gsl_multifit_fdfsolver_lmsder;
          gsl_multifit_fdfsolver *s;
          int status, info;
          size_t i;
          const size_t n = N;
          const size_t p = 3;
          // init cog
          gsl_matrix *J = gsl_matrix_alloc(n, p);
          gsl_matrix *covar = gsl_matrix_alloc (p, p);

          double force_x[N],force_y[N],force_z[N],torque_x[N],torque_y[N],torque_z[N], weights[N];
          for (int i = 0; i < N; i++)
          {
           force_x[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.x;
           force_y[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.y;
           force_z[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.force.z;

           torque_x[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.x;
           torque_y[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.y;
           torque_z[i]=this->msg_data_acquired_array[i].arm_wrench_stamped.wrench.torque.z;

           //weights[i] = 1.0 / (0.5 * 0.5)
           weights[i] = 1.0 / (1 * 1);
          }
          struct data data_estim = { n,force_x,force_y,force_z,torque_x,torque_y,torque_z};
          gsl_multifit_function_fdf f;

          ////////////////////////////////////////////////////////////////////////////////////////////////////////////
          ////////////////////////////////////// GSL cog              ////////////////////////////////////////////////
          ////////////////////////////////////////////////////////////////////////////////////////////////////////////
          ROS_INFO("Estimating Cog");
          //double x_init[p] = { 0.0, 0.0,0.18};
          double x_init[p] = { 0.0, 0.0,0.18};
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
          f.f =&center_f;

          f.df =NULL;
          //f.df =&expb_df;   /* set to NULL for finite-difference Jacobian */
          f.n = n;
          f.p = p;
          f.params = &data_estim;

          //ROS_INFO("Data created");
          s = gsl_multifit_fdfsolver_alloc(T, n, p);
          //ROS_INFO("Alloc Done");
           //initialize solver with starting point and weights 
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
           ROS_INFO( "initial |f(x)| = %g\n", chi0);
           ROS_INFO( "final   |f(x)| = %g\n", chi);
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
           

          // ROS_INFO ( "status = %s\n", gsl_strerror (status));
          geometry_msgs::PointStamped cog_msgs;
          cog_msgs.point.x=FIT(0);
          cog_msgs.point.y=FIT(1);
          cog_msgs.point.z=FIT(2);
          cog_msgs.header.seq=1;
          cog_msgs.header.stamp=ros::Time::now();
          cog_msgs.header.frame_id="left_measure";
          pub_cog_.publish(cog_msgs);



          ROS_INFO("Estimating cog DONE");

      }
     }
    }


    if(this->index>=N)
    {
     ROS_INFO("Put Object on the hand"); 
     this->index=this->index+1; 
     double weight_object = weight_measured-this->hand_weight;
     ROS_INFO("Diff weight hand %f",(float)weight_object) ;
     if(abs(weight_object)<=0.2*(this->hand_weight))
     { 
      // gravity_center[0]=(1/this->hand_weight)*center[0];
      // gravity_center[1]=(1/this->hand_weight)*center[1];
      // gravity_center[2]=(1/this->hand_weight)*center[2];
      weight_estimate_now.weight=weight_measured;
      ROS_INFO("No object in the hand. weight of the hand: %f",(float)weight_measured);  
      //ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);
     }
     else
     { 
       this->index=this->index+1;
       ROS_INFO("Object in the hand: weight without the hand = %f",(float)weight_object);
       weight_estimate_now.weight=weight_object;
       weight_estimate_now.time_now=ros::Time::now();

       // gravity_center[0]=(1/weight_measured)*center[0];
       // gravity_center[1]=(1/weight_measured)*center[1];
       // gravity_center[2]=(1/weight_measured)*center[2];
     
       //ROS_INFO("gravity_center %f %f %f",(float)gravity_center[0],(float)gravity_center[1],(float)gravity_center[2]);


       weight_estimate_now.act=(int)1;
       weight_estimate_now.time_now=ros::Time::now();
      }
    }

      // geometry_msgs::PointStamped cog_msgs;
      // cog_msgs.point.x=gravity_center[0];
      // cog_msgs.point.y=gravity_center[1];
      // cog_msgs.point.z=gravity_center[2];
      // cog_msgs.header.seq=1;
      // cog_msgs.header.stamp=ros::Time::now();
      // cog_msgs.header.frame_id="left_measure";
      // pub_cog_.publish(cog_msgs);
      //ROS_INFO("Estimating cog DONE");
    
   
    pub_weight_states_.publish(weight_estimate_now);
  }

}

}//namespace grasp_estimator

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Weight_Estimator_Server");
    //grasp_estimator::WeightEstimated::ConstPtr weight_estimate_now;
	ros::NodeHandle nh;
    //tf::TransformStamped msgs_tf;
	grasp_estimator::Weight_Estimator_Server Weight_Estimator_Server(nh);

  Weight_Estimator_Server.index = 0;
  Weight_Estimator_Server.hand_weight=0;
  int interrupt = 0;
    
  ROS_INFO("Weight_Estimator_Server is Here !!!");
    
	ros::Rate loop_rate(20);

	while (ros::ok())
	{

		Weight_Estimator_Server.publishDataState();
		//ROS_INFO("msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x: %f ",(float)msg_data_acquired_recent.arm_wrench_stamped.wrench.force.x); 
        //
		ros::spinOnce();

    //if(Weight_Estimator_Server.index==N-1){interrupt=1;} 
      

		//ROS_INFO("Data Published !");
		loop_rate.sleep();
	}

	return 0;
}
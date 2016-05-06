

// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <ios>
#include <iostream>
#include <tf/transform_listener.h>
// ROS point cloud

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>


// message
#include <visualization_msgs/Marker.h>
#include "grasp_estimator/WeightEstimated.h"
#include "grasp_estimator/VolumeEstimated.h"

#define NO_STATIC_UNLINE
#define Hz 10
namespace grasp_estimator
{

class Volume_Estimator_Server
{
private:
	// suggested members, note the member_name_ naming convention

	// the node handle
	ros::NodeHandle nh_;

	// node handle in the private namespace
	ros::NodeHandle priv_nh_;
         //! A tf transform listener
        //tf::TransformListener listener_;

	// publishers
	ros::Publisher pub_volume_states_;

        
	ros::Publisher pub_volume_shape_convex_;
        // subscribers
        ros::Subscriber msg_cloud_now_;
	
	ros::Subscriber weight_estimated_now_;
	
	tf::TransformListener listener_;
	//pcl::PointCloud<pcl::PointXYZRGB> crop_cloud;
	
public:
      // callback functions
      void take_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
      void take_mass(const grasp_estimator::WeightEstimated::ConstPtr &mass_now);
      void volume_estimation();
      void publishDataState();
      
      pcl::PCLPointCloud2 pcl_pc2;
      pcl::PointCloud<pcl::PointXYZRGBA> cloud_in;
      
      grasp_estimator::VolumeEstimated volume_estimate_now;
      
      grasp_estimator::WeightEstimated weight_in;
      
      tf::StampedTransform left_palm_hand_dummy_link_tf_stamp;
      //double weight_now;
      double mass_now;
     
      double convex_volume_now;
      
      double convex_density_now;
      
      double convex_area_now;
      
      bool received;
	// constructor
      Volume_Estimator_Server(ros::NodeHandle nh) : 
      nh_(nh), 
      priv_nh_("~")
     {                 
       msg_cloud_now_= nh_.subscribe("/pacman_vision/processed_scene",1, &Volume_Estimator_Server::take_cloud, this);
       weight_estimated_now_ = nh_.subscribe("/left_hand/mass_estimator_server_",1,&Volume_Estimator_Server::take_mass,this);         
       // advertise topics
       pub_volume_states_ = nh_.advertise<grasp_estimator::VolumeEstimated>(nh_.resolveName("/left_hand/volume_estimator_server_"), 5);
     
       pub_volume_shape_convex_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("/left_hand/shape_convex_server_"),5);
  
    
      }

	//! Empty stub
	~Volume_Estimator_Server() {}
 

};      


void Volume_Estimator_Server::take_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{  //this->mass_now=0.5;  
   // if(this->mass_now!=0.0)
  {
    //pub_volume_shape_convex_.publish(msg);
    this->received=false;
    //ROS_INFO("Acquiring point cloud");
    pcl::fromROSMsg(*msg,cloud_in);
//     ROS_INFO("Converting point cloud");
//     pcl::fromPCLPointCloud2(pcl_pc2,cloud_in);
    //ROS_INFO("Converting point cloud DONE");
    this->received=true;
  }
}

void Volume_Estimator_Server::take_mass(const grasp_estimator::WeightEstimated::ConstPtr &mass_now)
{
   this->mass_now = mass_now->mass;
   //ROS_INFO("Mass now = %g",(double)this->mass_now);
}

void Volume_Estimator_Server::volume_estimation()
{  //this->mass_now=0.5;  
  //ROS_INFO("Starting volume estimation");
  if(this->mass_now!=0.0&& this->received==true ) //&&cloud_in.is_dense
  //if(this->received==true)
   { 
    //ROS_INFO("Found object on the hand. Try to estimate volume");
    
       ros::Time now1 = ros::Time(0);
//   // left_hand_palm_dummy_link
   listener_.waitForTransform("left_arm_palm_dummy_link","/head_asus/camera_rgb_optical_frame",now1, ros::Duration(1));
   listener_.lookupTransform("left_arm_palm_dummy_link","/head_asus/camera_rgb_optical_frame",now1, this->left_palm_hand_dummy_link_tf_stamp);
//   
     ROS_INFO("Tf Acquired");
     pcl::CropBox<pcl::PointXYZRGBA> crop_box(false);
     crop_box.setNegative(true);
     crop_box.setKeepOrganized(true);
     
     crop_box.setInputCloud(cloud_in.makeShared());
     
      Eigen::Vector4f max_box(0.3,0.15,0.15,1.0);max_box.resize(4);
      crop_box.setMax(max_box);
      Eigen::Vector4f min_box(0.0,-0.15,-0.15,1.0);min_box.resize(4);
      crop_box.setMin(min_box);
     tf::Vector3 pos(this->left_palm_hand_dummy_link_tf_stamp.getOrigin());
     Eigen::Vector3f translation_box(pos[0],pos[1],pos[2]); //translation_box.resize(3);
     crop_box.setTranslation(translation_box);
//    
     tf::Matrix3x3 rot_matrix_box(this->left_palm_hand_dummy_link_tf_stamp.getBasis());
      tfScalar roll_box;
      tfScalar pitch_box;
      tfScalar yaw_box;
//      
      rot_matrix_box.getRPY(roll_box,pitch_box,yaw_box);
//    
      Eigen::Vector3f rotation_box(roll_box,pitch_box,yaw_box);//rotation_box.resize(3);
     crop_box.setRotation(rotation_box);
//     
     pcl::PointCloud<pcl::PointXYZRGBA> cloud_cropped;
     crop_box.filter(cloud_cropped); 	
    
    pcl::ConvexHull<pcl::PointXYZRGBA> convex_hull;
     std::vector<pcl::Vertices> polygons;
  
     convex_hull.setInputCloud(cloud_cropped.makeShared());
     convex_hull.setDimension((int)3);
     convex_hull.setComputeAreaVolume(true);
     //pcl::PointCloud<pcl::PointXYZRGB> points;
      ROS_INFO("Computing a convex hull for all points given");
      pcl::PointCloud<pcl::PointXYZRGBA> cloud_out;
      convex_hull.reconstruct(cloud_out,polygons);  // <------   there is the error ,polygons
      ROS_INFO("Computed a convex hull for all points given");
      
      ROS_INFO("Cloud out width  :  %d",cloud_out.width);
      ROS_INFO("Cloud out height : %d",cloud_out.height);
     ROS_INFO("Filling points");
     //Returns the total area of the convex hull.
     this->convex_area_now=convex_hull.getTotalArea();
//     //Returns the total volume of the convex hull. 
    this->convex_volume_now=convex_hull.getTotalVolume();
//   
     if(this->convex_volume_now!=0.0&&this->mass_now!=0.0){
       this->convex_density_now=(this->mass_now)/(this->convex_volume_now);
     }
     this->volume_estimate_now.convex_volume=this->convex_volume_now;
     this->volume_estimate_now.convex_area=this->convex_area_now;
     this->volume_estimate_now.convex_density=this->convex_density_now; 
     
     
       visualization_msgs::Marker marker;
      	 marker.header.frame_id = "/head_asus/camera_rgb_optical_frame";
         marker.header.stamp = ros::Time();
        marker.ns = "grasp_estimator";
        marker.id = 0;
      
        marker.type = visualization_msgs::Marker::LINE_STRIP;  

	marker.points.resize(cloud_out.size());
	
        marker.color.a = 0.5;
        marker.color.b = 0.2f;
        marker.color.g = 1.0f;
        marker.color.r = 0.0f;
	marker.scale.x = 0.002;
        marker.scale.y = 0.002;
        marker.scale.z = 0.002;
       for (size_t i = 0; i < cloud_out.size(); ++i)
        {
 	marker.points[i].x=cloud_out.points[i].x;
        marker.points[i].y=cloud_out.points[i].y;
 	marker.points[i].z=cloud_out.points[i].z;
	}
       marker.lifetime = ros::Duration(2.0);
	
	pub_volume_shape_convex_.publish(marker);
     
     //PCA pca() 
     pcl::PCA<pcl::PointXYZRGBA> pca(cloud_out,true);
     pcl::PointCloud<pcl::PointXYZRGBA> cloud_projection;
   
     Eigen::Vector4f mean(pca.getMean());
     Eigen::Vector3f eigenvalues(pca.getEigenValues());
     Eigen::MatrixXf matrix(pca.getCoefficients());
     Eigen::Matrix3f rotation(pca.getEigenVectors()); 
     
     
     //ROS_INFO("rotation size = %d", rotation.size());
     rotation.resize(3,3);
     //ROS_INFO("MAtrix size %d ", matrix.size());
     tf::Matrix3x3 rot_matrix;
     //
     rot_matrix.setValue(rotation(0,0),rotation(0,1),rotation(0,2),rotation(1,0),rotation(1,1),rotation(1,2),rotation(2,0),rotation(2,1),rotation(2,2));
       	
     tfScalar roll;
     tfScalar pitch;
     tfScalar yaw;
     
     rot_matrix.getRPY(roll,pitch,yaw);
     tf::Quaternion q;
     q.setRPY(roll,pitch,yaw);
     
     
     ROS_INFO("Mean 0 = %f",(float)mean(0));
     ROS_INFO("Mean 1 = %f",(float)mean(1));
     ROS_INFO("Mean 2 = %f",(float)mean(2));
     ROS_INFO("Mean 3 = %f",(float)mean(3));
     

     
     double mod(sqrt(pow(eigenvalues(0),2)+pow(eigenvalues(1),2)+pow(eigenvalues(2),2)));
     
     ROS_INFO("Height = %f",(float)eigenvalues(0)/mod);
     ROS_INFO("Widht = %f",(float)eigenvalues(1)/mod);
     ROS_INFO("Depth = %f",(float)eigenvalues(2)/mod);
     
     this->volume_estimate_now.height=eigenvalues(0)/mod;
     this->volume_estimate_now.width=eigenvalues(1)/mod;
     this->volume_estimate_now.depth=eigenvalues(2)/mod;
     
     this->volume_estimate_now.mean_0=mean(0);
     this->volume_estimate_now.mean_1=mean(1);
     this->volume_estimate_now.mean_2=mean(2);
     this->volume_estimate_now.mean_3=mean(3);
     
    
	
	
	visualization_msgs::Marker marker_PCA;
      	 marker_PCA.header.frame_id = "/head_asus/camera_rgb_optical_frame";
         marker_PCA.header.stamp = ros::Time();
        marker_PCA.ns = "grasp_estimator";
        marker_PCA.id = 1;
      
        marker_PCA.type = visualization_msgs::Marker::SPHERE;  

	marker_PCA.pose.position.x= mean(0);
        marker_PCA.pose.position.y=mean(1);
        marker_PCA.pose.position.z=mean(2);
        marker_PCA.pose.orientation.x=q.x();
        marker_PCA.pose.orientation.y=q.y();
        marker_PCA.pose.orientation.z=q.z();
        marker_PCA.pose.orientation.w=q.w();
        marker_PCA.color.a = 0.5;
        marker_PCA.color.b = 1.0f;
        marker_PCA.color.g = 0.0f;
        marker_PCA.color.r = 1.0f;
	marker_PCA.scale.x = eigenvalues(0)/mod;
        marker_PCA.scale.y = eigenvalues(1)/mod;
        marker_PCA.scale.z = eigenvalues(2)/mod;
        marker_PCA.lifetime = ros::Duration(2.0);
	
	pub_volume_shape_convex_.publish(marker_PCA);
        ROS_INFO("Filled points and pub");
      
  }
   else
   { 
      //ROS_INFO("No object on the hand");  
      this->volume_estimate_now.convex_volume=0.0;
      this->volume_estimate_now.convex_area=0.0;
      this->volume_estimate_now.convex_density=0.0; 
      
   }
}


void Volume_Estimator_Server::publishDataState()
{ 
  this->volume_estimate_now.time_now=ros::Time::now();
  pub_volume_states_.publish(volume_estimate_now);
}

}//namespace grasp_estimator

int main(int argc, char **argv)
{ 
  ROS_INFO("Volume_Estimator_Server is Here !!!");
  
  ros::init(argc, argv, "Volume_Estimator_Server");
    
  ros::NodeHandle nh;

  ros::Rate loop_rate(Hz);
  ros::AsyncSpinner spinner(4);
  grasp_estimator::Volume_Estimator_Server Volume_Estimator_Server(nh);
  Volume_Estimator_Server.received=false;
  Volume_Estimator_Server.mass_now=0.0;
   Volume_Estimator_Server.volume_estimate_now.convex_volume=0.0;
    Volume_Estimator_Server.volume_estimate_now.convex_area=0.0;
     Volume_Estimator_Server.volume_estimate_now.convex_density=0.0;  
     
   spinner.start();
   
  
    while (ros::ok())
	 {
	   //if(Volume_Estimator_Server.mass_now!=0.0)
	   {  
	   Volume_Estimator_Server.volume_estimation();
	   }
	   Volume_Estimator_Server.publishDataState();
	   loop_rate.sleep();
	 }
  //ros::spin();	 
  spinner.stop();
  return 0;
}
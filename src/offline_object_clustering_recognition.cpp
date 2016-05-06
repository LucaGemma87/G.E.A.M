

  // Author(s): Luca Gemma

  #include <iostream>
  #include <fstream>
  // #include <ostream>
  #include <sstream>
  #include <sstream>
  #include <stdlib.h>
  #include <ios>


  #include <tf/transform_listener.h>
  #include <tf/transform_broadcaster.h>
  #include <tf/transform_datatypes.h>

  // ROS headers
  #include <ros/ros.h>



  // Message of estimator node
  #include "grasp_estimator/GraspShape.h"
  #include "grasp_estimator/WeightEstimated.h"
  #include "grasp_estimator/VolumeEstimated.h"
  #include "grasp_estimator/DataAcquired.h"
  //Message of Object Recognition 
  #include "grasp_estimator/ObjectRecognition.h"


  // OpenCV3 K-means
  // #include <opencv2/highgui/highgui.hpp>
  // #include <opencv2/core/core.hpp>
  // #include <opencv2/imgproc/imgproc.hpp>
  // #include <boost/graph/graph_concepts.hpp>

  // Dlib Machine Learning
  #include <vector>
  #include <dlib/svm.h>
  #include <dlib/svm_threaded.h>
  #include <dlib/statistics.h>
  #include <dlib/clustering.h>
  #include <dlib/rand.h>


  // service
  #include <std_srvs/Empty.h>

  // binomial coefficent
  //#include <boost/math/special_functions/binomial.hpp>

  // experimental Talking robot
  #include <sound_play/sound_play.h>


  // FLANN Ktree

  #include <assert.h>

  #include <flann/flann.h>
  #include <flann/io/hdf5.h>

  #define Hz 100
  #define n_descriptors 50


  namespace grasp_estimator
  {
   using namespace std;
   using namespace dlib;
   using namespace flann;
  class Object_Clustering_Recognition_Server
  {
  typedef dlib::matrix<double,n_descriptors,1> sample_type;                // define the dimensions of dataset
  typedef radial_basis_kernel<sample_type> kernel_type;	// kernel type: radial basis 
  typedef one_vs_one_trainer<any_trainer<sample_type> > object_trainer;
  typedef    flann::Matrix<double> dataset;
  private:
  	// suggested members, note the member_name_ naming convention
  	// the node handle
  	ros::NodeHandle nh_;
  	// node handle in the private namespace
  	ros::NodeHandle priv_nh_;
  	// publishers
  	ros::Publisher pub_object_type_;
      // subscribers
      ros::Subscriber sub_cog_now_;
      ros::Subscriber sub_grasp_shape_now_;
  	ros::Subscriber sub_weight_now_;
  	ros::Subscriber sub_data_now_;
  	 /// declaration of service servers
      ros::ServiceServer srvServer_Start_;
      ros::ServiceServer srvServer_Stop_;
      // Type of message
      grasp_estimator::WeightEstimated weight_now;  
  	grasp_estimator::WeightEstimated cog_now; 	
      grasp_estimator::GraspShape grasp_shape_now;	
  	grasp_estimator::DataAcquired data_now;
  	
      bool grasp_shape_received;	
      bool weight_received;
  	bool cog_received;
  	bool data_received;
    
  	
  	std::string node_name;
  	std::string data_file_name;
  	std::string centroid_file_name;
  	std::string data_file_dir;
  	std::string centroid_file_dir;
  	std::string data_file_name_txt;
  	std::string data_file_name_txt_dir;
  	bool recalculate_centroid;
  	bool create_header;
  	int object_type_database;
  	int labels_type;
  	//int object_type_grasp_database; // forse anche questo dovrebbe essere 
  	std::vector<int>  object_type_grasp_database;
  	std::vector<std::string> object_name_vector;
  	 
	std::vector<std::vector<int> > exp_grasp_type_object;
  	
  	std::vector<double> centroid_vector;
  	grasp_estimator::ObjectRecognition object_recognized_now;
  	grasp_estimator::ObjectRecognition object_clustering_now; // forse andrebbe creato come un std:: vector
  	grasp_estimator::ObjectRecognition object_data;
  	
  	std::vector<sample_type> samples;
      std::vector<sample_type> initial_centers,test_centers;
       std::vector<double>labels;
      sample_type m;
      
      std::vector<std::string> object_recognition;
      
  	// not used because already included into ObjectRecongnition Message
  	//geometry_msgs::WrenchStamped wrench_now;
  	//sensor_msgs::JointState hand_joint_now;
      
      // voice
      sound_play::SoundRequest voice;
      ros::Publisher talker;
      std::vector<std::string> flann_object_recognition;
  public:
      
      int test;
      int count_data;
      bool add_data;
      bool object_recognized;
      bool test_made;
      bool run_state;
      std::string object_name;
      std::string grasp_type;
      int number_data;
      int count_centroid;

      bool make_clustering;
      bool load_database_in_flann; 

      // FLANN


      int nn;

      int num_dataset;

      boost::shared_ptr<dataset> flann_database;
      boost::shared_ptr<dataset> flann_query;
      boost::shared_ptr<Index<ChiSquareDistance<double> > >  flann_index;
      
      void take_grasp_shape(const grasp_estimator::GraspShape::ConstPtr &grasp_shape);
      void take_weight(const grasp_estimator::WeightEstimated::ConstPtr &weight);
      void take_cog(const grasp_estimator::WeightEstimated::ConstPtr &cog);
      void take_data(const grasp_estimator::DataAcquired::ConstPtr &data);
      void clustering();
      void testing();
      void publishDataObject();
      void addData();
      //void comb(int n, int r, int *arr, int sz);
      void stop_testing();
      
      
      void getROSParameters();
      void setROSParameters();

      void saveDataClustering();
      void saveDataToTxt();

      void loadDataBase();
      //void loadCentroid(); 

      bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);    
      
      // constructor
      Object_Clustering_Recognition_Server(ros::NodeHandle nh) : 
      nh_(nh), 
      priv_nh_("~")
  	{
  	  sub_cog_now_=nh_.subscribe<grasp_estimator::WeightEstimated>(nh_.resolveName("/left_hand/cog_estimator_server_"),1,&Object_Clustering_Recognition_Server::take_cog,this);
  	  
  	  sub_grasp_shape_now_= nh_.subscribe<grasp_estimator::GraspShape>(nh_.resolveName("/left_hand/grasp_shape_estimator_server_"),1,&Object_Clustering_Recognition_Server::take_grasp_shape,this);
  	  
  	  sub_weight_now_=nh_.subscribe<grasp_estimator::WeightEstimated>(nh_.resolveName("/left_hand/mass_estimator_server_"),1,&Object_Clustering_Recognition_Server::take_weight,this);
  	
  	  sub_data_now_=nh_.subscribe<grasp_estimator::DataAcquired>(nh_.resolveName("/left_hand/grasp_state_data_server_"),1,&Object_Clustering_Recognition_Server::take_data,this);
  	  
  	  pub_object_type_=nh_.advertise<grasp_estimator::ObjectRecognition>(nh_.resolveName("/left_hand/object_recognition_server_"),1);
  	  
  	  srvServer_Start_ = nh_.advertiseService("/Object_Clustering_Recognition_Server/start", &Object_Clustering_Recognition_Server::srvCallback_Start,this);

            srvServer_Stop_ = nh_.advertiseService("/Object_Clustering_Recognition_Server/stop", &Object_Clustering_Recognition_Server::srvCallback_Stop, this);
            
  	  
  	  // sound client
           talker = nh_.advertise<sound_play::SoundRequest>("/robotsound", 5);
        
  	 
  	 grasp_shape_received = false;	
        weight_received = false;
  	  cog_received = false;
  	  data_received = false;
  	  test_made = false;
      make_clustering = false;
  	  number_data=(int)0;
  	  node_name = "/offline_object_clustering_recognition_server/";
  	  run_state=false;
  	  
  	   // configure voice
           voice.command = sound_play::SoundRequest::PLAY_ONCE;
           voice.sound = sound_play::SoundRequest::SAY;

      data_now.hand_joint_curr_state.name.resize(33);
      data_now.hand_joint_curr_state.position.resize(33);
      data_now.hand_joint_curr_state.velocity.resize(33);
      data_now.hand_joint_curr_state.effort.resize(33);


      object_data.hand_joint_curr_state.name.resize(33);
      object_data.hand_joint_curr_state.position.resize(33);
      object_data.hand_joint_curr_state.velocity.resize(33);
      object_data.hand_joint_curr_state.effort.resize(33);

     object_recognition.clear();
     flann_object_recognition.clear();
     num_dataset = 0;
     nn=(int)1;
     exp_grasp_type_object.resize(3);
             flann_database.reset();
       flann_query.reset();

  	}  
      	//! Empty stub
     ~Object_Clustering_Recognition_Server()
     {
        flann_database.reset();
       flann_query.reset();
       object_recognition.clear();
     
     }
  };

  void Object_Clustering_Recognition_Server::take_grasp_shape(const grasp_estimator::GraspShape::ConstPtr &grasp_shape)
  {
   this->grasp_shape_now = *grasp_shape;
   grasp_shape_received=true;
   //ROS_INFO("Grasp Shape Received");
  }


  void Object_Clustering_Recognition_Server::take_weight(const grasp_estimator::WeightEstimated::ConstPtr &weight)
  {
   this->weight_now = *weight;
   weight_received=true;
   //ROS_INFO("Weight Received");
  }


  void Object_Clustering_Recognition_Server::take_cog(const grasp_estimator::WeightEstimated::ConstPtr &cog)
  {
   this->cog_now = *cog;
   cog_received=true;
   //ROS_INFO("Cog Received");
  }


  void Object_Clustering_Recognition_Server::take_data(const grasp_estimator::DataAcquired::ConstPtr& data)
  {
    this->data_now = *data;
    data_received = true;  
  }


  void Object_Clustering_Recognition_Server::publishDataObject()
  {
    pub_object_type_.publish<grasp_estimator::ObjectRecognition>(object_recognized_now);
  }


  bool Object_Clustering_Recognition_Server::srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  { 
    
     ROS_INFO("Plese select the mode:");
     ROS_INFO("- 1 for add data to database;");
     ROS_INFO("- 2 for testing.");
     ROS_INFO("Waiting for choose...");
     std::cin >> test;
     ROS_INFO("choose received !");
     
     if((int)test==1){
       ROS_INFO("Please insert the object name: ");
       std::cin >> object_name;
       ROS_INFO("Object Name received: %s",object_name.c_str());
       ROS_INFO("Please insert the grasp type: ");
       std::cin >> grasp_type;
       ROS_INFO("Grasp type received: %s", grasp_type.c_str());
       ROS_INFO("Please insert the labels: ");
       std::cin >> labels_type;
       ROS_INFO("Labels received: %d", (int)labels_type);
        
       setROSParameters(); 
   	    
       ROS_INFO(" Starting new acquisition service  ");
      }
     else{
      
     	ROS_INFO("Starting testing service"); 
     }
   
    
    std::string cog_service_start_name("/cog_estimator_server/start");
    std::string weight_service_start_name("/Weight_Estimator_Server/start"); 
    std::string grasp_shape_service_start_name("/Grasp_Shape_Estimator_Server/start");
    //std_srvs::Empty::Request req_;
    std_srvs::Empty srv;
    if (!ros::service::call(cog_service_start_name,srv))
     {
       ROS_ERROR("Call to cog service start failed");
       exit(0);
      }
    else 
      {
       ROS_INFO("Call to cog service start DONE");
      }
     //sleep(1.0); 
     if (!ros::service::call(weight_service_start_name, srv))
      {
       ROS_ERROR("Call to weight service start failed");
       exit(0);
      }
      else 
      {
        ROS_INFO("Call to weight service start DONE");
      }
     //sleep(1.0); 
     if (!ros::service::call(grasp_shape_service_start_name, srv))
      {
       ROS_ERROR("Call to grasp shape service start failed");
       exit(0);
      }
     else 
      {
        ROS_INFO("Call to grasp shape service start DONE");
      }  
     //sleep(1.0); 
     loadDataBase();
     ROS_INFO("Starting  Object Clustering Recognition node");
     run_state = true;
     return true;
     ROS_INFO_STREAM("Plese give me an object");
     voice.arg = "Plese give me an object";
     talker.publish(voice);
  }


  bool Object_Clustering_Recognition_Server::srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    saveDataClustering();
    
    std::string cog_service_stop_name("/cog_estimator_server/stop");
    std::string weight_service_stop_name("/Weight_Estimator_Server/stop"); 
    std::string grasp_shape_service_stop_name("/Grasp_Shape_Estimator_Server/stop");
  //   std_srvs::Empty::Request req_;
    std_srvs::Empty srv;
       nh_.getParam(node_name+"data_file_name_txt_dir", data_file_name_txt_dir);
     data_file_name_txt="data.txt";
    if (!ros::service::call(cog_service_stop_name,srv))
     {
       ROS_ERROR("Call to cog service stop failed");
       exit(0);
      }
    else 
      {
       ROS_INFO("Call to cog service stop DONE");
      }
        //sleep(1.0); 
     if (!ros::service::call(weight_service_stop_name, srv))
      {
       ROS_ERROR("Call to weight service stop failed");
       exit(0);
      }
      else 
      {
        ROS_INFO("Call to weight service stop DONE");
      }
        //sleep(1.0); 
     if (!ros::service::call(grasp_shape_service_stop_name, srv))
      {
       ROS_ERROR("Call to grasp shape service stop failed");
       exit(0);
      }
     else 
      {
        ROS_INFO("Call to grasp shape service stop DONE");
      }  
        //sleep(1.0); 
     ROS_INFO("Stopping Object Clustering Recognition  node");

     run_state = false;
     return true;
  }

  void Object_Clustering_Recognition_Server::stop_testing()
  {
    if(test==(int)1) {saveDataClustering();} 
    
    std::string cog_service_stop_name("/cog_estimator_server/stop");
    std::string weight_service_stop_name("/Weight_Estimator_Server/stop"); 
    std::string grasp_shape_service_stop_name("/Grasp_Shape_Estimator_Server/stop");
  //   std_srvs::Empty::Request req_;
    std_srvs::Empty srv;
       nh_.getParam(node_name+"data_file_name_txt_dir", data_file_name_txt_dir);
     data_file_name_txt="data.txt";
    if (!ros::service::call(cog_service_stop_name,srv))
     {
       ROS_ERROR("Call to cog service stop failed");
       exit(0);
      }
    else 
      {
       ROS_INFO("Call to cog service stop DONE");
      }
        //sleep(1.0); 
     if (!ros::service::call(weight_service_stop_name, srv))
      {
       ROS_ERROR("Call to weight service stop failed");
       exit(0);
      }
      else 
      {
        ROS_INFO("Call to weight service stop DONE");
      }
        //sleep(1.0); 
     if (!ros::service::call(grasp_shape_service_stop_name, srv))
      {
       ROS_ERROR("Call to grasp shape service stop failed");
       exit(0);
      }
     else 
      {
        ROS_INFO("Call to grasp shape service stop DONE");
      }  
        //sleep(1.0); 
     ROS_INFO("Stopping Object Clustering Recognition  node");

     run_state = false;
  }   


  void Object_Clustering_Recognition_Server::addData()
  {
    if(cog_now.cog_stop==true)
      
      { // summuning data
    
        if(  std::isnan(object_data.weight || std::isinf(object_data.weight))   ||
             std::isnan(cog_now.mass_2 || std::isinf(cog_now.mass_2))           ||
             std::isnan(cog_now.weight || std::isinf(cog_now.density))          ||
             std::isnan(cog_now.cx || std::isinf(cog_now.cx))                   ||    
             std::isnan(cog_now.cy || std::isinf(cog_now.cy))                   ||      
             std::isnan(cog_now.cz || std::isinf(cog_now.cz))                   ||    
             std::isnan(cog_now.gx || std::isinf(cog_now.gx))                   ||    
             std::isnan(cog_now.gy || std::isinf(cog_now.gy))                   ||      
             std::isnan(cog_now.gz || std::isinf(cog_now.gz))                                             
         ) 
          {ROS_ERROR(" no valid data from cog estimation");}
       else  
       { 
        count_data=count_data+1;
        //object proprieties
        object_data.weight+=weight_now.weight;
        object_data.mass+=cog_now.mass_2;
        object_data.density+=cog_now.density;
        object_data.cx+=cog_now.cx;
        object_data.cy+=cog_now.cy;
        object_data.cz+=cog_now.cz;
        
        object_data.gx+=cog_now.gx;
        object_data.gy+=cog_now.gy;
        object_data.gz+=cog_now.gz;
       
        //grasp_shape
        object_data.x_0+=grasp_shape_now.x_0;
        object_data.y_0+=grasp_shape_now.y_0;
        object_data.z_0+=grasp_shape_now.z_0;
        object_data.r_1+=grasp_shape_now.r_1;

        object_data.h+=grasp_shape_now.h;
        object_data.w+=grasp_shape_now.w;
        object_data.de+=grasp_shape_now.de;

       object_data.r_2+=grasp_shape_now.r_2;
       object_data.r_3+=grasp_shape_now.r_3;

       object_data.a+=grasp_shape_now.a;
       object_data.b+=grasp_shape_now.b;
       object_data.c+=grasp_shape_now.c;
       object_data.d+=grasp_shape_now.d;

       object_data.v+=grasp_shape_now.v;
       

       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       // Hand JointState
       object_data.hand_joint_curr_state.header = data_now.hand_joint_curr_state.header;
       object_data.hand_joint_curr_state.position = data_now.hand_joint_curr_state.position;
        
       object_data.sinergy_joint_error=data_now.hand_error_position;
     
       // wrench 
       object_data.wrench_now.header = data_now.arm_wrench_stamped.header; 
       object_data.wrench_now.wrench.force.x += data_now.arm_wrench_stamped.wrench.force.x;
       object_data.wrench_now.wrench.force.y += data_now.arm_wrench_stamped.wrench.force.y;
       object_data.wrench_now.wrench.force.z += data_now.arm_wrench_stamped.wrench.force.z;

       object_data.wrench_now.wrench.torque.x += data_now.arm_wrench_stamped.wrench.torque.x;
       object_data.wrench_now.wrench.torque.y += data_now.arm_wrench_stamped.wrench.torque.y;
       object_data.wrench_now.wrench.torque.z += data_now.arm_wrench_stamped.wrench.torque.z; 
       }  
     }
    else
     {
       if(count_data!=0)
       { 
         ROS_INFO("<----------Adding data to database---------->");
         //ROS_INFO("MEAN data");  
         //object proprieties
         object_data.weight=object_data.weight/count_data;
         m(0)=object_data.weight;
         object_data.mass=object_data.mass/count_data;
         m(1)=object_data.mass;
         object_data.density=object_data.density/count_data;
         m(2)=object_data.density;
         object_data.cx=object_data.cx/count_data;
         m(3)=object_data.cx;
         object_data.cy=object_data.cy/count_data;
         m(4)=object_data.cy;
         object_data.cz=object_data.cz/count_data;
         m(5)=object_data.cz;
         object_data.gx=object_data.gx/count_data;
         m(6)=object_data.gx;
         object_data.gy=object_data.gy/count_data;
         m(7)=object_data.gy;
         object_data.gz=object_data.gz/count_data;
         m(8)=object_data.gz;
         ROS_INFO("Cog data Acquired");

         //grasp_shape
         object_data.x_0=object_data.x_0/count_data;
         m(9)=object_data.x_0;
         object_data.y_0=object_data.y_0/count_data;
         m(10)=object_data.y_0;
         object_data.z_0=object_data.z_0/count_data;
         m(11)=object_data.z_0;
         object_data.r_1=object_data.r_1/count_data;
         m(12)=object_data.r_1;
         object_data.h=object_data.h/count_data;
         m(13)=object_data.h;
         object_data.w=object_data.w/count_data;
         m(14)=object_data.w;
         object_data.de=object_data.de/count_data;
         m(15)=object_data.de;
         object_data.r_2=object_data.r_2/count_data;
         m(16)=object_data.r_2;
         object_data.r_3=object_data.r_3/count_data;
         m(17)= object_data.r_3;
         object_data.a=object_data.a/count_data;
         m(18)=object_data.a;
         object_data.b=object_data.b/count_data;
         m(19)=object_data.b;
         object_data.c=object_data.c/count_data;
         m(20)=object_data.c;
         object_data.d=object_data.d/count_data;
         m(21)=object_data.d;
         object_data.v=object_data.v/count_data;
         m(22)=object_data.v;
         ROS_INFO("Grasp Shape data Acquired");  
       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //       ['left_hand_index_abd_joint', 'left_hand_index_inner_joint', 'left_hand_index_inner_joint_mimic', 'left_hand_index_middle_joint', 'left_hand_index_middle_joint_mimic', 'left_hand_index_outer_joint', 'left_hand_index_outer_joint_mimic', 'left_hand_little_abd_joint', 'left_hand_little_inner_joint', 
         
  //        'left_hand_little_inner_joint_mimic', 'left_hand_little_middle_joint', 'left_hand_little_middle_joint_mimic', 
  //        
  //        'left_hand_little_outer_joint', 'left_hand_little_outer_joint_mimic', 'left_hand_middle_abd_joint',
  //        
  //        'left_hand_middle_inner_joint', 'left_hand_middle_inner_joint_mimic', 'left_hand_middle_middle_joint', 
  //        
  //        'left_hand_middle_middle_joint_mimic', 'left_hand_middle_outer_joint', 'left_hand_middle_outer_joint_mimic', 
  //        
  //        'left_hand_ring_abd_joint', 'left_hand_ring_inner_joint', 'left_hand_ring_inner_joint_mimic', 
  //        
  //        'left_hand_ring_middle_joint', 'left_hand_ring_middle_joint_mimic', 'left_hand_ring_outer_joint', 
  //        
  //        'left_hand_ring_outer_joint_mimic', 'left_hand_synergy_joint', 'left_hand_thumb_abd_joint', 'left_hand_thumb_inner_joint', 'left_hand_thumb_inner_joint_mimic', 'left_hand_thumb_outer_joint', 'left_hand_thumb_outer_joint_mimic']



       ///   inserire la parte  sui hand joint state  
       //  hand current joints state 

       m(23)=  object_data.hand_joint_curr_state.position[28];
       // Hand sinergy joint errror
       object_data.sinergy_joint_error=object_data.sinergy_joint_error;
       m(24)=object_data.sinergy_joint_error;
       // contiue with orther joints

       // thumb
       m(25) = object_data.hand_joint_curr_state.position[0]; 
       m(26) = object_data.hand_joint_curr_state.position[1];
       m(27) = object_data.hand_joint_curr_state.position[3];
       // index 
       m(28) = object_data.hand_joint_curr_state.position[5]; //'left_hand_index_outer_joint'
       m(29) = object_data.hand_joint_curr_state.position[6]; //  'left_hand_little_abd_joint'
       m(30) =  object_data.hand_joint_curr_state.position[8]; // 'left_hand_little_inner_joint'
       m(31) =  object_data.hand_joint_curr_state.position[10]; //'left_hand_little_middle_joint'
       
       m(32) =  object_data.hand_joint_curr_state.position[12]; // 'left_hand_little_outer_joint'
       m(33) =  object_data.hand_joint_curr_state.position[13]; // 'left_hand_middle_abd_joint'
       m(34) =  object_data.hand_joint_curr_state.position[15]; // 'left_hand_middle_inner_joint'
       m(35) =  object_data.hand_joint_curr_state.position[17]; // 'left_hand_middle_middle_joint'
       
       m(36) =  object_data.hand_joint_curr_state.position[19]; // 'left_hand_middle_outer_joint'
       m(37) =  object_data.hand_joint_curr_state.position[20]; // 'left_hand_ring_abd_joint'
       m(38) =  object_data.hand_joint_curr_state.position[22]; // 'left_hand_ring_inner_joint'
       m(39) =  object_data.hand_joint_curr_state.position[24]; // 'left_hand_ring_middle_joint'
       
       m(40) =  object_data.hand_joint_curr_state.position[26]; // 'left_hand_ring_outer_joint'
       m(41) =  object_data.hand_joint_curr_state.position[27]; //  'left_hand_thumb_abd_joint'
       m(42) =  object_data.hand_joint_curr_state.position[29]; //  'left_hand_thumb_inner_joint'
       m(43) = object_data.hand_joint_curr_state.position[31]; // 'left_hand_thumb_outer_joint'
       ROS_INFO("Joint state  Acquired");

       // wrench
       object_data.wrench_now.wrench.force.x= object_data.wrench_now.wrench.force.x /count_data;
       m(44)=object_data.wrench_now.wrench.force.x;
       object_data.wrench_now.wrench.force.y = object_data.wrench_now.wrench.force.y/count_data;
       m(45)=object_data.wrench_now.wrench.force.y;
       object_data.wrench_now.wrench.force.z = object_data.wrench_now.wrench.force.z /count_data;
       m(46)=object_data.wrench_now.wrench.force.z;
       object_data.wrench_now.wrench.torque.x= object_data.wrench_now.wrench.torque.x/count_data;
       m(47)=object_data.wrench_now.wrench.torque.x;
       object_data.wrench_now.wrench.torque.y= object_data.wrench_now.wrench.torque.y/count_data;
       m(48)=object_data.wrench_now.wrench.torque.y;
       object_data.wrench_now.wrench.torque.z = object_data.wrench_now.wrench.torque.z/count_data;
       m(49)=object_data.wrench_now.wrench.torque.z;
       ROS_INFO("Wrench  Acquired");
         
       ROS_INFO("Data Acquiring  DONE");
       
       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
       flann_query = boost::make_shared<dataset> (new double[1*n_descriptors],1,n_descriptors);
      // riempire m for adding new data 
        if(test==(int)1) {
  	samples.push_back(m);
  	labels.push_back(labels_type);
       for(int i=0; i<n_descriptors;++i)
      { 
        flann_query->operator[](0)[i]=m(i);
      }
      flann_index->addPoints(*flann_query,1);
      flann_index->save(data_file_dir + "/flann_index.idx");
      flann::save_to_file(*flann_database,data_file_dir + "/flann_database.hdf5","flann_database");
      saveDataClustering();	
        }
        if(test==(int)2) {
  	//samples.push_back(m);
  	labels.push_back(labels_type);
        for(int i=0; i<n_descriptors;++i)
      { 
        flann_query->operator[](0)[i]=m(i);
      }
    stop_testing();
        }
       

       //this->saveDataToTxt();
       add_data = true;
       if(count_data!=0){
         ROS_INFO("Data number %d",(int)number_data+1);
         number_data=number_data+1;
         ROS_INFO("<----------Add data to database DONE---------->");
         //ROS_INFO("count data: %d", (int)count_data);
         
       }
     }
     count_data=0;
   } 
  }



  void Object_Clustering_Recognition_Server::getROSParameters()
  {
  // Get the name of output data file
      if(nh_.hasParam(node_name+"data_file_name"))
      {
       nh_.getParam(node_name+"data_file_name",data_file_name);
       ROS_INFO_STREAM("Output data file:  " + data_file_name);
      }
      else
      {
        ROS_ERROR("No data_file_name parameter");
      }
    // Get the name of output centroid file  
      if(nh_.hasParam(node_name+"centroid_file_name"))
      {
       nh_.getParam(node_name+"centroid_file_name",centroid_file_name);
       ROS_INFO_STREAM("Output centroid file:  " + centroid_file_name);  
      }
      else
      {
        ROS_ERROR("No centroid_file_name parameter");
      }
      // Get the name of data file directory
      if(nh_.hasParam(node_name+"data_file_dir"))
      {
       nh_.getParam(node_name+"data_file_dir", data_file_dir);
       ROS_INFO_STREAM("Data file directory:  " + data_file_dir);  
      } 
      else
      {
       ROS_ERROR("No data_file_dir parameter. ");
      } 
      // Get the name of centroid file directory
      if(nh_.hasParam(node_name+"centroid_file_dir"))
      {
       nh_.getParam(node_name+"centroid_file_dir", centroid_file_dir);
       ROS_INFO_STREAM("Centroid file directory:  " + centroid_file_dir);
        
      } 
      else
      {
       ROS_ERROR("No data_file_dir parameter. ");
      } 
      // Get the name of file to store data
      if(nh_.hasParam(node_name+"data_file_name_txt"))
      {
       nh_.getParam(node_name+"data_file_name_txt", data_file_name_txt);
       ROS_INFO_STREAM("Data file txt:  " + data_file_name_txt);
      }
      else
     {
       ROS_ERROR("No data_file_name_txt parameter");
     }
    // Get the name of directory to save data in txt
     if(nh_.hasParam(node_name+"data_file_name_txt_dir"))
     {
      nh_.getParam(node_name+"data_file_name_txt_dir", data_file_name_txt_dir);
      ROS_INFO_STREAM("Data file txt  directory:  " + data_file_name_txt_dir);
     } 
     else
     {
      ROS_ERROR("No data_file_name_txt_dir parameter");
     }
   
  // whether the user wants calculate centroid
     if(nh_.hasParam(node_name+"recalculate_centroid"))
     {
       nh_.param(node_name+"recalculate_centroid", recalculate_centroid, true);
       if(recalculate_centroid==true) ROS_INFO_STREAM(" Set recalculate centroid to:  true");
       else ROS_INFO_STREAM("Set recalculate centroid to:  false");
     }
     else
     {
      ROS_ERROR("No recalculate_centroid parameter");
     }
     
     // whether the user wants to initialize header of txt file 
     if(nh_.hasParam(node_name+"create_header"))
     {
       nh_.param(node_name+"create_header", create_header, false);
       if(create_header==true) ROS_INFO_STREAM(" Set create header to:  true");
       else ROS_INFO_STREAM("Set create header to:  false");
     }
     else
     {
      ROS_ERROR("No Set create header  parameter");
     }
       
    if(nh_.hasParam(node_name+"count_centroid"))
     {
       nh_.getParam(node_name+"count_centroid", count_centroid);
       ROS_INFO("Found %d centroid",count_centroid);
     }
     else
     {
      ROS_ERROR("No count centroid parameter");
     }
     centroid_vector.resize(count_centroid);
  }


  void Object_Clustering_Recognition_Server::setROSParameters()
  {
    // set the name of output data file
      //  data_file_name=object_name+"_"+grasp_type+"_"+data_file_name;
      //  ROS_INFO_STREAM("Output data file:  " +data_file_name);
   
      // // set the name of data file directory
      //  data_file_dir=data_file_dir+"/"+object_name+"/"+grasp_type;
      //  ROS_INFO_STREAM("Data file directory:  " + data_file_dir);  
   
      // set the name of file to store data in txt
       data_file_name_txt=object_name+"_"+grasp_type+"_"+data_file_name_txt;
       ROS_INFO_STREAM("Data file txt:  " + data_file_name_txt);
    // set the name of directory to save data in txt
     data_file_name_txt_dir=data_file_name_txt_dir+"/"+object_name+"/"+grasp_type;
      ROS_INFO_STREAM("Data file txt  directory:  " + data_file_name_txt_dir);
      
      	if(nh_.hasParam(node_name+"count_exp_"+object_name+"_"+grasp_type)) 
       	{
            nh_.getParam(node_name+"count_exp_"+object_name+"_"+grasp_type, number_data);
       
             ROS_INFO("Start from experiment: %d",(int)number_data);
  	   if (number_data==0){count_centroid=count_centroid+1;}
  	}
  	else
  	{ROS_ERROR("NO INITIAL Start experiment value ");}
      // initialize the file with data
  // expand the path
   
            if(!data_file_name_txt_dir.empty() && data_file_name_txt_dir[0] == '~') {
                assert(data_file_name_txt_dir.size() == 1 or data_file_name_txt_dir[1] == '/');  // or other error handling
                char const* home = getenv("HOME");
                if (home or (home = getenv("USERPROFILE"))) {
                    data_file_name_txt_dir.replace(0, 1, home);
                }
                else {
                    char const *hdrive = getenv("HOMEDRIVE"),
                            *hm_meas_file_dir = getenv("HOMEPATH");
                    assert(hdrive);  // or other error handling
                    assert(hm_meas_file_dir);
                    data_file_name_txt_dir.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
                }
            }
    if (create_header==true){
            std::ofstream data_txt_file;
            data_txt_file.open(( data_file_name_txt_dir + "/" + data_file_name_txt).c_str(), std::ios::out);
  // 
            std::stringstream data_txt_file_header;
  // 
            data_txt_file_header << object_name+" "+" "+grasp_type+" "+"data  \n";
            data_txt_file_header << "\% [object, grasp_type, weight, mass, density, cx, cy, cz, gx, gy, gz, x_0, y_0, z_0, r_1, h, w, de, r_2, r_3, a, b, c, d, v, left_hand_synergy_joint,  sinergy_joint_error, left_hand_index_abd_joint, left_hand_index_inner_joint, left_hand_index_middle_joint,  left_hand_index_outer_joint,  left_hand_little_abd_joint, left_hand_little_inner_joint,  left_hand_little_middle_joint,  left_hand_little_outer_joint,  left_hand_middle_abd_joint, left_hand_middle_inner_joint,  left_hand_middle_middle_joint,  left_hand_middle_outer_joint,  left_hand_ring_abd_joint, left_hand_ring_inner_joint,  left_hand_ring_middle_joint,  left_hand_ring_outer_joint , left_hand_thumb_abd_joint, left_hand_thumb_inner_joint,  left_hand_thumb_outer_joint, fx, fy, fz, tx, ty, tz]\n";
  	  
  	  data_txt_file << data_txt_file_header.str();
            data_txt_file.close();
  	  ROS_INFO_STREAM("Header created");
    }	  
    else{
      ROS_INFO_STREAM("Header already created");
    }	           
  }


  void Object_Clustering_Recognition_Server::saveDataToTxt()
  {
          std::ofstream data_txt_file;
          data_txt_file.open(( data_file_name_txt_dir + "/" + data_file_name_txt).c_str(), std::ios::out | std::ios::app);

          std::stringstream data_file_text;
  	data_file_text << object_name << " " << grasp_type << " ";
          data_file_text << object_data.weight << " " << object_data.mass << " " << object_data.density << " ";
          data_file_text << object_data.cx << " " << object_data.cy << " " << object_data.cz << " ";
          data_file_text << object_data.gx << " " << object_data.gy << " " << object_data.gz;
          data_file_text << object_data.x_0 << " " << object_data.y_0 << " " << object_data.z_0 << " ";
          data_file_text << object_data.r_1 << " " << object_data.h << " " << object_data.w << " ";
          data_file_text << object_data.de << " " << object_data.r_2 << " " << object_data.r_3 << " ";
          data_file_text << object_data.a << " " << object_data.b << " " << object_data.b << " ";
          data_file_text << object_data.d << " " << object_data.v << " ";
          
          //data_file_text << left_hand_synergy_joint << " ";
          data_file_text <<m(23)<< " "<< object_data.sinergy_joint_error << " ";
          

            ///////////////////////////////////////////
           // Insert the hand joint  current state  //
          ///////////////////////////////////////////
          data_file_text <<m(25)<< " "<<m(26)<< " "<<m(27)<< " " <<m(28)<< " "<<m(29)<< " "<<m(30)<< " "<<m(31)<< " "<<m(32)<< " "<<m(33)<< " "<<m(34)<< " "<<m(35)<< " "<<m(36)<< " "<<m(37)<< " "<<m(38)<< " "<<m(39)<< " "<<m(40)<< " "<<m(41)<< " "<<m(42)<< " "<<m(43)<< " ";



        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          
          data_file_text << object_data.wrench_now.wrench.force.x << " " << object_data.wrench_now.wrench.force.y << " " << object_data.wrench_now.wrench.force.z << " ";
          data_file_text << object_data.wrench_now.wrench.torque.x << " " << object_data.wrench_now.wrench.torque.y << " " << object_data.wrench_now.wrench.torque.z;
          data_file_text << "\n";
  	
          data_txt_file << data_file_text.str();
     
          data_txt_file.close();
          ROS_INFO("Data Saved to txt ");
  }


  void Object_Clustering_Recognition_Server::saveDataClustering()
  {

       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // weight, mass, density, cx, cy, cz, , gx, gy, gz
      // x_0, y_0, z_0, r_1, h, w, de, r_2, r_3, a, b, c, d, v,
      // left_hand_synergy_joint, sinergy_joint_error, 
      // left_hand_index_abd_joint, left_hand_index_inner_joint, left_hand_index_middle_joint,  left_hand_index_outer_joint,  
      // left_hand_little_abd_joint, left_hand_little_inner_joint,  left_hand_little_middle_joint,  left_hand_little_outer_joint,  
      // left_hand_middle_abd_joint, left_hand_middle_inner_joint,  left_hand_middle_middle_joint,  left_hand_middle_outer_joint,  
      // left_hand_ring_abd_joint, left_hand_ring_inner_joint,  left_hand_ring_middle_joint,  left_hand_ring_outer_joint ,
      // left_hand_thumb_abd_joint, left_hand_thumb_inner_joint,  left_hand_thumb_outer_joint, 
      // fx, fy, fz, tx, ty, tz]\n";


     ROS_INFO("Saving database into a yaml file");

     XmlRpc::XmlRpcValue database;
     database.setSize(50);
     for(unsigned int i=0; i<50; i++)
  	 database[i] = (double)m(i);
     ROS_INFO("Database copy Done");
   // // set the parameters in the parameter server
    // number_data=1;
    int n_data(number_data-1);
    if(n_data<0){n_data=0;}
    std::string n_string;
      
    std::ostringstream convert;   // stream used for the conversion
    convert << n_data; 
    n_string=convert.str();
    nh_.setParam(node_name+object_name+"_"+grasp_type+"_exp_"+ n_string, database);
    nh_.setParam(node_name+"count_exp_"+object_name+"_"+grasp_type, number_data);
    nh_.setParam(node_name+"count_centroid", count_centroid);
      ROS_INFO("Set param name DONE");
   // // dump the parameters to YAML file
    std::string file = data_file_dir +"/" + data_file_name;
   ROS_INFO_STREAM("Saving on : " +file);
   // // first create the directory
   std::string command = std::string("mkdir -p ") + data_file_dir  ;
   std::system(command.c_str());

   // // now dump the yaml file
    command.clear();
    command = std::string("rosparam dump ")  + file + std::string(" /object_clustering_recognition_server");
    std::system(command.c_str());

     ROS_INFO("Save database into a yaml file: DONE"); 
   ROS_INFO("ALL DATA SAVED");
  }


  void Object_Clustering_Recognition_Server::loadDataBase()
  {
    ROS_INFO_STREAM("Starting Loading DataBase");

            if(nh_.hasParam(node_name+"make_clustering")) 
          {
            nh_.param(node_name+"make_clustering", make_clustering,false);
            ROS_INFO_STREAM("Loaded clustering choice");
            } 
            else
      {
        ROS_ERROR_STREAM("No clustering choice");
      }

    
                  if(nh_.hasParam(node_name+"load_database_in_flann")) 
          {
            nh_.param(node_name+"load_database_in_flann", load_database_in_flann,true);
            ROS_INFO_STREAM("load_database_in_flann choice");
            } 
            else
      {
        ROS_ERROR_STREAM("No load_database_in_flann");
      }
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////   NEW CODE ////////////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

     // First Load the parameters usefull for loading all database 
    	   
     // object number into database
     if(nh_.hasParam(node_name+"count_object_type"))
     {
       nh_.getParam(node_name+"count_object_type", object_type_database);
       ROS_INFO("Found %d  object in database",(int)object_type_database);
       object_name_vector.clear();
       object_type_grasp_database.clear();
       
       object_name_vector.resize(object_type_database);
       object_type_grasp_database.resize(object_type_database);
       exp_grasp_type_object[0].resize(object_type_database);
       exp_grasp_type_object[1].resize(object_type_database);
       exp_grasp_type_object[2].resize(object_type_database);
       //ROS_INFO("Data resized");
         	if(nh_.hasParam(node_name+"object_name")) 
       	        {
  	        nh_.getParam(node_name+"object_name", object_name_vector);
  	        ROS_INFO_STREAM("Loaded Object Names");
  	        } 
  	        else
  		{
  		  ROS_ERROR_STREAM("No object name");
  		}
      
       for (int i = 0; i < object_type_database; ++i)
       {  
          std::string i_string; 
          std::ostringstream convert;   // stream used for the conversion
          convert << i; 
          i_string=convert.str();
  	//ROS_INFO_STREAM("Loading data"+i_string);

       	if(nh_.hasParam(node_name+"grasp_type_object_"+i_string)) 
       	{
       	 nh_.getParam(node_name+"grasp_type_object_"+i_string, object_type_grasp_database[i]); // so this will be an std::vector
          ROS_INFO("Grasp type %d for object %s",(int) object_type_grasp_database[i],object_name_vector[i].c_str());
  	     	if(nh_.hasParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_0")) 
       	   {
  	          nh_.getParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_0", exp_grasp_type_object[0][i]);
  	          ROS_INFO("Loaded %d exp grasp_0 for %s", exp_grasp_type_object[0][i],object_name_vector[i].c_str() );
  	        }  
  	        else
  		   {
  		    ROS_ERROR_STREAM("No experimetal number found for object "+ object_name_vector[i] +"with grasp type "+ i_string);
  		  }    
  		
  		if(nh_.hasParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_1")) 
       {
  	          nh_.getParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_1", exp_grasp_type_object[1][i]);
  	          ROS_INFO("Loaded %d exp grasp_1 for %s", exp_grasp_type_object[1][i] ,object_name_vector[i].c_str());
  	   }  
  	        else
      {
  		  ROS_ERROR_STREAM("No experimetal number found for object "+ object_name_vector[i] +"with grasp type "+ i_string);
  		} 
  			     	
  	        if(nh_.hasParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_2")) 
       	  {
  	          nh_.getParam(node_name+"count_exp_"+object_name_vector[i].c_str()+"_grasp_2", exp_grasp_type_object[2][i]);
  	          ROS_INFO("Loaded %d exp grasp_2 for %s", exp_grasp_type_object[2][i] ,object_name_vector[i].c_str());
  	        }  
  	        else
  		{
  		  ROS_ERROR_STREAM("No experimetal number found for object "+ object_name_vector[i] +"with grasp type "+ i_string);
  		} 
  	}
          else
          {
           ROS_ERROR("No grasp object type %d parameter", (int)i);
          }
           num_dataset+=exp_grasp_type_object[0][i]+exp_grasp_type_object[1][i]+exp_grasp_type_object[2][i];
       }

     }
     else
     {
      ROS_ERROR("No object type number parameter");
     }

      ROS_INFO("Data in database %d",(int)num_dataset);
      ROS_INFO_STREAM("Loaded grasp types for each object");
      // Then Load all database
       dataset database_temp(new double[num_dataset*n_descriptors],num_dataset,n_descriptors);
       
       ROS_INFO_STREAM("Flann Database created"); 
       dataset query_temp(new double[1*n_descriptors],1,n_descriptors);
      
       ROS_INFO_STREAM("Flann Query Created");

       int lab(1),index_database(0);
       samples.clear();
       flann_object_recognition.clear();
//       flann_object_recognition.resize(num_dataset);
      for (int i = 0; i < object_type_database; ++i)
      { 	
         for (int j = 0; j < object_type_grasp_database[i]; ++j)
         {
	   std::string j_string;  std::ostringstream convert;
           convert << j; j_string=convert.str();
             for (int k = 0; k < exp_grasp_type_object[j][i]; ++k)
              {     
                std::string k_string; 
                std::ostringstream convert_2;   // stream used for the conversion
                convert_2<< k;
                k_string=convert_2.str();
                while(!nh_.hasParam(node_name+object_name_vector[i]+"_"+"grasp_"+j_string+"_exp_"+k_string)){ros::spinOnce();}
                if(nh_.hasParam(node_name+object_name_vector[i]+"_"+"grasp_"+j_string+"_exp_"+k_string)){
                 
    		
                std::vector<double> data_in;
                data_in.clear();
    		 data_in.resize(50);
         	 nh_.getParam(node_name+object_name_vector[i]+"_"+"grasp_"+j_string+"_exp_"+k_string,data_in);   
                 for ( int ii =0; ii<data_in.size(); ++ii)
		 {
    		   m(ii)=data_in[ii];
     		   
                   database_temp[index_database][ii]=data_in[ii];
	           if(index_database==(num_dataset-1))
                  {
                   query_temp[0][ii]=m(ii);
                   //std::cout <<  query_temp[0][ii] << " ";
                   if (ii == (n_descriptors-1)){std::cout << endl;}
                  } 
	         }
         	  samples.push_back(m);
    		  //ROS_INFO("Class labels: %d",(int)f);
    		  //ROS_INFO_STREAM(object_name_vector[i]);
		  //std::cout << lab << endl;
    		  labels.push_back(lab); //<--- insert  label
    		  //object_recognition[f] = ;  
          
                 //ROS_INFO("Loaded exp %d",index_database);
                 //std::cout << endl;
		 flann_object_recognition.push_back(object_name_vector[i]+" with  " + "grasp_"+j_string);
                 if(index_database<num_dataset){index_database=index_database+1;}
                 else {ROS_ERROR("Exceded dataset");}
		 }
		 else
		 {
		   ROS_INFO("No data experiment %d",index_database);
		}
              }  
           if(lab<count_centroid){lab=lab+1;};
         }
      }
      ROS_INFO_STREAM("LOAD EXP");
      
      int f=0;
       object_recognition.clear();
      
       for (int i = 0; i < object_type_database; ++i)
       { 	         
 	for (int j = 0; j < object_type_grasp_database[i]; ++j)
         { 
//   	  if(f<=count_centroid)
//             {
            std::string j_string;  std::ostringstream convert;
            convert << j; j_string=convert.str();
          object_recognition.push_back(object_name_vector[i]+" with  " + "grasp_"+j_string);
     	  //ROS_INFO_STREAM(object_name_vector[i]+" with  " + "grasp_"+j_string);
           //std::cout << i <<" "<< j <<" " << f << endl;
 	   // ROS_INFO("Class labels: %d",(int)f);
//  	  if(f<count_centroid){f=f+1;};
        
 	  //} 
 	 }
      } 

       flann_database.reset();
       flann_query.reset();
       flann_database = boost::make_shared<dataset>(database_temp); 
       ROS_INFO_STREAM("Boost flann database");
       flann_query = boost::make_shared<dataset>(query_temp); 
       ROS_INFO_STREAM("Boost flann query");
      ROS_INFO_STREAM("DataBase Loaded");
    }



  void Object_Clustering_Recognition_Server::clustering()
  {
//    //  6 elements from cog estimator
//    // 14 elements from shape grasp estimator
//    // 20 elements from hand joints state 
//    // 1 element from sinergy error state (position)
//    // 6 element from F/T force torque sensor
//    // 3 element from Imu Sensor
   
   
  ROS_INFO("<----------Multi training start ---------->");  

   object_trainer trainer;
  // 
   krr_trainer<kernel_type> rbf_trainer;
  rbf_trainer.set_kernel(kernel_type(0.01));
   trainer.set_trainer(rbf_trainer);
  // randomize_samples(samples, labels);
  // //ROS_INFO("DONE");
  // //cout << "cross validation: \n" << cross_validate_multiclass_trainer(trainer, samples, labels, 3*count_centroid) << endl;
   one_vs_one_decision_function<object_trainer> df = trainer.train(samples, labels);
  // 
  // //  for (unsigned long i = 0; i < samples.size(); ++i)
  // //  {
  // //  cout << "predicted label: "<< df(samples[i])  << ", true label: "<< labels[i] << endl;
  // //  }
   cout << "test multiclass function: \n" << test_multiclass_decision_function(df, samples, labels) << endl;

   ROS_INFO("<----------Multi Training Done---------->"); 
      

   ROS_INFO("<----------Flann Training start---------->");


   if(load_database_in_flann==true){ 
     
//            Matrix<int> flann_indices(new int[1*nn], 1, nn);
//       Matrix<double> flann_dists(new double[1*nn], 1, nn);
      Index<ChiSquareDistance<double> > flann_index_temp(*flann_database, flann::KDTreeIndexParams(4));
  ROS_INFO("<---------- 2 ---------->");   
      flann_index_temp.buildIndex();                                                                                               
  flann_index = boost::make_shared<Index<ChiSquareDistance<double> > > (flann_index_temp);
      ROS_INFO("<---------- 3 ---------->");
      // do a knn search, using 128 checks
//   flann_index->knnSearch(*flann_query, flann_indices, flann_dists, nn, flann::SearchParams(256));
//   ROS_INFO("4) Flann Indices = %d",flann_indices[0][0]);
//   ROS_INFO("5) Flann ChiSquareDistance = %g",flann_dists[0][0]);
  flann_index->save(data_file_dir + "/flann_index.idx");
  flann::save_to_file(*flann_database,data_file_dir + "/flann_database.hdf5","flann_database");
  
  ROS_INFO("<---------- 5 ---------->");
}
else
{
 flann_database.reset(new dataset); 
flann::load_from_file(*flann_database, data_file_dir + "/flann_database.hdf5","flann_database");
flann_index.reset(new Index<ChiSquareDistance<double> > (*flann_database,SavedIndexParams(data_file_dir + "/flann_index.idx")));
flann_index->buildIndex();
ROS_INFO("Load Database and Index from disk");  
}
  ROS_INFO("<----------Flann Training Done---------->");
  add_data=false;
   
   ROS_INFO("<----------Clustering Done---------->");   
   
   //this->saveDataClustering();
  }




  void Object_Clustering_Recognition_Server::testing()
  {
    
  ROS_INFO("<---------- Multi Training on database ---------->");  

   object_trainer trainer;
   ROS_INFO("<---------- 1 ---------->");
   krr_trainer<kernel_type> rbf_trainer;
   ROS_INFO("<---------- 2---------->");
   rbf_trainer.set_kernel(kernel_type(0.01));
   ROS_INFO("<---------- 3 ---------->");
   trainer.set_trainer(rbf_trainer);
  // //randomize_samples(samples, labels);
  //
    ROS_INFO("<---------- 4 ---------->");
    one_vs_one_decision_function<object_trainer> df = trainer.train(samples, labels);
    ROS_INFO("<---------- 5 ---------->");
    //cout << "cross validation: \n" << cross_validate_multiclass_trainer(trainer, samples, labels, count_centroid) << endl;
    

    ROS_INFO("<---------- Multi Training on database Done ---------->");


   ROS_INFO("<---------- Multiclass function ---------->");
    cout << "test multiclass function: \n" << test_multiclass_decision_function(df, samples, labels) << endl;
  
           one_vs_one_decision_function<object_trainer>::binary_function_table functs;
           functs = df.get_binary_decision_functions();
           cout << "number of binary decision functions in df: " << functs.size() << endl;

   cout << endl;
   ROS_INFO("<-- -- Multiclass start --->"); 
   cout << "predicted label: "<< df(m)  <<  endl;
   ROS_INFO("<-- -- Multiclass DONE>"); 
   int obj((int)df(m));
   ROS_INFO("class  %d",(int)df(m));
   int index_ob((obj-1));
   ROS_INFO("Index_ob %d",(int)index_ob);
   ROS_INFO_STREAM(" This is:   "+ object_recognition[index_ob]); 
   
  voice.arg = " This is:   "+ object_recognition[index_ob];
  talker.publish(voice);
  
     ROS_INFO("<----------Flann Testing---------->");
  
   flann_database.reset(new dataset); 
   flann::load_from_file(*flann_database, data_file_dir + "/flann_database.hdf5","flann_database");
   flann_index.reset(new Index<ChiSquareDistance<double> > (*flann_database,SavedIndexParams(data_file_dir + "/flann_index.idx")));
   flann_index->buildIndex();
   ROS_INFO("Load Flann Database and Index from disk");  
     ROS_INFO("Flann Query");
   for(int i=0;i<flann_query->rows;++i)
    {
     for(int j=0;j<flann_query->cols;++j)
     {
       std::cout << flann_query->operator[](i)[j] << " "; 
     }
     std::cout << endl;
    } 
  Matrix<int> flann_indices(new int[1*nn], 1, nn);
  Matrix<double> flann_dists(new double[1*nn], 1, nn);  
  ROS_INFO("Flann knnSearch");
  flann_index->knnSearch(*flann_query, flann_indices, flann_dists, nn, flann::SearchParams(256));
  ROS_INFO("4) Flann Indices = %d",flann_indices[0][0]);
  ROS_INFO("5) Flann ChiSquareDistance = %g",flann_dists[0][0]);
   ROS_INFO_STREAM(" This is:   "+ flann_object_recognition[flann_indices[0][0]]);  
   
   ROS_INFO("<----------Flann Testing DONE---------->");
   ROS_INFO("<----------Testing Done---------->");
   test_made=true;
   
   //object_recognized = true;  // se oggetto riconosciuto
   ROS_INFO("Is the recongnition wrong ?");
   ROS_INFO("- 0 for false ");
   ROS_INFO("- 1 for true.");
   int rec;
   std::cin >> rec; 
   if (rec==0) {	object_recognized==false; ROS_INFO("Trying a new acquisition");}
   else  {object_recognized = true;
        
       // ROS_INFO("Please insert the object name: ");
       // std::cin >> object_name;
       // ROS_INFO("Object Name received: %s",object_name.c_str());
       // ROS_INFO("Please insert the grasp type: ");
       // std::cin >> grasp_type;
       // ROS_INFO("Grasp type received: %s", grasp_type.c_str());
       //stop_testing();
       //run_state=false;
   }
    // altrimenti 
   add_data=false;
  }


  }//namespace grasp_estimator




  int main(int argc, char **argv)
  { 
    ROS_INFO("Object_Clustering_Recognition_Server is Here !!!");
    ros::init(argc, argv, "Object_Clustering_Recognition_Server");
    ros::NodeHandle nh;
    // ros::Rate loop_rate(Hz);
    // ros::AsyncSpinner spinner(1);
    grasp_estimator::Object_Clustering_Recognition_Server object_clustering_recognition_server(nh);
    int test(0);
    object_clustering_recognition_server.add_data=false;
    object_clustering_recognition_server.object_recognized=false;
    object_clustering_recognition_server.getROSParameters();
    object_clustering_recognition_server.loadDataBase();

    //object_clustering_recognition_server.loadCentroid();
    
  object_clustering_recognition_server.clustering();
  object_clustering_recognition_server.testing(); // <---- to delete
  //    ROS_INFO("Plese select the mode:");
  //    ROS_INFO("- 1 for add data to database;");
  //    ROS_INFO("- 2 for testing.");
  //    ROS_INFO("Waiting for choose...");
  //    std::cin >> test;
  //    ROS_INFO("choose received !");
  //    
  //    if((int)test==1){
  //      ROS_INFO("Please insert the object name: ");
  //      std::cin >> object_clustering_recognition_server.object_name;
  //      ROS_INFO("Object Name received: %s", object_clustering_recognition_server.object_name.c_str());
  //      ROS_INFO("Please insert the grasp type: ");
  //      std::cin >> object_clustering_recognition_server.grasp_type;
  //      ROS_INFO("Grasp type received: %s", object_clustering_recognition_server.grasp_type.c_str());
  //      object_clustering_recognition_server.setROSParameters(); 
  //  	     object_clustering_recognition_server.saveDataClustering();
  //      ROS_INFO("Wait for starting new acquisition service  ");
  //     }
  //    else{
  //     
  //    	ROS_INFO("Waiting for starting testing service"); 
  //    }
  //  
  //   object_clustering_recognition_server.test=test;
    //spinner.start();
   //  while (ros::ok())
   //   { 
  	//  	if(object_clustering_recognition_server.run_state==true){
  	//  	   if(object_clustering_recognition_server.test==(int)1)
  	//        {
  	//          object_clustering_recognition_server.addData(); 
   //               if (object_clustering_recognition_server.add_data==true){
  	// 	      //object_clustering_recognition_server.loadDataBase();
  	//              if(object_clustering_recognition_server.make_clustering){ object_clustering_recognition_server.clustering();};
   //            object_clustering_recognition_server.stop_testing();
   //            } 
  	//             }
  	//          else{
  	//           object_clustering_recognition_server.addData();
  	//           if (object_clustering_recognition_server.add_data==true){object_clustering_recognition_server.testing();}
  	//           if (object_clustering_recognition_server.object_recognized==false&&object_clustering_recognition_server.test_made==true){
  	//                ROS_INFO("Switching mode");
  	//                object_clustering_recognition_server.test=1;}
  	//             }
  	//          object_clustering_recognition_server.publishDataObject();  
  	    
  	//     }
  	// }  
    //spinner.stop();
    ros::spinOnce();
    return 0;
  }
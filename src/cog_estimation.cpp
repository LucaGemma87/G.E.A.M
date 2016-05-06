
/*
 *
 *  Created on: 2016
 *  Authors:   Luca Gemma

 */


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <moveit/move_group_interface/move_group.h>
#include <FT_sensor_calib.h>
#include <eigen3/Eigen/Core>

// codice mio

#include <visualization_msgs/Marker.h>
#include "grasp_estimator/WeightEstimated.h"
#include "grasp_estimator/GraspShape.h"


#include <std_srvs/Empty.h>
using namespace grasp_estimator;


class COGEstimNode
{


public:
	ros::NodeHandle n_;
	ros::AsyncSpinner *spinner;
	ros::Subscriber topicSub_ft_raw_;
	ros::Subscriber topicSub_Accelerometer_;
        
    unsigned int m_pose_counter;
    move_group_interface::MoveGroup *m_group;
	ros::Subscriber hand_move_sub;
	ros::Subscriber grasp_shape_sub_;
	ros::Publisher vis_pub;
	ros::Publisher data_estimated_pub;

     /// declaration of service servers
    ros::ServiceServer srvServer_Start_;
    ros::ServiceServer srvServer_Stop_;


	bool hand_stopped;
	bool cog_estim_start;
	double mass_input;
	visualization_msgs::Marker position_hand_msgs;
	grasp_estimator::WeightEstimated data_pub;
	grasp_estimator::GraspShape grasp_shape_now;
	double mass;
	
	double density_object;
    Eigen::Vector3d COM_pos;
    Eigen::Vector3d f_bias;
    Eigen::Vector3d t_bias;
	
	std::string pose_name;
	
	int m_number_random_poses;

	geometry_msgs::Vector3Stamped gravity_ft_frame;

	bool run_state;
	
	COGEstimNode()
	{
		n_ = ros::NodeHandle("~");
		spinner = new ros::AsyncSpinner(1);
		spinner->start();


        
		// topicSub_ft_raw_ = n_.subscribe("ft_raw", 1, &FTCalibNode::topicCallback_ft_raw, this);
		topicSub_ft_raw_ = n_.subscribe("/left_ft_sensor/left/force_torque_sensor_filtered", 1, &COGEstimNode::topicCallback_ft_raw, this);
		//topicSub_Accelerometer_ = n_.subscribe("imu", 1, &FTCalibNode::topicCallback_imu, this);
		//topicSub_Accelerometer_ = n_.subscribe("/left_ft_sensor/left/IMU_filtered", 1, &COGEstimNode::topicCallback_imu, this);
          topicSub_Accelerometer_ = n_.subscribe("/left_hand/imu_info/palm_imu", 1, &COGEstimNode::topicCallback_imu, this);
        hand_move_sub=n_.subscribe("/left_hand/mass_estimator_server_",1,&COGEstimNode::take_hand_stopped,this);
		grasp_shape_sub_ = n_.subscribe<grasp_estimator::GraspShape>(n_.resolveName("/left_hand/grasp_shape_estimator_server_"),1,&COGEstimNode::grasp_estimation_server,this);

		srvServer_Start_ = n_.advertiseService("start", &COGEstimNode::srvCallback_Start,this);
        srvServer_Stop_ = n_.advertiseService("stop", &COGEstimNode::srvCallback_Stop, this);
		

		vis_pub=n_.advertise<visualization_msgs::Marker>("/left_hand/marker_cog_hand_server_",1);
        data_estimated_pub=n_.advertise<grasp_estimator::WeightEstimated>("/left_hand/cog_estimator_server_",1);
		
		m_pose_counter = 0;
		m_ft_counter = 0;

		m_received_ft = false;
		m_received_imu = false;

		m_finished = false;
		m_tf_listener = new tf::TransformListener();

		m_ft_calib = new FTSensorCalib();
		
		run_state=false;
	}

	~COGEstimNode()
	{
		saveCalibData();
		//delete spinner;
		delete m_group;
		//delete m_ft_calib;
		//delete m_tf_listener;
	}

	bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("Starting  Cog Estimation node");
      run_state = true;
      return true;
    }


    bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Stopping Cog Estimation node");
        m_pose_counter=0;
        run_state = false;
        return true;
    }


	bool getROSParameters()
	{

		// Get the moveit group name
		if(n_.hasParam("moveit_group_name"))
		{
			n_.getParam("moveit_group_name", m_moveit_group_name);
		}

		else
		{
			ROS_ERROR("No moveit_group_name parameter, shutting down node...");
			n_.shutdown();
			return false;
		}

		// Get the name of output calibration file
		if(n_.hasParam("calib_file_name"))
		{
			n_.getParam("calib_file_name", m_calib_file_name);
		}

		else
		{
			ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
			m_calib_file_name = std::string("ft_calib_data.yaml");
		}

        // Get the name of calibration file directory
        if(n_.hasParam("calib_file_dir"))
        {
            n_.getParam("calib_file_dir", m_calib_file_dir);
        }

        else
        {
            ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_calib_file_dir = std::string("~/.ros/ft_calib");
        }


        // Get the name of file to store the gravity and F/T measurements
        if(n_.hasParam("meas_file_name"))
        {
            n_.getParam("meas_file_name", m_meas_file_name);
        }

        else
        {
            ROS_WARN("No meas_file_name parameter, setting to default 'ft_calib_meas.txt'");
            m_meas_file_name = std::string("ft_calib_meas.txt");
        }

        // Get the name of directory to save gravity and force-torque measurements
        if(n_.hasParam("meas_file_dir"))
		{
            n_.getParam("meas_file_dir", m_meas_file_dir);
		}

		else
		{
            ROS_WARN("No meas_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_meas_file_dir = std::string("~/.ros/ft_calib");
		}

		if(n_.hasParam("poses_frame_id"))
		{
			n_.getParam("poses_frame_id", m_poses_frame_id);
		}

        else
        {
            ROS_ERROR("No poses_frame_id parameter, shutting down node ...");
            n_.shutdown();
            return false;
        }


        // whether the user wants to use random poses
        n_.param("random_poses", m_random_poses, false);

        // number of random poses
        n_.param("number_random_poses", m_number_random_poses, 2);


        // initialize the file with gravity and F/T measurements

        // expand the path
        if (!m_meas_file_dir.empty() && m_meas_file_dir[0] == '~') {
            assert(m_meas_file_dir.size() == 1 or m_meas_file_dir[1] == '/');  // or other error handling
            char const* home = getenv("HOME");
            if (home or (home = getenv("USERPROFILE"))) {
                m_meas_file_dir.replace(0, 1, home);
            }
            else {
                char const *hdrive = getenv("HOMEDRIVE"),
                        *hm_meas_file_dir = getenv("HOMEPATH");
                assert(hdrive);  // or other error handling
                assert(hm_meas_file_dir);
                m_meas_file_dir.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
            }
        }

        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out);

        std::stringstream meas_file_header;

        meas_file_header << "\% gravity , f/t measurements all expressed in F/T sensor frame\n";
        meas_file_header << "\% [gx, gy, gz, fx, fy, fz, tx, ty, tz]\n";

        meas_file << meas_file_header.str();

        meas_file.close();

        return true;
    }
    // connects to the move arm servers
	void init()
	{
		m_group = new move_group_interface::MoveGroup(m_moveit_group_name);
	}
    
    void grasp_estimation_server(const grasp_estimator::GraspShape::ConstPtr &grasp_shape)
    {
      grasp_shape_now = *grasp_shape;
    }

	// Calibrates the FT sensor by putting the arm in several different positions
	bool moveNextPose()
	{

		std::stringstream ss;
		
		ss << m_pose_counter;
		
		//std::string pose_name;
		//Eigen::Matrix<double, 6, 1> pose;

		// either find poses from the parameter server
		// poses should be in "pose%d" format (e.g. pose0, pose1, pose2 ...)
		// and they should be float arrays of size 6
		if(!m_random_poses)
		{
 			if(!getNamePose("pose"+ss.str(), pose_name))
 			{
 				ROS_INFO("Finished group %s poses", m_group->getName().c_str());
 				m_finished = true;
 				return true;
 			}

			// geometry_msgs::Pose pose_;
			// pose_.position.x = pose(0);
			// pose_.position.y = pose(1);
			// pose_.position.z = pose(2);

			// tf::Quaternion q;
			// q.setRPY((double)pose(3), (double)pose(4), (double)pose(5));

			// tf::quaternionTFToMsg(q, pose_.orientation);

			// geometry_msgs::PoseStamped pose_stamped;
			// pose_stamped.pose = pose_;
			// pose_stamped.header.frame_id = m_poses_frame_id;
			// pose_stamped.header.stamp = ros::Time::now();

			// m_group->setPoseTarget(pose_stamped);
                    m_group->setPlannerId("PRMstarkConfigDefault");
 		   ROS_INFO_STREAM("Arm state desidered: "+  pose_name);
 		   m_group->setNamedTarget(pose_name);
 			      // call the moves
 
 		   if( !(m_group->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
 			{
 			 ROS_ERROR("An error occured during Moving Arm");
 			}

		}
		else // or execute random poses
 		{
 			if(m_pose_counter<=m_number_random_poses-1)
 			{
 				m_group->setRandomTarget();
 				ROS_INFO("Executing pose %d",m_pose_counter);
 			}
 
 			else
 			{
 				ROS_INFO("Finished group %s random poses", m_group->getName().c_str());
 				//m_finished = true;
 				return true;
 			}
		}


		
		m_group->move();
		m_pose_counter++;
		
		ROS_INFO("Finished executing pose %d", m_pose_counter);
		if(this->m_pose_counter>m_number_random_poses-1)
		{ 
		  this->m_pose_counter=0;
		  //m_finished = true;
		  
		}
		return true;
	}


    // gets the next pose from the parameter server
	// pose name is a string
    bool getNamePose(const std::string &pose_param_name, std::string &pose_name)
	{       
	  
	  
		if(n_.hasParam(pose_param_name))
		{
			n_.getParam(pose_param_name, pose_name);
		}
		else
		{
			ROS_WARN("Pose parameter %s not found", pose_param_name.c_str());
			return false;
		}
        return true;
	}

	// prints out the pose (3-D positions) of the calibration frame at each of the positions
	// of the left arm
	void saveCalibData()
	{
		double mass;
		Eigen::Vector3d COM_pos;
		Eigen::Vector3d f_bias;
		Eigen::Vector3d t_bias;

		getCalib(mass, COM_pos, f_bias, t_bias);

		XmlRpc::XmlRpcValue bias;
		bias.setSize(6);
		for(unsigned int i=0; i<3; i++)
			bias[i] = (double)f_bias(i);

		for(unsigned int i=0; i<3; i++)
			bias[i+3] = (double)t_bias(i);

		XmlRpc::XmlRpcValue COM_pose;
		COM_pose.setSize(6);
		for(unsigned int i=0; i<3; i++)
			COM_pose[i] = (double)COM_pos(i);

		for(unsigned int i=0; i<3; i++)
			COM_pose[i+3] = 0.0;

		// set the parameters in the parameter server
		n_.setParam("/ft_calib/bias", bias);
		n_.setParam("/ft_calib/gripper_mass", mass);
		n_.setParam("/ft_calib/gripper_com_frame_id", m_ft_raw.header.frame_id.c_str());
		n_.setParam("/ft_calib/gripper_com_pose", COM_pose);

		// dump the parameters to YAML file
		std::string file = m_calib_file_dir + std::string("/") + m_calib_file_name;

		// first create the directory
		std::string command = std::string("mkdir -p ") + m_calib_file_dir;
		std::system(command.c_str());

		// now dump the yaml file
		command.clear();
		command = std::string("rosparam dump ") + file ;
		std::system(command.c_str());
		ROS_INFO("DATA SAVED");
	}

    // saves the gravity and force-torque measurements to a file for postprocessing
    void saveMeasurements(geometry_msgs::Vector3Stamped gravity, geometry_msgs::WrenchStamped ft_meas)
    {
        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out | std::ios::app);

        std::stringstream meas_file_text;

        meas_file_text << gravity.vector.x << " " << gravity.vector.y << " " << gravity.vector.z << " ";
        meas_file_text << ft_meas.wrench.force.x << " " << ft_meas.wrench.force.y << " " << ft_meas.wrench.force.z << " ";
        meas_file_text << ft_meas.wrench.torque.x << " " << ft_meas.wrench.torque.y << " " << ft_meas.wrench.torque.z << "\n";

        meas_file << meas_file_text.str();

        meas_file.close();
    }

	// finished moving the arm through the poses set in the config file
	bool finished()
	{
		return(m_finished);
	}

	void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
	{
		ROS_DEBUG("In ft sensorcallback");
		m_ft_raw = *msg;
		m_received_ft = true;
	}
       

	// gets readings from accelerometer and transforms them to the FT sensor frame
	void topicCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		ROS_DEBUG("In accelerometer read callback");

		m_imu= *msg;
		m_received_imu = true;
	}

	void addMeasurement()
	{

 		m_ft_avg.wrench.force.x = -m_ft_avg.wrench.force.x/(double)m_ft_counter;
	   		m_ft_avg.wrench.force.y = -m_ft_avg.wrench.force.y/(double)m_ft_counter;
 		m_ft_avg.wrench.force.z = -m_ft_avg.wrench.force.z/(double)m_ft_counter;
 
 		m_ft_avg.wrench.torque.x = -m_ft_avg.wrench.torque.x/(double)m_ft_counter;
		m_ft_avg.wrench.torque.y = -m_ft_avg.wrench.torque.y/(double)m_ft_counter;
 		m_ft_avg.wrench.torque.z = -m_ft_avg.wrench.torque.z/(double)m_ft_counter;

// 	    m_ft_avg.wrench.force.x = m_ft_avg.wrench.force.x/(double)m_ft_counter;
// 		m_ft_avg.wrench.force.y = m_ft_avg.wrench.force.y/(double)m_ft_counter;
// 		m_ft_avg.wrench.force.z = m_ft_avg.wrench.force.z/(double)m_ft_counter;
// 
// 		m_ft_avg.wrench.torque.x = m_ft_avg.wrench.torque.x/(double)m_ft_counter;
// 		m_ft_avg.wrench.torque.y = m_ft_avg.wrench.torque.y/(double)m_ft_counter;
// 		m_ft_avg.wrench.torque.z = m_ft_avg.wrench.torque.z/(double)m_ft_counter;


		m_ft_counter = 0;

		if(!m_received_ft)
		{
			ROS_ERROR("Haven't received F/T sensor measurements");
			return;
		}

		if(!m_received_imu)
		{
			ROS_ERROR("Haven't received accelerometer readings");
			return;
		}

		// express gravity vector in F/T sensor frame
		geometry_msgs::Vector3Stamped gravity;
		gravity.header.stamp = ros::Time();
		gravity.header.frame_id = m_imu.header.frame_id;
		gravity.vector = m_imu.linear_acceleration;
		// gravity.vector.x = -gravity.vector.x;
		// gravity.vector.y = -gravity.vector.y;
		// gravity.vector.z = -gravity.vector.z;

		//geometry_msgs::Vector3Stamped gravity_ft_frame;  New code



		try
		{
			m_tf_listener->transformVector(m_ft_raw.header.frame_id, gravity, gravity_ft_frame);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming accelerometer reading to the F/T sensor frame");
			ROS_ERROR("%s.", ex.what());
			return;
		}

		m_ft_calib->addMeasurement(gravity_ft_frame, m_ft_avg);
        saveMeasurements(gravity_ft_frame, m_ft_avg);
	}

	void getCalib(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias, Eigen::Vector3d &t_bias)
	{

		Eigen::VectorXd ft_calib = m_ft_calib->getCalib();
       // Eigen::Vector3d center_mass_position;
		mass = fabs(ft_calib(0)); // original
		//mass = fabs(ft_calib(0));
	        density_object = mass/grasp_shape_now.v;
		Eigen::Vector3d center_mass_position(ft_calib(1)/mass,
				ft_calib(2)/mass,
				fabs(ft_calib(3))/mass);

		COM_pos = center_mass_position;

		f_bias(0) = -ft_calib(4);
		f_bias(1) = -ft_calib(5);
		f_bias(2) = -ft_calib(6);
		t_bias(0) = -ft_calib(7);
		t_bias(1) = -ft_calib(8);
		t_bias(2) = -ft_calib(9);
		cog_estim_start=true;
        
	}

	void averageFTMeas()
	{

		if(m_ft_counter==0)
		{

			m_ft_avg = m_ft_raw;

		}

		else
		{

			m_ft_avg.wrench.force.x = m_ft_avg.wrench.force.x + m_ft_raw.wrench.force.x;
			m_ft_avg.wrench.force.y = m_ft_avg.wrench.force.y + m_ft_raw.wrench.force.y;
			m_ft_avg.wrench.force.z = m_ft_avg.wrench.force.z + m_ft_raw.wrench.force.z;

			m_ft_avg.wrench.torque.x = m_ft_avg.wrench.torque.x + m_ft_raw.wrench.torque.x;
			m_ft_avg.wrench.torque.y = m_ft_avg.wrench.torque.y + m_ft_raw.wrench.torque.y;
			m_ft_avg.wrench.torque.z = m_ft_avg.wrench.torque.z + m_ft_raw.wrench.torque.z;

		}
		m_ft_counter++;
	}
	
	void take_hand_stopped(const grasp_estimator::WeightEstimated::ConstPtr &mass_now)
{
   this->hand_stopped = mass_now->hand_stopped;
   this->mass_input = mass_now->mass;
}

void publishDataState()
{ 
  this->data_pub.time_now=ros::Time::now();
  vis_pub.publish(this->position_hand_msgs);
  data_estimated_pub.publish(this->data_pub);
  //this->data_pub.hand_stopped=false;
  this->data_pub.cog_stop=false;
}


private:

	//move_group_interface::MoveGroup *m_group;

	//unsigned int m_pose_counter;
	unsigned int m_ft_counter;

	bool m_finished;

	bool m_received_ft;
	bool m_received_imu;

	// ft calib stuff
	FTSensorCalib *m_ft_calib;

	// expressed in FT sensor frame
	geometry_msgs::WrenchStamped m_ft_raw;
	geometry_msgs::WrenchStamped m_ft_avg; // average over 100 measurements

	// accelerometer readings
	sensor_msgs::Imu m_imu;

	tf::TransformListener *m_tf_listener;

	//	***** ROS parameters ***** //
	// name of the moveit group
	std::string m_moveit_group_name;

	// name of output calibration file
	std::string m_calib_file_name;

	// name of output directory
	// default: ~/.ros/ft_calib
	std::string m_calib_file_dir;

    // name of file with recorded gravity and F/T measurements
    std::string m_meas_file_name;

    // name of directory for saving gravity and F/T measurements
    // default: ~/.ros/ft_calib
    std::string m_meas_file_dir;

	// frame id of the poses to be executed
	std::string m_poses_frame_id;

	// if the user wants to execute just random poses
	// default: false
	bool m_random_poses;

	// number of random poses
	// default: 30
	

};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "cog_estimation_node");
  ROS_INFO("Cog estimation server is Here ");
  ros::NodeHandle nh;

 
  COGEstimNode cog_estim_node;
 
  if(!cog_estim_node.getROSParameters())
  {
   cog_estim_node.n_.shutdown();
   ROS_ERROR("Error getting ROS parameters");
  }
    cog_estim_node.init();
    /// main loop
    double loop_rate_;
    cog_estim_node.n_.param("loop_rate", loop_rate_, 1000.0);
    ros::Rate loop_rate(loop_rate_); // Hz

    // waiting time after end of each pose to take F/T measurements
    double wait_time;
    cog_estim_node.n_.param("wait_time", wait_time, 10.0);
    bool ret = false;
    unsigned int n_measurements = 0;
    ros::Time t_end_move_arm = ros::Time::now();
    cog_estim_node.hand_stopped=false;
    //cog_estim_node.data_pub.calibration=false;
    cog_estim_node.position_hand_msgs.header.frame_id = "left_measure";
    cog_estim_node.position_hand_msgs.header.stamp = ros::Time();
    cog_estim_node.position_hand_msgs.ns = "cog_estimation_node";
    cog_estim_node.position_hand_msgs.id = 0;
    while (cog_estim_node.n_.ok() && !cog_estim_node.finished())
    {
      cog_estim_node.publishDataState();
      //		Move the arm, then calibrate sensor
//       if(cog_estim_node.run_state==false && cog_estim_node.hand_stopped==true )
//       {
// 	    	
//     	cog_estim_node.m_group->setPlannerId("PRMstarkConfigDefault");
//  		ROS_INFO_STREAM("Arm state desidered:  dual_hand_arm_exp");
//  		cog_estim_node.m_group->setNamedTarget("dual_hand_arm_exp");
//  			      // call the move
//  		if( !(cog_estim_node.m_group->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
//  	    {
//  		  ROS_ERROR("An error occured during Moving Arm");
//  	    }
//       }
      //if( cog_estim_node.run_state==true)
      if(cog_estim_node.hand_stopped==true && cog_estim_node.run_state==true)
      {  
	//ROS_INFO("Object detected on the hand");
	if(!ret)
	{
	  ret = cog_estim_node.moveNextPose();
	  t_end_move_arm = ros::Time::now();
	}
	// average 100 measurements to calibrate the sensor in each position
	else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time)
	{
	  n_measurements++;
	  cog_estim_node.averageFTMeas(); // average over 100 measurements;
	  if(n_measurements==100)
	  {
	    ret = false;
	    n_measurements = 0;
	    cog_estim_node.addMeasurement(); // stacks up measurement matrices and FT measurements
	    cog_estim_node.getCalib(cog_estim_node.mass, cog_estim_node.COM_pos, cog_estim_node.f_bias, cog_estim_node.t_bias);
	    if(cog_estim_node.mass!=0.0)
	    {					
	      std::cout << "-------------------------------------------------------------" << std::endl;
	      std::cout << "Current calibration estimate:" << std::endl;
	      std::cout << std::endl << std::endl;
	      std::cout << "Mass: " << cog_estim_node.mass << std::endl << std::endl;
	      ROS_INFO("Object density = %g",cog_estim_node.density_object);
	      
	      std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
	      std::cout << "[" << cog_estim_node.COM_pos(0) << ", " << cog_estim_node.COM_pos(1) << ", " << cog_estim_node.COM_pos(2) << "]";
	      std::cout << std::endl << std::endl;
	      std::cout << "FT bias: " << std::endl;
	      std::cout << "[" << cog_estim_node.f_bias(0) << ", " << cog_estim_node.f_bias(1) << ", " << cog_estim_node.f_bias(2) << ", ";
	      std::cout << cog_estim_node.t_bias(0) << ", " << cog_estim_node.t_bias(1) << ", " << cog_estim_node.t_bias(2) << "]";
	      std::cout << std::endl << std::endl;
	      if(sqrt(pow(cog_estim_node.COM_pos(0),2)+pow(cog_estim_node.COM_pos(1),2)+pow(cog_estim_node.COM_pos(2),2))>1.50 || cog_estim_node.COM_pos(2)<0.0)
	      {
		if(sqrt(pow(cog_estim_node.COM_pos(0),2)+pow(cog_estim_node.COM_pos(1),2)+pow(cog_estim_node.COM_pos(2),2))>1.50 ){ROS_ERROR("Cog too far from the hand");}
		if(cog_estim_node.COM_pos(2)<0.0){ROS_ERROR("Z flipped ! ");}
		//cog_estim_node.m_pose_counter=cog_estim_node.m_pose_counter-1;				
	      }
	      else
	      {	
		//visualization_msgs::Marker position_hand_msgs;
		cog_estim_node.position_hand_msgs.type = visualization_msgs::Marker::SPHERE;
		cog_estim_node.position_hand_msgs.action = visualization_msgs::Marker::ADD;
		cog_estim_node.position_hand_msgs.pose.position.x = (double)cog_estim_node.COM_pos(0);
		cog_estim_node.position_hand_msgs.pose.position.y = (double)cog_estim_node.COM_pos(1);
		cog_estim_node.position_hand_msgs.pose.position.z = (double)cog_estim_node.COM_pos(2);
		cog_estim_node.position_hand_msgs.pose.orientation.x = 0.0;
		cog_estim_node.position_hand_msgs.pose.orientation.y = 0.0;
		cog_estim_node.position_hand_msgs.pose.orientation.z = 0.0;
		cog_estim_node.position_hand_msgs.pose.orientation.w = 1.0;
		cog_estim_node.position_hand_msgs.scale.x = 2*0.01;
		cog_estim_node.position_hand_msgs.scale.y = 2*0.01;
		cog_estim_node.position_hand_msgs.scale.z = 2*0.01;
		cog_estim_node.position_hand_msgs.color.a = 1.0; // Don't forget to set the alpha!
		cog_estim_node.position_hand_msgs.color.r = 1.0f;
		cog_estim_node.position_hand_msgs.color.g = 0.0f;
		cog_estim_node.position_hand_msgs.color.b = 0.0f;
		cog_estim_node.position_hand_msgs.lifetime = ros::Duration(10.0);
		//vis_pub.publish(position_hand_msgs);
	   
		//grasp_estimator::WeightEstimated data_pub;
		//cog_estim_node.data_pub.calibration=true;
		cog_estim_node.data_pub.mass_2=cog_estim_node.mass;
		cog_estim_node.data_pub.time_now=ros::Time();
		cog_estim_node.data_pub.cog_stop=true;
		cog_estim_node.data_pub.cx=(double)cog_estim_node.COM_pos(0);
		cog_estim_node.data_pub.cy=(double)cog_estim_node.COM_pos(1);
		cog_estim_node.data_pub.cz=(double)cog_estim_node.COM_pos(2);

		// NEW code 
		cog_estim_node.data_pub.gx=(double)cog_estim_node.gravity_ft_frame.vector.x;
		cog_estim_node.data_pub.gy=(double)cog_estim_node.gravity_ft_frame.vector.y;
		cog_estim_node.data_pub.gz=(double)cog_estim_node.gravity_ft_frame.vector.z;
		// end new code

		cog_estim_node.data_pub.density=(double)cog_estim_node.density_object;
		//data_estimated_pub.publish(data_pub);
		
		cog_estim_node.publishDataState();
	   
		cog_estim_node.saveCalibData();
	      }  
	    std::cout << "-------------------------------------------------------------" << std::endl << std::endl << std::endl;
	   }
	    else
	    {
	      ROS_ERROR("Failed Calibration");
	      ROS_INFO("Tring a new estimation");
	      cog_estim_node.position_hand_msgs.type = visualization_msgs::Marker::SPHERE;
	      cog_estim_node.position_hand_msgs.action = visualization_msgs::Marker::ADD;
	      cog_estim_node.position_hand_msgs.pose.position.x = 0;
	      cog_estim_node.position_hand_msgs.pose.position.y = 0;
	      cog_estim_node.position_hand_msgs.pose.position.z = 0;
	      cog_estim_node.position_hand_msgs.pose.orientation.x = 1.0;
	      cog_estim_node.position_hand_msgs.pose.orientation.y = 0.0;
	      cog_estim_node.position_hand_msgs.pose.orientation.z = 0.0;
	      cog_estim_node.position_hand_msgs.pose.orientation.w = 0.0;
	      cog_estim_node.position_hand_msgs.scale.x = 2*0.01;
	      cog_estim_node.position_hand_msgs.scale.y = 2*0.01;
	      cog_estim_node.position_hand_msgs.scale.z = 2*0.01;
	      cog_estim_node.position_hand_msgs.color.a = 1.0; // Don't forget to set the alpha!
	      cog_estim_node.position_hand_msgs.color.r = 0.5f;
	      cog_estim_node.position_hand_msgs.color.g = 0.5f;
	      cog_estim_node.position_hand_msgs.color.b = 0.0f;
	      cog_estim_node.position_hand_msgs.lifetime = ros::Duration(10.0);
   
	   
	      cog_estim_node.data_pub.cog_stop=false;
	      cog_estim_node.data_pub.mass_2=0.0;
	      cog_estim_node.data_pub.time_now=ros::Time();
	      cog_estim_node.data_pub.cx=(double)0.0;
	      cog_estim_node.data_pub.cy=(double)0.0;
	      cog_estim_node.data_pub.cz=(double)0.0;
              cog_estim_node.data_pub.density = (double)0.0;
//	      cog_estim_node.publishDataState();
	    }
	}		
        
         ros::spinOnce();
         //ros::spin();
	  
 	  cog_estim_node.publishDataState();
	  loop_rate.sleep();
        }
      }
//        else
//        {
//   	  if(cog_estim_node.pose_name!="dual_hand_arm_exp"&&cog_estim_node.cog_estim_start==true)
//  	  {
//  	    ROS_INFO("Moving arm to exp position because No object in the hand");
//  	                       cog_estim_node.m_group->setPlannerId("PRMstarkConfigDefault");
//   		   //ROS_INFO_STREAM("Arm state desidered: dual_hand_arm_exp");
//   		   cog_estim_node.m_group->setNamedTarget("dual_hand_arm_exp");
//   			      // call the moves
//                     cog_estim_node.pose_name="dual_hand_arm_exp";
//  		   cog_estim_node.m_pose_counter=0;
//   		   if( !(cog_estim_node.m_group->move()==moveit_msgs::MoveItErrorCodes::SUCCESS) )
//   			{
//   			 ROS_ERROR("An error occured during Moving Arm");
//   			}
//   	           
//  	  }
//  	}
    
      
    }	
    return 0;
}



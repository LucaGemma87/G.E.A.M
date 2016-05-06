/*
 * vito_wrench_estimation
 *
 *  Created on: Feb 19, 2016
 *  Authors:  Luca Gemma
 */


#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <utils/pseudo_inversion.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/math/special_functions/sign.hpp>


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>

class VitoWrenchEstimationNode
{

public:

	ros::NodeHandle n_;


	// force estimation from joint efforts
	ros::Publisher topicPub_EstimatedWrench_;
	ros::Publisher topicPub_FilteredWrench_;

	ros::Publisher topicPub_TwistError_;
	ros::Publisher topicPub_FilteredTwistError_;
	
	ros::Publisher pub_hand_cog_;

	ros::Subscriber topicSub_JointStates_;

	ros::Subscriber topicSub_JointTrajControllerState_;
        
	ros::Subscriber topicSub_JointStateEstimator_;
	
        std::vector<double> r;
	std::vector<double> Ki;
	
	 std::vector<double> dot_ext;
	std::vector<double>  int_ext;
	
	double mass;
        
	double loop_freq;
	
	VitoWrenchEstimationNode()
	{
		n_=  ros::NodeHandle("~");
		topicPub_EstimatedWrench_ = n_.advertise<geometry_msgs::WrenchStamped>("estimated_wrench", 1);
		topicPub_FilteredWrench_ = n_.advertise<geometry_msgs::WrenchStamped>("filtered_wrench", 1);

		topicPub_TwistError_ = n_.advertise<geometry_msgs::TwistStamped>("twist_error", 1);
		topicPub_FilteredTwistError_ = n_.advertise<geometry_msgs::TwistStamped>("filtered_twist_error", 1);
		
		pub_hand_cog_ = n_.advertise<visualization_msgs::Marker>(n_.resolveName("/cog_pub_"),5);
		

		topicSub_JointStates_ = n_.subscribe("/left_arm/joint_states", 10, &VitoWrenchEstimationNode::topicCallback_JointStates, this);
		//topicSub_JointStates_ = n_.subscribe("/lwr/joint_states", 10, &VitoWrenchEstimationNode::topicCallback_JointStates, this);
// // 		topicSub_JointTrajControllerState_ = n_.subscribe("/left_arm/joint_trajectory_controller/state", 10,
// 				&VitoWrenchEstimationNode::topicCallback_JointTrajControllerState, this);
 		//topicSub_JointTrajControllerState_ = n_.subscribe("/lwr/vs_computed_torque_controller/state", 10,
		//		&VitoWrenchEstimationNode::topicCallback_JointTrajControllerState, this);
		topicSub_JointTrajControllerState_ = n_.subscribe("/left_arm/vs_computed_torque_controller/state", 10,
				&VitoWrenchEstimationNode::topicCallback_JointTrajControllerState, this);
		//topicSub_JointTrajControllerState_ = n_.subscribe("/lwr/computed_torque_controller/state", 10,
		//		&VitoWrenchEstimationNode::topicCallback_JointTrajControllerState, this);
		
		topicSub_JointStateEstimator_ =n_.subscribe("/lwr/joint_state_estimator/estimated_joint_state",10,&VitoWrenchEstimationNode::topicCallback_JointStateEstimator,this);

		m_initialized = false;
		m_received_js = false;
		m_received_jcs = true;
		m_received_jacc = false;
	}

	~VitoWrenchEstimationNode()
	{
		delete m_JntToJacSolver;
	}

	void getROSParameters()
	{
		n_.param("chain_tip_frame", m_chain_tip_frame, std::string("left_arm_7_link"));
		n_.param("chain_root_frame", m_chain_root_frame, std::string("left_arm_base_link"));

		{
			XmlRpc::XmlRpcValue filter_coeff_b_xml;
			if(n_.hasParam("filter_coeff_b_effort"))
			{
				n_.getParam("filter_coeff_b_effort", filter_coeff_b_xml);

				if(filter_coeff_b_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_b_effort parameter invalid size, setting default value:");
					m_filter_coeff_b_effort = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
					std::cout << m_filter_coeff_b_effort << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_b_xml.size(); i++)
						m_filter_coeff_b_effort(i) = (double) filter_coeff_b_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_b_effort parameter not found, setting default value");
				m_filter_coeff_b_effort = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
				std::cout << m_filter_coeff_b_effort << std::endl;
			}

			XmlRpc::XmlRpcValue filter_coeff_a_xml;
			if(n_.hasParam("filter_coeff_a_effort"))
			{
				n_.getParam("filter_coeff_a_effort", filter_coeff_a_xml);

				if(filter_coeff_a_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_a_effort parameter invalid size, setting default value:");
					m_filter_coeff_a_effort = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
					std::cout << m_filter_coeff_a_effort << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_a_xml.size(); i++)
						m_filter_coeff_a_effort(i) = (double) filter_coeff_a_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_a_effort parameter not found, setting default value:");
				m_filter_coeff_a_effort = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
				std::cout << m_filter_coeff_a_effort << std::endl;
			}
		}

		{
			XmlRpc::XmlRpcValue filter_coeff_b_xml;
			if(n_.hasParam("filter_coeff_b_vel"))
			{
				n_.getParam("filter_coeff_b_vel", filter_coeff_b_xml);

				if(filter_coeff_b_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_b_vel parameter invalid size, setting default value:");
					m_filter_coeff_b_vel = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
					std::cout << m_filter_coeff_b_vel << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_b_xml.size(); i++)
						m_filter_coeff_b_vel(i) = (double) filter_coeff_b_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_b_vel parameter not found, setting default value:");
				m_filter_coeff_b_vel = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
				std::cout << m_filter_coeff_b_vel << std::endl;
			}

			XmlRpc::XmlRpcValue filter_coeff_a_xml;
			if(n_.hasParam("filter_coeff_a_vel"))
			{
				n_.getParam("filter_coeff_a_vel", filter_coeff_a_xml);

				if(filter_coeff_a_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_a_vel parameter invalid size, setting default value:");
					m_filter_coeff_a_vel = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
					std::cout << m_filter_coeff_a_vel << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_a_xml.size(); i++)
						m_filter_coeff_a_vel(i) = (double) filter_coeff_a_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_a_vel parameter not found, setting default value:");
				m_filter_coeff_a_vel = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
				std::cout << m_filter_coeff_a_vel << std::endl;
			}
		}

	}

	void Init()
	{

		ROS_INFO("---------------------");
		ROS_INFO("Initializing chain from %s to %s", m_chain_root_frame.c_str(),
				m_chain_tip_frame.c_str());
		if(!GetKDLChain())
		{
			return;
		}

		std::cout << std::endl;
		ROS_INFO("DOF in the chain: %d", kdl_chain.getNrOfJoints());
		ROS_INFO("Joint names: ");
		for(unsigned int i=0; i<kdl_chain.getNrOfSegments(); i++)
		{
			if( kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
			{
				m_joint_names.push_back(kdl_chain.getSegment(i).getJoint().getName());
				ROS_INFO("%s", m_joint_names.back().c_str());
			}

		}
		ROS_INFO("Chain initialized");
		

		m_DOF = m_joint_names.size();

		m_joint_pos = std::vector<double>(m_DOF, 0.0);
		m_joint_pos_dot = std::vector<double>(m_DOF, 0.0);
		m_joint_efforts = std::vector<double>(m_DOF, 0.0);
		m_joint_efforts_required = std::vector<double>(m_DOF, 0.0);

		m_joint_controller_pos = std::vector<double>(m_DOF, 0.0);
		m_joint_vel_error = std::vector<double>(m_DOF, 0.0);
                
		m_joint_accel = std::vector<double>(m_DOF,0.0);
		
		Ki = std::vector<double>(m_DOF, 10.0);
		r = std::vector<double>(m_DOF, 0.0);
		
		dot_ext =std::vector<double>(m_DOF, 0.0);
		int_ext =std::vector<double>(m_DOF, 0.0);

		m_JntToJacSolver = new KDL::ChainJntToJacSolver(kdl_chain);
		        // Get the gravity vector (direction and magnitude)
                gravity_ = KDL::Vector::Zero();
                gravity_(2) = -9.81;
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain, gravity_) );
		M_.resize(kdl_chain.getNrOfJoints());
		C_.resize(kdl_chain.getNrOfJoints());
		G_.resize(kdl_chain.getNrOfJoints());
		ROS_INFO("Param initializated");
               ROS_INFO("----------------------");
		m_initialized = true;
	}

	bool GetKDLChain()
	{
		// first get the tree from the URDF
		KDL::Tree tree;


		/// Get robot_description from ROS parameter server
		std::string param_name = "robot_description";
		std::string full_param_name;
		std::string xml_string;

		n_.searchParam(param_name, full_param_name);
		if (n_.hasParam(full_param_name))
		{
			n_.getParam(full_param_name.c_str(), xml_string);
		}

		else
		{
			ROS_ERROR("Robot description parameter not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if (xml_string.size() == 0)
		{
			ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
			n_.shutdown();
			return false;
		}
		ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

		/// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR("Failed to parse urdf file");
			n_.shutdown();
			return false;
		}
		ROS_DEBUG("Successfully parsed urdf file");

		if (!kdl_parser::treeFromUrdfModel(model, tree)){
			ROS_ERROR("Failed to construct kdl tree");
			n_.shutdown();
			return false;
		}

		if(!tree.getChain(m_chain_root_frame, m_chain_tip_frame, kdl_chain))
		{
			ROS_ERROR("Error getting chain from %s to %s, shutting down node ...", m_chain_root_frame.c_str(), m_chain_tip_frame.c_str());
			n_.shutdown();
			return false;
		}

		return true;
	}

	void topicCallback_JointStates(const sensor_msgs::JointStateConstPtr &msg)
	{
		if(!m_initialized)
		{
			ROS_WARN("Node not initialized");
			return;
		}

		ROS_DEBUG("In joint state callback");
		unsigned int joint_count = 0;

		for(unsigned int i=0; i<m_DOF; i++)
		{
			for(unsigned int j=0; j<msg->name.size(); j++)
			{
				if(m_joint_names[i] == (std::string)(msg->name[j].data()))
				{
					m_joint_pos[i] = msg->position[j];
					m_joint_pos_dot[i] =msg->velocity[j];
					//m_joint_efforts[i] = -1*msg->effort[j];
					m_joint_efforts[i] =msg->effort[j];
					joint_count++;
				}
			}
		}

		if(joint_count==m_DOF)
		{
			m_received_js = true;
			SendEstWrench();
		}

		else
		{
			ROS_WARN("Expected %d joint positions in joint_states callback, got %d ",
					m_DOF, joint_count);
		}
	}

	void topicCallback_JointTrajControllerState(const control_msgs::JointTrajectoryControllerStateConstPtr &msg)
	{

		if(!m_initialized)
		{
			ROS_WARN("Node not initialized");
			return;
		}

		ROS_DEBUG("In joint trajectory controller state callback");
		if(msg->actual.positions.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint position msg size %d, got %d.",
					m_DOF, msg->actual.positions.size());
		}

		if(msg->error.velocities.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint velocity error msg size %d, got %d.",
					m_DOF, msg->actual.positions.size());
		}

		if(msg->joint_names.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint names msg size %d, got %d.",
					m_DOF, msg->joint_names.size());
		}

		unsigned int joint_count = 0;
		for(unsigned int i=0; i<m_DOF; i++)
		{
			if(msg->joint_names[i] != m_joint_names[i])
			{
				ROS_ERROR("In joint trajectory controller state CB: expected joint name %s, got %s.",
						m_joint_names[i].c_str(), msg->joint_names[i].c_str());
				break;
			}
			joint_count++;
		}

		if(joint_count == m_DOF)
		{
			m_joint_controller_pos = msg->actual.positions;
			m_joint_vel_error = msg->error.velocities;
			m_joint_efforts_required = msg->actual.effort;

			m_received_jcs = true;
			SendTwistError();
		}

		else
		{
			return;
		}

	}
        
        	void topicCallback_JointStateEstimator(const control_msgs::JointTrajectoryControllerStateConstPtr &msg)
	{

		if(!m_initialized)
		{
			ROS_WARN("Node not initialized");
			return;
		}

// 		ROS_DEBUG("In joint trajectory controller state callback");
// 		if(msg->actual.positions.size()!=m_DOF)
// 		{
// 			ROS_ERROR("In joint trajectory controller state CB: expected joint position msg size %d, got %d.",
// 					m_DOF, msg->actual.positions.size());
// 		}
// 
// 		if(msg->error.velocities.size()!=m_DOF)
// 		{
// 			ROS_ERROR("In joint trajectory controller state CB: expected joint velocity error msg size %d, got %d.",
// 					m_DOF, msg->actual.positions.size());
// 		}
// 
// 		if(msg->joint_names.size()!=m_DOF)
// 		{
// 			ROS_ERROR("In joint trajectory controller state CB: expected joint names msg size %d, got %d.",
// 					m_DOF, msg->joint_names.size());
// 		}

		unsigned int joint_count = 0;
		for(unsigned int i=0; i<m_DOF; i++)
		{
			if(msg->joint_names[i] != m_joint_names[i])
			{
				ROS_ERROR("In joint trajectory controller state CB: expected joint name %s, got %s.",
						m_joint_names[i].c_str(), msg->joint_names[i].c_str());
				break;
			}
			joint_count++;
		}

		if(joint_count == m_DOF)
		{
			
			m_joint_accel = msg->actual.accelerations;

			m_received_jacc = true;
		}

		else
		{
			return;
		}

	}
        
	bool isInitialized()
	{
		return m_initialized;
	}


	// expressed in the base frame
	bool SendEstWrench()
	{
		// first get the jacobian
		KDL::Jacobian J(m_DOF);

		KDL::JntArray q_in,q_dot_in,q_dot_dot_in;
		q_in.resize(m_DOF);
		q_dot_in.resize(m_DOF);
		q_dot_dot_in.resize(m_DOF);
		double Ki_array[m_DOF],r_array[m_DOF];
		

		for(unsigned int i=0; i<m_DOF; i++)
		{
			q_in(i) = m_joint_pos[i];
			q_dot_in(i) = m_joint_pos_dot[i];
			q_dot_dot_in(i) = m_joint_accel[i];
		  /*      Ki_array[i]=Ki[i];
		        r_array[i]=r[i]; */ 
		}

		m_JntToJacSolver->JntToJac(q_in, J);

		Eigen::MatrixXd joint_effort,residuals,joint_effort_required,diff_effort,nominal_effort;
		joint_effort.resize(m_DOF, 1);
		nominal_effort.resize(m_DOF, 1);
		joint_effort_required.resize(m_DOF, 1);
		residuals.resize(m_DOF, 1);
                diff_effort.resize(m_DOF,1);
		
	       id_solver_->JntToMass(q_in, M_);
    	       id_solver_->JntToCoriolis(q_in, q_dot_in, C_);
    	       id_solver_->JntToGravity(q_in, G_);
		for(unsigned int i=0; i<m_DOF; i++){
			joint_effort(i) = m_joint_efforts[i];
			
			//ROS_INFO("joint_effort %d = %f",(int)i,double(joint_effort(i)));
		        joint_effort_required(i) = m_joint_efforts_required[i];
			//ROS_INFO("joint_effort_required %d = %f",(int)i,double(joint_effort_required(i)));
		        
			//ROS_INFO("DIFF effort %d = %f",(int)i,double(diff_effort(i)));
		         
		         //diff_effort(i) = joint_effort_required(i)-joint_effort(i);
			nominal_effort(i)=M_(i,i)*q_dot_dot_in(i)+C_(i)*q_dot_in(i)+G_(i)+M_(i,i)*q_dot_in(i); 
			diff_effort(i) = joint_effort(i)-nominal_effort(i) ;
		}
		
		Eigen::MatrixXd JT = J.data.transpose();
		Eigen::MatrixXd J_pinv;
		pseudo_inverse(JT, J_pinv,true);
		// to calculate wrench in ee frame
		
		
	      
	       
	      
	       
	       Eigen::Matrix<double, 6, 1> estimated_wrench = J_pinv*(diff_effort);
	       
// 	       KDL::Wrench transp_wrench;
// 	       transp_wrench.force(0)=estimated_wrench(0);
// 	       transp_wrench.force(1)=estimated_wrench(1);
// 	       transp_wrench.force(2)=estimated_wrench(2);
// 	       transp_wrench.torque(0)=estimated_wrench(3);
// 	       transp_wrench.torque(1)=estimated_wrench(4);
// 	       transp_wrench.torque(2)=estimated_wrench(5);
// 	       KDL::Vector transp_vector(kdl_chain.getSegment(6).getJoint().JointOrigin());
// 	       //getFrameToTip().p);
// 	       ROS_INFO("x = %g , y = %g ,z = %g",(double)transp_vector[0],(double)transp_vector[1],(double)transp_vector[2]);
// 	       transp_wrench.RefPoint(transp_vector);
// 	       KDL::Rotation  rot_matrix(kdl_chain.getSegment(6).getFrameToTip().M);
// 	       KDL::Vector force_vector(rot_matrix*transp_wrench.force);
// // 	       KDL::Vector torque_vector(rot_matrix*transp_wrench.torque);
// 	       ROS_INFO("Transp Wrench: %g, %g, %g, %g, %g, %g",(double)transp_wrench.force(0),(double)transp_wrench.force(1),(double)transp_wrench.force(2),(double)transp_wrench.torque(0),(double)transp_wrench.torque(1),(double)transp_wrench.torque(2));
// 	       
// 	        ROS_INFO("Rot Wrench: %g, %g, %g, %g, %g, %g",(double)force_vector(0),(double)force_vector(1),(double)force_vector(2),(double)torque_vector(0),(double)torque_vector(1),(double)torque_vector(2));
		//Eigen::Matrix<double, 6, 1> estimated_wrench =JT.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(diff_effort);
                ROS_INFO("Newton = %g",(double)(sqrt(pow(estimated_wrench(0),2)+pow(estimated_wrench(1),2)+pow(estimated_wrench(2),2))));
// 		ROS_INFO("Mass = %g",(double)(sqrt(boost::math::sign(estimated_wrench(0))*pow(estimated_wrench(0),2)+boost::math::sign(estimated_wrench(1))*pow(estimated_wrench(1),2)+boost::math::sign(estimated_wrench(2))*pow(estimated_wrench(2),2)))/(double)9.822);
		
		mass=(sqrt(pow(estimated_wrench(0),2)+pow(estimated_wrench(1),2)+pow(estimated_wrench(2),2)))/9.822;
		ROS_INFO("Mass = %g",(double)(sqrt(pow(estimated_wrench(0),2)+pow(estimated_wrench(1),2)+pow(estimated_wrench(2),2)))/(double)9.822);
		    	// computing Inertia, Coriolis and Gravity matrices

	       
	       // To do add command calculation without object and substrack this to estimated wrench like in "IROS14_EstContactForces.pdf"
	       
	       
// 	        for(unsigned int i=0; i<m_DOF; i++)
// 		{
// 		  //r_dot(i)=Ki(i)*()
// 		  //delta_time
// 		  dot_ext[i] = diff_effort(i)+C_(i)*q_dot_in(i)-G_(i);//+int_ext[i];
// 		  int_ext[i] =int_ext[i] + dot_ext[i]*(1/(loop_freq));
// 		  r[i] = Ki[i]*(M_(i,i)*q_dot_in(i)-int_ext[i]) ;
// 		  residuals(i) = r[i];
// 		  //ROS_INFO("Res di %d = %f ", (int)i,(double)r[i]);
// 		}
// 		
// 		Eigen::Matrix<double, 6, 1> estimated_wrench_2 =JT.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(residuals);
		//for(unsigned int i=0; i<6; i++)
		//{  
		//ROS_INFO("estimated wrench with residual %d = %f ", (int)i,(double)estimated_wrench_2(i));
		//}
		
// 		ROS_INFO("Newton 2 = %g",(double)(sqrt(pow(estimated_wrench_2(0),2)+pow(estimated_wrench_2(1),2)+pow(estimated_wrench_2(2),2))));
// 		ROS_INFO("Mass 2= %g",(double)(sqrt(pow(estimated_wrench_2(0),2)+pow(estimated_wrench_2(1),2)+pow(estimated_wrench_2(2),2)))/(double)9.822);
		
		
 		geometry_msgs::WrenchStamped estimated_wrench_msg;
		estimated_wrench_msg.header.stamp = ros::Time::now();
 		estimated_wrench_msg.header.frame_id = m_chain_tip_frame;
 
  		estimated_wrench_msg.wrench.force.x = estimated_wrench(0);
  		estimated_wrench_msg.wrench.force.y = estimated_wrench(1);
  		estimated_wrench_msg.wrench.force.z = estimated_wrench(2);
  
  		estimated_wrench_msg.wrench.torque.x = estimated_wrench(3);
  		estimated_wrench_msg.wrench.torque.y = estimated_wrench(4);
		estimated_wrench_msg.wrench.torque.z = estimated_wrench(5);
// 		estimated_wrench_msg.wrench.force.x = -transp_wrench.force(0);
//  		estimated_wrench_msg.wrench.force.y = -transp_wrench.force(1);
//  		estimated_wrench_msg.wrench.force.z = -transp_wrench.force(2);
//  
//  		estimated_wrench_msg.wrench.torque.x = -transp_wrench.torque(0);
//  		estimated_wrench_msg.wrench.torque.y = -transp_wrench.torque(1);
//  		estimated_wrench_msg.wrench.torque.z = -transp_wrench.torque(2);
                
// 		geometry_msgs::WrenchStamped estimated_wrench_msg;
// 		estimated_wrench_msg.header.stamp = ros::Time::now();
// 		estimated_wrench_msg.header.frame_id = m_chain_tip_frame;
// 
// 		estimated_wrench_msg.wrench.force.x = wrench.force(0);
// 		estimated_wrench_msg.wrench.force.y = wrench.force(1);
// 		estimated_wrench_msg.wrench.force.z = wrench.force(2);
// 
// 		estimated_wrench_msg.wrench.torque.x = wrench.torque(0);
// 		estimated_wrench_msg.wrench.torque.y = wrench.torque(1);
// 		estimated_wrench_msg.wrench.torque.z = wrench.torque(2);
 		topicPub_EstimatedWrench_.publish(estimated_wrench_msg);

		Eigen::Matrix<double, 6, 1> filtered_wrench;
		FilterWrench(estimated_wrench, filtered_wrench);

// 		geometry_msgs::WrenchStamped filtered_wrench_msg;
// 		filtered_wrench_msg.header = estimated_wrench_msg.header;
// 		filtered_wrench_msg.wrench.force.x = filtered_wrench(0);
// 		filtered_wrench_msg.wrench.force.y = filtered_wrench(1);
// 		filtered_wrench_msg.wrench.force.z = filtered_wrench(2);
// 
// 		filtered_wrench_msg.wrench.torque.x = filtered_wrench(3);
// 		filtered_wrench_msg.wrench.torque.y = filtered_wrench(4);
// 		filtered_wrench_msg.wrench.torque.z = filtered_wrench(5);
                
		geometry_msgs::WrenchStamped filtered_wrench_msg_2;
		filtered_wrench_msg_2.header = estimated_wrench_msg.header;
		filtered_wrench_msg_2.wrench.force.x = filtered_wrench(0);
		filtered_wrench_msg_2.wrench.force.y = filtered_wrench(1);
		filtered_wrench_msg_2.wrench.force.z = filtered_wrench(2);

		filtered_wrench_msg_2.wrench.torque.x = filtered_wrench(3);
		filtered_wrench_msg_2.wrench.torque.y = filtered_wrench(4);
		filtered_wrench_msg_2.wrench.torque.z = filtered_wrench(5);
                
		
		topicPub_FilteredWrench_.publish(filtered_wrench_msg_2);

		return true;
	}

	bool SendTwistError()
	{
		// first get the jacobian
		KDL::Jacobian J(m_DOF);

		KDL::JntArray q_in;
		q_in.resize(m_DOF);

		for(unsigned int i=0; i<m_DOF; i++)
		{
			q_in(i) = m_joint_pos[i];
		}

		m_JntToJacSolver->JntToJac(q_in, J);

		Eigen::Matrix<double, 6, 1> twist_error;
		Eigen::MatrixXd joint_vel_error;
		joint_vel_error.resize(m_DOF, 1);

		for(unsigned int i=0; i<m_DOF; i++)
			joint_vel_error(i) = m_joint_vel_error[i];

		twist_error = J.data*joint_vel_error;


		geometry_msgs::TwistStamped twist_error_msg;
		twist_error_msg.header.stamp = ros::Time::now();
		twist_error_msg.header.frame_id = m_chain_root_frame;

		twist_error_msg.twist.linear.x = twist_error(0);
		twist_error_msg.twist.linear.y = twist_error(1);
		twist_error_msg.twist.linear.z = twist_error(2);

		twist_error_msg.twist.angular.x = twist_error(3);
		twist_error_msg.twist.angular.y = twist_error(4);
		twist_error_msg.twist.angular.z = twist_error(5);

		topicPub_TwistError_.publish(twist_error_msg);

		Eigen::Matrix<double, 6, 1> filtered_twist_error;
		FilterTwist(twist_error, filtered_twist_error);

		geometry_msgs::TwistStamped filtered_twist_error_msg;
		filtered_twist_error_msg.header = twist_error_msg.header;
		filtered_twist_error_msg.twist.linear.x = filtered_twist_error(0);
		filtered_twist_error_msg.twist.linear.y = filtered_twist_error(1);
		filtered_twist_error_msg.twist.linear.z = filtered_twist_error(2);

		filtered_twist_error_msg.twist.angular.x = filtered_twist_error(3);
		filtered_twist_error_msg.twist.angular.y = filtered_twist_error(4);
		filtered_twist_error_msg.twist.angular.z = filtered_twist_error(5);

		topicPub_FilteredTwistError_.publish(filtered_twist_error_msg);


		return true;
	}


	void FilterWrench(const Eigen::Matrix<double, 6, 1> &wrench,
			Eigen::Matrix<double, 6, 1> &filtered_wrench)
	{
		static std::vector<Eigen::Matrix<double, 6, 1> > past_wrench(2, Eigen::Matrix<double, 6, 1>::Zero());
		static std::vector<Eigen::Matrix<double, 6, 1> > past_filtered_wrench(2, Eigen::Matrix<double, 6, 1>::Zero());
                
		 double Fx1,Fy1,Fz1,Tx1,Ty1,Tz1,Fx2,Fy2,Fz2,Tx2,Ty2,Tz2,mass_object;
		mass_object = mass;
		// coefficients of the filter
		// cutting at 0.25 the sampling frequency
		Eigen::Vector3d b = m_filter_coeff_b_effort;
		Eigen::Vector3d a = m_filter_coeff_a_effort;

		filtered_wrench = b(0)*wrench + b(1)*past_wrench[0] + b(2)*past_wrench[1]
		                                                                       - a(1)*past_filtered_wrench[0] - a(2) * past_filtered_wrench[1];


		past_wrench[1] = past_wrench[0];
		past_wrench[0] = wrench;


		past_filtered_wrench[1] = past_filtered_wrench[0];
		past_filtered_wrench[0] = filtered_wrench;
		
		Fx1 = past_filtered_wrench[1][0];
		Fy1 = past_filtered_wrench[1][1];
		Fz1 = past_filtered_wrench[1][2];
		Tx1 = past_filtered_wrench[1][3];
		Ty1 = past_filtered_wrench[1][4];
		Tz1 = past_filtered_wrench[1][5];
		   ROS_INFO("Fx1 = %g, Fy1 = %g, Fz1 = %g,Tx1 = %g,Ty1 = %g,Tz1 = %g.",Fx1,Fy1,Fz1,Tx1,Ty1,Tz1);
 		Fx2 = past_filtered_wrench[0][0];
 		Fy2 = past_filtered_wrench[0][1];
 		Fz2 = past_filtered_wrench[0][2];
 		Tx2 = past_filtered_wrench[0][3];
		Ty2 = past_filtered_wrench[0][4];
 		Tz2 = past_filtered_wrench[0][5];
		   ROS_INFO("Fx2 = %g, Fy2 = %g, Fz2 = %g,Tx2 = %g,Ty2 = %g,Tz2 = %g.",Fx2,Fy2,Fz2,Tx2,Ty2,Tz2);
		   
		   tf::Vector3 Torque(Tx1,Ty1,Tz2);
                   tf::Vector3 cog;
		   
                   tf::Matrix3x3 measure_matrix(0.0,Fz1,-Fy1, -Fz1,0.0,Fx1, Fy2,-Fx2,0.0);
		   cog=measure_matrix.inverse()*Torque/(mass_object);
		    
		   //ROS_INFO("Cog : Xcog = %g, Ycog = %g, Zcog = %g", (double)cog.x(), (double)cog.y(), (double)cog.z());
		   
		   
		   Eigen::MatrixXd cog_2,torque_2;
		   cog_2.resize(3, 1);
		   torque_2.resize(3, 1);
		   torque_2(0)=Tx1;torque_2(1)=Ty1;torque_2(2)=Tz2;
		   Eigen::MatrixXd measure_matrix_2_pinv,measure_matrix_2;
		   measure_matrix_2.resize(3,3);
		   measure_matrix_2(0,0)=0.0;measure_matrix_2(1,1)=0.0;measure_matrix_2(2,2)=0.0;
 	           measure_matrix_2(0,1)=Fz1;measure_matrix_2(0,2)=-Fy1;
 		   measure_matrix_2(1,0)=-Fz1;measure_matrix_2(1,2)=Fx1;
 		   measure_matrix_2(2,0)=Fy2;measure_matrix_2(2,1)=-Fx2;
		   

		
		   pseudo_inverse(measure_matrix_2, measure_matrix_2_pinv,true);
 		   cog_2=measure_matrix_2_pinv*torque_2/mass_object;//mass_object
 		    
 		   static std::vector<Eigen::Matrix<double, 3, 1> > past_cog(2, Eigen::Matrix<double, 3, 1>::Zero());
		   static std::vector<Eigen::Matrix<double, 3, 1> > past_filtered_cog(2, Eigen::Matrix<double, 3, 1>::Zero()); 
 		   Eigen::Matrix<double, 3, 1> filtered_cog;
		   filtered_cog = b(0)*cog_2 + b(1)*past_cog[0] + b(2)*past_cog[1]
		                                                                       - a(1)*past_filtered_cog[0] - a(2) * past_filtered_cog[1];


		   past_cog[1] = past_cog[0];
		   past_cog[0] = cog_2;


		  past_filtered_cog[1] = past_filtered_cog[0];
		  past_filtered_cog[0] = filtered_cog;
 		   
		   
		   ROS_INFO("Cog 2 : Xcog = %g, Ycog = %g, Zcog = %g", (double)filtered_cog(0), (double)filtered_cog(1), (double)filtered_cog(2));
		   
		    visualization_msgs::Marker cog_msgs;
                    cog_msgs.header.frame_id = m_chain_tip_frame;
                    cog_msgs.header.stamp = ros::Time();
                    cog_msgs.ns = "vito_wrench_estimation";
                    cog_msgs.id = 0;
                    cog_msgs.type = visualization_msgs::Marker::SPHERE;
                    cog_msgs.action = visualization_msgs::Marker::ADD;
                    cog_msgs.pose.position.x = (double)filtered_cog(0);
                    cog_msgs.pose.position.y = (double)filtered_cog(1);
                    cog_msgs.pose.position.z = (double)filtered_cog(2);
                    cog_msgs.pose.orientation.x = 0.0;
                    cog_msgs.pose.orientation.y = 0.0;
                    cog_msgs.pose.orientation.z = 0.0;
                    cog_msgs.pose.orientation.w = 1.0;
                    cog_msgs.scale.x = 2*0.005;
                    cog_msgs.scale.y = 2*0.005;
                    cog_msgs.scale.z = 2*0.005;
                    cog_msgs.color.a = 1.0; // Don't forget to set the alpha!
                    cog_msgs.color.r = 0.0f;
                    cog_msgs.color.g = 0.0f;
                    cog_msgs.color.b = 1.0f;
                    cog_msgs.lifetime = ros::Duration(0.2);
                    pub_hand_cog_.publish(cog_msgs);

	}

	void FilterTwist(const Eigen::Matrix<double, 6, 1> &twist,
			Eigen::Matrix<double, 6, 1> &filtered_twist)
	{
		static std::vector<Eigen::Matrix<double, 6, 1> > past_twist(2, Eigen::Matrix<double, 6, 1>::Zero());
		static std::vector<Eigen::Matrix<double, 6, 1> > past_filtered_twist(2, Eigen::Matrix<double, 6, 1>::Zero());

		// coefficients of the filter
		// cutting at 0.25 the sampling frequency
		Eigen::Vector3d b = m_filter_coeff_b_vel;
		Eigen::Vector3d a = m_filter_coeff_a_vel;

		filtered_twist = b(0)*twist + b(1)*past_twist[0] + b(2)*past_twist[1]
		                                                                   - a(1)*past_filtered_twist[0] - a(2) * past_filtered_twist[1];


		past_twist[1] = past_twist[0];
		past_twist[0] = twist;


		past_filtered_twist[1] = past_filtered_twist[0];
		past_filtered_twist[0] = filtered_twist;

	}

private:

	std::vector<double> m_joint_pos,m_joint_pos_dot;
	std::vector<double> m_joint_efforts,m_joint_efforts_required;

	std::vector<double> m_joint_controller_pos;
	std::vector<double> m_joint_vel_error;
	std::vector<double> m_joint_accel;


	std::string m_chain_root_frame;
	std::string m_chain_tip_frame;

	std::vector<std::string> m_joint_names;

        KDL::Chain kdl_chain;
	KDL::ChainJntToJacSolver *m_JntToJacSolver;

	Eigen::Vector3d m_filter_coeff_b_effort;
	Eigen::Vector3d m_filter_coeff_a_effort;

	Eigen::Vector3d m_filter_coeff_b_vel;
	Eigen::Vector3d m_filter_coeff_a_vel;

	bool m_received_js;
	bool m_received_jcs;
	bool m_received_jacc;
	


	bool m_initialized;
	unsigned int m_DOF;
	
	
	
       KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
       KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
       KDL::Vector gravity_; 
       boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

};



int main(int argc, char **argv) {

	ros::init(argc, argv, "vito_wrench_estimation");

	VitoWrenchEstimationNode wrench_estimation_node;
	wrench_estimation_node.getROSParameters();
	wrench_estimation_node.Init();

	if(!wrench_estimation_node.isInitialized())
	{
		ROS_ERROR("The wrench estimation node is not initialized, shutting down node...");
		wrench_estimation_node.n_.shutdown();
	}

	double loop_frequency;
	wrench_estimation_node.n_.param("loop_frequency", loop_frequency, 100.0);
        wrench_estimation_node.loop_freq=loop_frequency;
	ros::Rate loop_rate(loop_frequency);


	while(wrench_estimation_node.n_.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}

}





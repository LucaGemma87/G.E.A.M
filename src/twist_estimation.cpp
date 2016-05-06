/*
 * vito_twist_estimation
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

class VitoTwistEstimationNode
{

public:

	ros::NodeHandle n_;


	// force estimation from joint efforts
	ros::Publisher topicPub_Twist_;
	ros::Publisher topicPub_FilteredTwist_;
	
	ros::Subscriber topicSub_JointStates_;
	
        std::vector<double> r;
	std::vector<double> Ki;
	
	 std::vector<double> dot_ext;
	std::vector<double>  int_ext;
	
	double mass;
        
	double loop_freq;
	
	VitoTwistEstimationNode()
	{
		n_=  ros::NodeHandle("~");


		topicPub_Twist_ = n_.advertise<geometry_msgs::TwistStamped>("ft_sensor_twist", 1);
		topicPub_FilteredTwist_ = n_.advertise<geometry_msgs::TwistStamped>("ft_sensor_twist_filtered", 1);
		
		topicSub_JointStates_ = n_.subscribe("/left_arm/joint_states", 10, &VitoTwistEstimationNode::topicCallback_JointStates, this);
		//topicSub_JointStates_ = n_.subscribe("/lwr/joint_states", 10, &VitoWrenchEstimationNode::topicCallback_JointStates, this);
		


		m_initialized = false;
	}

	~VitoTwistEstimationNode()
	{
		delete m_JntToJacSolver;
	}

	void getROSParameters()
	{
		n_.param("chain_tip_frame", m_chain_tip_frame, std::string("left_measure"));
		n_.param("chain_root_frame", m_chain_root_frame, std::string("left_arm_base_link"));


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
		

		m_JntToJacSolver = new KDL::ChainJntToJacSolver(kdl_chain);

	

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
					joint_count++;
				}
			}
		}

		if(joint_count==m_DOF)
		{
			m_received_js = true;
			SendTwist();
		}

		else
		{
			ROS_WARN("Expected %d joint positions in joint_states callback, got %d ",
					m_DOF, joint_count);
		}
	}


         
	bool isInitialized()
	{
		return m_initialized;
	}


	bool SendTwist()
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

		Eigen::Matrix<double, 6, 1> twist;
		Eigen::MatrixXd joint_vel;
		joint_vel.resize(m_DOF, 1);

		for(unsigned int i=0; i<m_DOF; i++)
			joint_vel(i) = m_joint_pos_dot[i];

		twist = J.data*joint_vel;


		geometry_msgs::TwistStamped twist_msg;
		twist_msg.header.stamp = ros::Time::now();
		twist_msg.header.frame_id = m_chain_tip_frame;

		twist_msg.twist.linear.x = twist(0);
		twist_msg.twist.linear.y = twist(1);
		twist_msg.twist.linear.z = twist(2);

		twist_msg.twist.angular.x = twist(3);
		twist_msg.twist.angular.y = twist(4);
		twist_msg.twist.angular.z = twist(5);

		topicPub_Twist_.publish(twist_msg);

		Eigen::Matrix<double, 6, 1> filtered_twist;
		FilterTwist(twist, filtered_twist);

		geometry_msgs::TwistStamped filtered_twist_msg;
		filtered_twist_msg.header = twist_msg.header;
		filtered_twist_msg.twist.linear.x = filtered_twist(0);
		filtered_twist_msg.twist.linear.y = filtered_twist(1);
		filtered_twist_msg.twist.linear.z = filtered_twist(2);

		filtered_twist_msg.twist.angular.x = filtered_twist(3);
		filtered_twist_msg.twist.angular.y = filtered_twist(4);
		filtered_twist_msg.twist.angular.z = filtered_twist(5);

		topicPub_FilteredTwist_.publish(filtered_twist_msg);


		return true;
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

	std::string m_chain_root_frame;
	std::string m_chain_tip_frame;

	std::vector<std::string> m_joint_names;

        KDL::Chain kdl_chain;
	KDL::ChainJntToJacSolver *m_JntToJacSolver;


	Eigen::Vector3d m_filter_coeff_b_vel;
	Eigen::Vector3d m_filter_coeff_a_vel;

	bool m_received_js;
	bool m_received_jcs;
	bool m_received_jacc;
	


	bool m_initialized;
	unsigned int m_DOF;
	
       boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

};



int main(int argc, char **argv) {

	ros::init(argc, argv, "vito_wrench_estimation");

	VitoTwistEstimationNode twist_estimation_node;
	twist_estimation_node.getROSParameters();
	twist_estimation_node.Init();

	if(!twist_estimation_node.isInitialized())
	{
		ROS_ERROR("The twist estimation node is not initialized, shutting down node...");
		twist_estimation_node.n_.shutdown();
	}

	double loop_frequency;
	twist_estimation_node.n_.param("loop_frequency", loop_frequency, 100.0);
        twist_estimation_node.loop_freq=loop_frequency;
	ros::Rate loop_rate(loop_frequency);


	while(twist_estimation_node.n_.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}

}





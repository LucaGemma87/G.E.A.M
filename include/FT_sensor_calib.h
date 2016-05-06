/*
 *
 *  Least squares calibration of:
 *  - Bias of F/T sensor
 *  - Mass of attached gripper
 *  - Location of the center of mass of the gripper
 *
 *  Requires calibrated accelerometer readings
 *  (calibrated with respect to the robot).
 *
 *
 *  Created on: 2016
 *  Authors:   Luca Gemma
 *           
 */

#ifndef FT_SENSOR_CALIB_H_
#define FT_SENSOR_CALIB_H_
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Core>


// Least Squares calibration of bias of FT sensor and the mass and location of the COM of the gripper
namespace grasp_estimator{
class FTSensorCalib
{
public:

	FTSensorCalib();
	virtual ~FTSensorCalib();


	// adds a F/T measurement and the corresponding measurement matrix from the gravity
	// measurements of the accelerometer
	// gravity is assumed to be expressed in the F/T sensor frame
	virtual void addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
			const geometry_msgs::WrenchStamped &ft_raw);


	// Least squares to estimate the F/T sensor parameters
	// The estimated parameters are :
	// [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
	// m: mass of the gripper
	// [cx, cy, cz] are the coordinates of the center of mass of the gripper
	// FB : force bias
	// TB: torque bias
	// All expressed in the FT sensor frame
	virtual Eigen::VectorXd getCalib();
	void Zero();

	//ros::Publisher pub_cog_;
	
	//ros::Publisher pub_data_estimated_;



protected:

	Eigen::MatrixXd H; // stacked measurement matrices
	Eigen::VectorXd Z; // stacked F/T measurements
	// FT_sensor_frame_acc taken as 0

	unsigned int m_num_meas; // number of stacked measurements;

	// measurement matrix based on "On-line Rigid Object Recognition and Pose Estimation
	//  Based on Inertial Parameters", D. Kubus, T. Kroger, F. Wahl, IROS 2008
        virtual Eigen::MatrixXd getMeasurementMatrix(const geometry_msgs::Vector3Stamped &gravity);
	//void Zero();


	private:
	//void Zero();
};
}


#endif /* INERTIALPARAMESTIMATOR_H_ */

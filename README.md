# G.E.A.M
Grasp State Estimator 

First add to kinematics.yaml in vito_moveit_configuration/config this line:

# my kinematics
right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3  

to use target_pose planning in right_arm_planning


// to launch with real robot
roslaunch vito_description display.launch left_arm_enabled:=true use_robot_sim:=false use_calibration_package:=false
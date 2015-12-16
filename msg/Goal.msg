# simple message for arm planning

int32 decision

string arm_name

geometry_msgs/PoseStamped des_pose

float32[7]  joint_position 
int32[7]    joint_change_position

float32 var_roll_wrist 
float32 var_pitch_wrist 
float32 var_yaw_wrist 
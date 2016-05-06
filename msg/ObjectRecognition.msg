# simple message for object recongnition
#

string object_type

#std_msgs/Float64MultiArray centroid

# object proprierties
float32 weight
float32 mass
float32 density

#float32 mass_2
float32 cx
float32 cy
float32 cz

#grasp_shape
float32 x_0
float32 y_0
float32 z_0
float32 r_1

float32 h
float32 w
float32 de

float32 r_2
float32 r_3

float32 a
float32 b
float32 c
float32 d

float32 v

#hand joint state
sensor_msgs/JointState hand_joint_curr_state

#sinergy joint error
float32 sinergy_joint_error

# wrench 
geometry_msgs/WrenchStamped wrench_now

# IMU gravity data
float32 gx
float32 gy
float32 gz
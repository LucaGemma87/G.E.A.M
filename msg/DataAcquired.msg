#DataAcquired

# Time Acquisition

time time_now



# Current Joint position of the right hand

sensor_msgs/JointState hand_joint_curr_state



# Hand to vito anchor trasform

# index
geometry_msgs/TransformStamped hand_index_distal_link 
geometry_msgs/TransformStamped hand_index_knuckle_link
geometry_msgs/TransformStamped hand_index_middle_link
geometry_msgs/TransformStamped hand_index_proximal_link

#little
geometry_msgs/TransformStamped hand_little_distal_link
geometry_msgs/TransformStamped hand_little_knuckle_link
geometry_msgs/TransformStamped hand_little_middle_link
geometry_msgs/TransformStamped hand_little_proximal_link

#middle
geometry_msgs/TransformStamped hand_middle_distal_link
geometry_msgs/TransformStamped hand_middle_knuckle_link
geometry_msgs/TransformStamped hand_middle_middle_link
geometry_msgs/TransformStamped hand_middle_proximal_link

#ring
geometry_msgs/TransformStamped hand_ring_distal_link
geometry_msgs/TransformStamped hand_ring_knuckle_link
geometry_msgs/TransformStamped hand_ring_middle_link
geometry_msgs/TransformStamped hand_ring_proximal_link

#thumb
geometry_msgs/TransformStamped hand_thumb_distal_link
geometry_msgs/TransformStamped hand_thumb_knuckle_link
geometry_msgs/TransformStamped hand_thumb_proximal_link




#  Force / Torque Sensor

geometry_msgs/WrenchStamped arm_wrench_stamped

geometry_msgs/TransformStamped vito_anchor_2_arm_7_link

geometry_msgs/TransformStamped vito_anchor_2_left_measure
 
# Current Motor position [degree] of the hand

float32 hand_synergy_joint_state_position
float32 hand_synergy_joint_state_velocity
float32 hand_synergy_joint_state_effort
float64 hand_error_position

# Current Motor Current  [A] of the hand

float32 motor_current



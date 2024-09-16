std_msgs/Header 	header

geometry_msgs/Pose  pose
geometry_msgs/Twist twist

bool[4] feet_contacts
geometry_msgs/Vector3[4] feet_pos_targets
geometry_msgs/Vector3[4] feet_vel_targets
geometry_msgs/Vector3[4] feet_acc_targets
geometry_msgs/Vector3[4] feet_wrenches
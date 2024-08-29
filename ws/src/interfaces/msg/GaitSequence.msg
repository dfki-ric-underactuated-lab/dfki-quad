
float32[3] target_position
float32[3] target_velocity
float32[4] target_orientation
float32[3] target_twist
float32 target_height
float32 current_height

bool[11] contact_sequence_0
bool[11] contact_sequence_1
bool[11] contact_sequence_2
bool[11] contact_sequence_3

float32[11] foot_position_sequence_x_0
float32[11] foot_position_sequence_x_1
float32[11] foot_position_sequence_x_2
float32[11] foot_position_sequence_x_3
float32[11] foot_position_sequence_y_0
float32[11] foot_position_sequence_y_1
float32[11] foot_position_sequence_y_2
float32[11] foot_position_sequence_y_3
float32[11] foot_position_sequence_z_0
float32[11] foot_position_sequence_z_1
float32[11] foot_position_sequence_z_2
float32[11] foot_position_sequence_z_3

float32[11] reference_trajectory_position_x
float32[11] reference_trajectory_position_y
float32[11] reference_trajectory_position_z

float32[11] reference_trajectory_orientation_w
float32[11] reference_trajectory_orientation_x
float32[11] reference_trajectory_orientation_y
float32[11] reference_trajectory_orientation_z

float32[11] desired_reference_trajectory_position_x
float32[11] desired_reference_trajectory_position_y
float32[11] desired_reference_trajectory_position_z

float32[11] desired_reference_trajectory_orientation_w
float32[11] desired_reference_trajectory_orientation_x
float32[11] desired_reference_trajectory_orientation_y
float32[11] desired_reference_trajectory_orientation_z

float32[11] reference_trajectory_velocity_x
float32[11] reference_trajectory_velocity_y
float32[11] reference_trajectory_velocity_z

float32[11] reference_trajectory_twist_x
float32[11] reference_trajectory_twist_y
float32[11] reference_trajectory_twist_z

float32[11] swing_time_sequence_0
float32[11] swing_time_sequence_1
float32[11] swing_time_sequence_2
float32[11] swing_time_sequence_3

float32[4] gait_swing_time

uint8 sequence_mode
uint8 KEEP = 0
uint8 MOVE = 1
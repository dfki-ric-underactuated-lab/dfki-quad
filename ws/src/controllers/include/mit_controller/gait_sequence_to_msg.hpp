#pragma once

#include "interfaces/msg/gait_sequence.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/mit_controller_params.hpp"

interfaces::msg::GaitSequence gait_sequence_to_msg(const GaitSequence& sequence) {
  static const int size = 11;  // size from message definition
  assert(MPC_PREDICTION_HORIZON + 1 <= size);
  assert(GAIT_SEQUENCE_SIZE >= size);
  interfaces::msg::GaitSequence msg;
  assert(msg.contact_sequence_0.size() == size);
  msg.target_position[0] = sequence.target_position.x();
  msg.target_position[1] = sequence.target_position.y();
  msg.target_position[2] = sequence.target_position.z();
  msg.target_velocity[0] = sequence.target_velocity.x();
  msg.target_velocity[1] = sequence.target_velocity.y();
  msg.target_velocity[2] = sequence.target_velocity.z();
  msg.target_orientation[0] = sequence.target_orientation.w();
  msg.target_orientation[1] = sequence.target_orientation.x();
  msg.target_orientation[2] = sequence.target_orientation.y();
  msg.target_orientation[3] = sequence.target_orientation.z();
  msg.target_twist[0] = sequence.target_twist.x();
  msg.target_twist[1] = sequence.target_twist.y();
  msg.target_twist[2] = sequence.target_twist.z();
  msg.target_height = sequence.target_height;
  msg.current_height = sequence.current_height;

  // msg.sequence_size = size;

  // msg.contact_sequence_0.resize(size);
  // msg.contact_sequence_1.resize(size);
  // msg.contact_sequence_2.resize(size);
  // msg.contact_sequence_3.resize(size);
  // msg.foot_position_sequence_x_0.resize(size);
  // msg.foot_position_sequence_x_1.resize(size);
  // msg.foot_position_sequence_x_2.resize(size);
  // msg.foot_position_sequence_x_3.resize(size);
  // msg.foot_position_sequence_y_0.resize(size);
  // msg.foot_position_sequence_y_1.resize(size);
  // msg.foot_position_sequence_y_2.resize(size);
  // msg.foot_position_sequence_y_3.resize(size);
  // msg.foot_position_sequence_z_0.resize(size);
  // msg.foot_position_sequence_z_1.resize(size);
  // msg.foot_position_sequence_z_2.resize(size);
  // msg.foot_position_sequence_z_3.resize(size);
  // msg.reference_trajectory_position_x.resize(size);
  // msg.reference_trajectory_position_y.resize(size);
  // msg.reference_trajectory_position_z.resize(size);
  // msg.reference_trajectory_orientation_w.resize(size);
  // msg.reference_trajectory_orientation_x.resize(size);
  // msg.reference_trajectory_orientation_y.resize(size);
  // msg.reference_trajectory_orientation_z.resize(size);
  // msg.desired_reference_trajectory_position_x.resize(size);
  // msg.desired_reference_trajectory_position_y.resize(size);
  // msg.desired_reference_trajectory_position_z.resize(size);
  // msg.desired_reference_trajectory_orientation_w.resize(size);
  // msg.desired_reference_trajectory_orientation_x.resize(size);
  // msg.desired_reference_trajectory_orientation_y.resize(size);
  // msg.desired_reference_trajectory_orientation_z.resize(size);
  // msg.reference_trajectory_velocity_x.resize(size);
  // msg.reference_trajectory_velocity_y.resize(size);
  // msg.reference_trajectory_velocity_z.resize(size);
  // msg.reference_trajectory_twist_x.resize(size);
  // msg.reference_trajectory_twist_y.resize(size);
  // msg.reference_trajectory_twist_z.resize(size);

  // msg.swing_time_sequence_0.resize(size);
  // msg.swing_time_sequence_1.resize(size);
  // msg.swing_time_sequence_2.resize(size);
  // msg.swing_time_sequence_3.resize(size);

  for (int i = 0; i < N_LEGS; ++i) {
    msg.gait_swing_time[i] = sequence.gait_swing_time[i];
  }

  msg.sequence_mode = (uint8_t)sequence.sequence_mode;

  for (int i = 0; i < size; ++i) {
    msg.contact_sequence_0[i] = sequence.contact_sequence[i][0];
    msg.contact_sequence_1[i] = sequence.contact_sequence[i][1];
    msg.contact_sequence_2[i] = sequence.contact_sequence[i][2];
    msg.contact_sequence_3[i] = sequence.contact_sequence[i][3];
    msg.foot_position_sequence_x_0[i] = sequence.foot_position_sequence[i][0].x();
    msg.foot_position_sequence_x_1[i] = sequence.foot_position_sequence[i][1].x();
    msg.foot_position_sequence_x_2[i] = sequence.foot_position_sequence[i][2].x();
    msg.foot_position_sequence_x_3[i] = sequence.foot_position_sequence[i][3].x();
    msg.foot_position_sequence_y_0[i] = sequence.foot_position_sequence[i][0].y();
    msg.foot_position_sequence_y_1[i] = sequence.foot_position_sequence[i][1].y();
    msg.foot_position_sequence_y_2[i] = sequence.foot_position_sequence[i][2].y();
    msg.foot_position_sequence_y_3[i] = sequence.foot_position_sequence[i][3].y();
    msg.foot_position_sequence_z_0[i] = sequence.foot_position_sequence[i][0].z();
    msg.foot_position_sequence_z_1[i] = sequence.foot_position_sequence[i][1].z();
    msg.foot_position_sequence_z_2[i] = sequence.foot_position_sequence[i][2].z();
    msg.foot_position_sequence_z_3[i] = sequence.foot_position_sequence[i][3].z();
    msg.reference_trajectory_position_x[i] = sequence.reference_trajectory_position[i].x();
    msg.reference_trajectory_position_y[i] = sequence.reference_trajectory_position[i].y();
    msg.reference_trajectory_position_z[i] = sequence.reference_trajectory_position[i].z();
    msg.reference_trajectory_orientation_w[i] = sequence.reference_trajectory_orientation[i].w();
    msg.reference_trajectory_orientation_x[i] = sequence.reference_trajectory_orientation[i].x();
    msg.reference_trajectory_orientation_y[i] = sequence.reference_trajectory_orientation[i].y();
    msg.reference_trajectory_orientation_z[i] = sequence.reference_trajectory_orientation[i].z();
    msg.desired_reference_trajectory_position_x[i] = sequence.desired_reference_trajectory_position[i].x();
    msg.desired_reference_trajectory_position_y[i] = sequence.desired_reference_trajectory_position[i].y();
    msg.desired_reference_trajectory_position_z[i] = sequence.desired_reference_trajectory_position[i].z();
    msg.desired_reference_trajectory_orientation_w[i] = sequence.desired_reference_trajectory_orientation[i].w();
    msg.desired_reference_trajectory_orientation_x[i] = sequence.desired_reference_trajectory_orientation[i].x();
    msg.desired_reference_trajectory_orientation_y[i] = sequence.desired_reference_trajectory_orientation[i].y();
    msg.desired_reference_trajectory_orientation_z[i] = sequence.desired_reference_trajectory_orientation[i].z();
    msg.reference_trajectory_velocity_x[i] = sequence.reference_trajectory_velocity[i].x();
    msg.reference_trajectory_velocity_y[i] = sequence.reference_trajectory_velocity[i].y();
    msg.reference_trajectory_velocity_z[i] = sequence.reference_trajectory_velocity[i].z();
    msg.reference_trajectory_twist_x[i] = sequence.reference_trajectory_twist[i].x();
    msg.reference_trajectory_twist_y[i] = sequence.reference_trajectory_twist[i].y();
    msg.reference_trajectory_twist_z[i] = sequence.reference_trajectory_twist[i].z();
    msg.swing_time_sequence_0[i] = sequence.swing_time_sequence[i][0];
    msg.swing_time_sequence_1[i] = sequence.swing_time_sequence[i][1];
    msg.swing_time_sequence_2[i] = sequence.swing_time_sequence[i][2];
    msg.swing_time_sequence_3[i] = sequence.swing_time_sequence[i][3];
  }
  return msg;
}

#pragma once

#include <array>

#include "common/state_interface.hpp"
#include "eigen3/Eigen/Dense"
#include "mit_controller_params.hpp"

struct GaitSequence {
  // Time point of gs creation such that the time dependent value are all valid
  StateInterface::TimePoint time_stamp;
  // all values are represented in world frame
  // current target
  Eigen::Vector3d target_position;
  Eigen::Vector3d target_velocity;
  Eigen::Quaterniond target_orientation;
  Eigen::Vector3d target_twist;
  double target_height;
  double current_height;
  // target sequences
  std::array<std::array<bool, N_LEGS>, GAIT_SEQUENCE_SIZE> contact_sequence;
  std::array<std::array<Eigen::Vector3d, N_LEGS>, GAIT_SEQUENCE_SIZE> foot_position_sequence;
  std::array<Eigen::Vector3d, GAIT_SEQUENCE_SIZE> reference_trajectory_position;
  std::array<Eigen::Quaterniond, GAIT_SEQUENCE_SIZE> reference_trajectory_orientation;
  std::array<Eigen::Vector3d, GAIT_SEQUENCE_SIZE> desired_reference_trajectory_position;
  std::array<Eigen::Quaterniond, GAIT_SEQUENCE_SIZE> desired_reference_trajectory_orientation;
  std::array<Eigen::Vector3d, GAIT_SEQUENCE_SIZE> reference_trajectory_velocity;
  std::array<Eigen::Vector3d, GAIT_SEQUENCE_SIZE> reference_trajectory_twist;
  std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE> swing_time_sequence;
  std::array<double, N_LEGS> gait_swing_time;
  enum Mode { KEEP = 0, MOVE = 1 } sequence_mode;
  // std::array<double, GAIT_SEQUENCE_SIZE> rel_time; // Required? used to be in old message
};
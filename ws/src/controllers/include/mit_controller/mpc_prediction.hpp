#pragma once
#include <array>

#include "Eigen/Core"
#include "mit_controller_params.hpp"

struct MPCPrediction {
  std::array<Eigen::Quaterniond, MPC_PREDICTION_HORIZON + 1> orientation;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> position;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> angular_velocity;
  std::array<Eigen::Vector3d, MPC_PREDICTION_HORIZON + 1> linear_velocity;
  std::array<Eigen::Matrix<double, 13, 1, Eigen::ColMajor>, MPC_PREDICTION_HORIZON + 1> raw_data;
};
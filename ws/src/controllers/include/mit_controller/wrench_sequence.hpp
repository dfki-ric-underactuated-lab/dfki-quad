#pragma once

#include <array>

#include "eigen3/Eigen/Dense"
#include "mit_controller_params.hpp"

struct WrenchSequence {
  std::array<std::array<Eigen::Vector3d, N_LEGS>, MPC_PREDICTION_HORIZON> forces;
};
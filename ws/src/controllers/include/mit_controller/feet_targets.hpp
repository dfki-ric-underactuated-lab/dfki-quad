#pragma once

#include <array>
#include <eigen3/Eigen/Core>

#include "mit_controller_params.hpp"

struct FeetTargets {
  std::array<Eigen::Vector3d, N_LEGS> positions;
  std::array<Eigen::Vector3d, N_LEGS> velocities;
  std::array<Eigen::Vector3d, N_LEGS> accelerations;
};
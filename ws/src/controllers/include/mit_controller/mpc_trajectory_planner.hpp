#pragma once

#include "common/quaternion_operations.hpp"
#include "eigen3/Eigen/Dense"
#include "gait_sequence.hpp"
#include "potato_sim/potato_model.hpp"
#include "target.hpp"

class MPCTrajectoryPlanner {
 public:
  MPCTrajectoryPlanner(double dt,
                       const StateInterface &quad_state,
                       const ModelInterface &quad_model,
                       bool fix_standing_position,
                       double distance_threshold,
                       double angular_threshold,
                       double velocity_threshold);
  void plan_trajectory(GaitSequence &sequence, const Target &target);

 private:
  void plan_desired_trajectory(GaitSequence &sequence, const Target &target);
  void set_sequence_mode(GaitSequence &sequence, const Target &target);
  void set_current_target(GaitSequence &sequence, const Target &target);

  double dt_;
  const StateInterface &quad_state_;  // Automatically updates as its a reference
  const ModelInterface &quad_model_;
  bool fix_standing_position_;
  bool sequence_initialized_;
  double distance_threshold_;
  double angular_threshold_;
  double velocity_threshold_;
  std::array<double, N_LEGS> foot_heights_;
};
#pragma once

#include <memory>

#include "SwingTrajectory.hpp"
#include "swing_leg_controller_interface.hpp"

class SwingLegController : public SwingLegControllerInterface {
 private:
  // Parameters
  double swing_height_;
  double world_blend_;
  double maximum_swing_leg_progress_to_update_target_;

  // Swing trajectory related stuff
  std::array<FootSwingTrajectory, N_LEGS> swing_trajectories_;
  StateInterface::TimePoint time_of_gait_sequence_;
  std::array<StateInterface::TimePoint::duration, N_LEGS> target_swing_times_;
  std::array<StateInterface::TimePoint::duration, N_LEGS> still_to_swing_when_gait_sequence_received;

  // Progress related stuff
  std::array<double, N_LEGS> progress_;
  std::array<LegState, N_LEGS> leg_states_;

  // State related stuff
  std::unique_ptr<ModelInterface> quad_model_;
  std::unique_ptr<StateInterface> quad_state_;

  void updateProgress(unsigned int leg_idx);

 public:
  SwingLegController(double swing_height,
                     double maximum_swing_leg_progress_to_update_target,
                     double world_blend,
                     std::unique_ptr<ModelInterface> quad_model,
                     std::unique_ptr<StateInterface> quad_state);

  void UpdateGaitSequence(const GaitSequence &gs) override;
  void UpdateState(const StateInterface &state) override;
  void UpdateModel(const ModelInterface &model) override;
  void GetFeetTargets(FeetTargets &feet_targets) override;
  void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS> &start_pos,
                       std::array<Eigen::Vector3d, N_LEGS> &end_pos) override;
  void GetProgress(std::array<double, N_LEGS> &progress, std::array<LegState, N_LEGS> &swing_states) override;
  void SetSwingHeight(double swing_height);
  void SetWorldBlend(double world_blend);
  void SetMaximumSwingProgressToUpdateTarget(double maximum_swing_leg_progress_to_update_target);
};
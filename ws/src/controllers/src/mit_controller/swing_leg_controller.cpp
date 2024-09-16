
#include "swing_leg_controller.hpp"

#include <iostream>

void SwingLegController::UpdateGaitSequence(const GaitSequence &gs) {
  // For each foot, search for begin and end of following (or current) flight phase.
  time_of_gait_sequence_ = gs.time_stamp;
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    int start_idx = GAIT_SEQUENCE_SIZE;
    int end_idx = GAIT_SEQUENCE_SIZE;
    // Find first index of flight phase
    for (unsigned int sequence_idx = 0; sequence_idx < GAIT_SEQUENCE_SIZE; sequence_idx++) {
      if (!gs.contact_sequence[sequence_idx][leg_idx]) {
        start_idx = sequence_idx;
        break;
      }
    }
    // Find first index after end of flight phase
    for (unsigned int sequence_idx = start_idx; sequence_idx < GAIT_SEQUENCE_SIZE; sequence_idx++) {
      if (gs.contact_sequence[sequence_idx][leg_idx]) {
        end_idx = sequence_idx;
        break;
      }
    }
    // Update the states
    if (start_idx > 0) {  // GS schedules the leg to stand at this current time stamp
      switch (leg_states_[leg_idx]) {
        case NOT_STARTED:
          std::cout << "GS scheduled leg to stance before it ever started to move";
          [[fallthrough]];
        case STANCE:  // Keep leg standing
          [[fallthrough]];
        case SWINGING:  // Early contact
          [[fallthrough]];
        case REACHED:  // Normal end
          leg_states_[leg_idx] = STANCE;
          break;
      }
    } else {  // GS schedules this leg to be in flight
      switch (leg_states_[leg_idx]) {
        case REACHED:
          // Reached but as there was no zero in between, we stay in reached
          leg_states_[leg_idx] = REACHED;
          break;
        case STANCE:  // First occurrence of swing phase
          [[fallthrough]];
        case NOT_STARTED:
          leg_states_[leg_idx] = NOT_STARTED;  // Still not started
          break;
        case SWINGING:
          leg_states_[leg_idx] = SWINGING;  // Keep swinging
          break;
      }
    }
    // Update the swing times:
    target_swing_times_[leg_idx] = std::chrono::duration_cast<StateInterface::TimePoint::duration>(
        std::chrono::duration<double>(gs.gait_swing_time[leg_idx]));
    still_to_swing_when_gait_sequence_received[leg_idx] =
        std::chrono::duration_cast<StateInterface::TimePoint::duration>(
            std::chrono::duration<double>(gs.swing_time_sequence[0][leg_idx]));

    // Update progresses
    updateProgress(leg_idx);

    if (leg_states_[leg_idx] == STANCE) {  // If stance phase it is safe to update the swing height
      swing_trajectories_[leg_idx].setHeight(swing_height_);
      swing_trajectories_[leg_idx].setWorldBlend(world_blend_);
    }

    // Now update the respective swing_trajectories accordingly
    // Initial positions
    if (leg_states_[leg_idx] == NOT_STARTED) {  // Set to current positions
      swing_trajectories_[leg_idx].setInitialPosition(quad_model_->GetFootPositionInWorld(leg_idx, *quad_state_));
    } else if (leg_states_[leg_idx] == STANCE) {  // Schedule for next phase
      swing_trajectories_[leg_idx].setInitialPosition(gs.foot_position_sequence[start_idx - 1][leg_idx]);
    }
    // Target positions
    if (leg_states_[leg_idx] == NOT_STARTED or leg_states_[leg_idx] == STANCE
        or (leg_states_[leg_idx] == SWINGING
            and (progress_[leg_idx] <= maximum_swing_leg_progress_to_update_target_))) {
      swing_trajectories_[leg_idx].setFinalPosition(gs.foot_position_sequence[end_idx][leg_idx]);
    }
  }
}

void SwingLegController::updateProgress(unsigned int leg_idx) {
  if (leg_states_[leg_idx] == SWINGING or leg_states_[leg_idx] == NOT_STARTED) {
    // Update progress
    auto time_since_gait_sequence = quad_state_->GetTime() - time_of_gait_sequence_;
    auto still_to_swing = still_to_swing_when_gait_sequence_received[leg_idx] - time_since_gait_sequence;
    auto time_in_swing_phase = target_swing_times_[leg_idx] - still_to_swing;
    auto phase_percentage = std::chrono::duration_cast<std::chrono::duration<double>>(time_in_swing_phase)
                            / std::chrono::duration_cast<std::chrono::duration<double>>(target_swing_times_[leg_idx]);
    if (phase_percentage >= 1.0) {
      leg_states_[leg_idx] = REACHED;
    }
    progress_[leg_idx] = std::clamp(phase_percentage, 0., 1.);
  } else {
    progress_[leg_idx] = 0.;
  }
}

void SwingLegController::GetProgress(std::array<double, N_LEGS> &progress, std::array<LegState, N_LEGS> &swing_states) {
  progress = progress_;  // TODO: only safe to all after GetFeetTargets
  swing_states = leg_states_;
}

void SwingLegController::GetFeetTargets(FeetTargets &feet_targets) {
  // Return the current targets, based on the current time
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    if (leg_states_[leg_idx] == NOT_STARTED or leg_states_[leg_idx] == SWINGING) {
      swing_trajectories_[leg_idx].computeSwingTrajectoryBezier(
          progress_[leg_idx],
          std::chrono::duration_cast<std::chrono::duration<double>>(target_swing_times_[leg_idx]).count());
      feet_targets.positions[leg_idx] = swing_trajectories_[leg_idx].getPosition();
      feet_targets.velocities[leg_idx] = swing_trajectories_[leg_idx].getVelocity();
      feet_targets.accelerations[leg_idx] = swing_trajectories_[leg_idx].getAcceleration();
      leg_states_[leg_idx] = SWINGING;
    } else if (leg_states_[leg_idx] == REACHED) {
      swing_trajectories_[leg_idx].computeSwingTrajectoryBezier(
          1.0, std::chrono::duration_cast<std::chrono::duration<double>>(target_swing_times_[leg_idx]).count());
      feet_targets.positions[leg_idx] = swing_trajectories_[leg_idx].getPosition();
      feet_targets.velocities[leg_idx] = swing_trajectories_[leg_idx].getVelocity();
      feet_targets.accelerations[leg_idx] = swing_trajectories_[leg_idx].getAcceleration();
    } else {
      feet_targets.positions[leg_idx].setZero();
      feet_targets.velocities[leg_idx].setZero();
      feet_targets.accelerations[leg_idx].setZero();
    }
  }
}

void SwingLegController::GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS> &start_pos,
                                         std::array<Eigen::Vector3d, N_LEGS> &end_pos) {
  for (unsigned int foot_idx = 0; foot_idx < N_LEGS; foot_idx++) {
    start_pos[foot_idx] = swing_trajectories_[foot_idx].getInitialPosition();
    end_pos[foot_idx] = swing_trajectories_[foot_idx].getFinalPosition();
  }
}

SwingLegController::SwingLegController(double swing_height,
                                       double maximum_swing_leg_progress_to_update_target,
                                       double world_blend,
                                       std::unique_ptr<ModelInterface> quad_model,
                                       std::unique_ptr<StateInterface> quad_state)
    : swing_height_(swing_height),
      world_blend_(world_blend),
      maximum_swing_leg_progress_to_update_target_(maximum_swing_leg_progress_to_update_target),
      quad_model_(std::move(quad_model)),
      quad_state_(std::move(quad_state)) {
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    swing_trajectories_[leg_idx].setHeight(swing_height_);
    swing_trajectories_[leg_idx].setWorldBlend(world_blend_);
    progress_[leg_idx] = 0.;
    leg_states_[leg_idx] = STANCE;
  }
}

void SwingLegController::UpdateState(const StateInterface &state) {
  *quad_state_ = state;

  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    updateProgress(leg_idx);
    if (leg_states_[leg_idx] == NOT_STARTED) {
      swing_trajectories_[leg_idx].setInitialPosition(quad_model_->GetFootPositionInWorld(leg_idx, state));
    }
  }
}

void SwingLegController::SetSwingHeight(double swing_height) { swing_height_ = swing_height; }
void SwingLegController::SetWorldBlend(double world_blend) { world_blend_ = world_blend; }

void SwingLegController::SetMaximumSwingProgressToUpdateTarget(double maximum_swing_leg_progress_to_update_target) {
  maximum_swing_leg_progress_to_update_target_ = maximum_swing_leg_progress_to_update_target;
}
void SwingLegController::UpdateModel(const ModelInterface &model) { *quad_model_ = model; }

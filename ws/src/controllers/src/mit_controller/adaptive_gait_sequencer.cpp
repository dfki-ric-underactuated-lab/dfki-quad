#include "adaptive_gait_sequencer.hpp"

#include <math.h>

#include <algorithm>
#include <array>
#include <optional>

#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "mpc_trajectory_planner.hpp"
#include "raibert_foot_step_planner.hpp"
#include "target.hpp"

AdaptiveGaitSequencer::AdaptiveGaitSequencer(const AdaptiveGait& gait,
                                             double k,
                                             const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                                             std::unique_ptr<StateInterface> quad_state,
                                             std::unique_ptr<ModelInterface> quad_model,
                                             unsigned int raibert_filtersize,
                                             bool raibert_z_on_plane,
                                             bool fix_standing_position,
                                             double fix_position_distance_threshold,
                                             double fix_position_angular_threshold,
                                             double fix_position_velocity_threshold,
                                             bool early_contact_detection)
    : target_({}),
      quad_state_(std::move(quad_state)),
      quad_model_(std::move(quad_model)),
      gait_(gait),
      foot_step_planner_(shoulder_positions, *quad_state_, *quad_model_, raibert_filtersize, raibert_z_on_plane, k),
      trajectory_planner_(MPC_DT,
                          *quad_state_,
                          *quad_model_,
                          fix_standing_position,
                          fix_position_distance_threshold,
                          fix_position_angular_threshold,
                          fix_position_velocity_threshold),
      early_contact_detection_(early_contact_detection) {}

void AdaptiveGaitSequencer::GetGaitSequence(GaitSequence& gait_sequence) {
  // update MPC trajectory
  trajectory_planner_.plan_trajectory(gait_sequence, target_);
  // update contact stuff
  if (early_contact_detection_) {
    gait_.update_sequence(gait_sequence,
                          MPC_CONTROL_DT,
                          quad_state_->GetFeetContacts(),
                          foot_step_planner_);  // TODO: calculate real dt instead
  } else {
    gait_.update_sequence(
        gait_sequence, MPC_CONTROL_DT, std::nullopt, foot_step_planner_);  // TODO: calculate real dt instead
  }
  // update footstep positions
  foot_step_planner_.get_foot_position_sequence(gait_sequence, gait_);
  gait_sequence.time_stamp = quad_state_->GetTime();
}

void AdaptiveGaitSequencer::UpdateState(const StateInterface& quad_state) {
  // update model
  *quad_state_ = quad_state;  // The others update automatically as they hold a reference
}

void AdaptiveGaitSequencer::UpdateTarget(const Target& target) { target_ = target; }

void AdaptiveGaitSequencer::GetGaitState(interfaces::msg::GaitState& state) {
  gait_.get_current_period(state.period);
  gait_.get_current_contact(state.contact);
  gait_.get_current_duty_factor(state.duty_factor);
  gait_.get_current_phase(state.phase);
  gait_.get_current_phase_offset(state.phase_offset);
  state.gait_sequencer = (uint8_t)GetType();
}

void AdaptiveGaitSequencer::UpdateModel(const ModelInterface& model) { *quad_model_ = model; }

AdaptiveGait& AdaptiveGaitSequencer::Gait() { return gait_; }
GS_Type AdaptiveGaitSequencer::GetType() const { return ADAPTIVE; }

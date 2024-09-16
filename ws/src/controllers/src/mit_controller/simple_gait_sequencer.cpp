#include "simple_gait_sequencer.hpp"

#include <math.h>

#include <algorithm>
#include <array>

#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "mpc_trajectory_planner.hpp"
#include "raibert_foot_step_planner.hpp"
#include "target.hpp"

SimpleGaitSequencer::SimpleGaitSequencer(const Gait& gait,
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
      trajectory_planner_(gait.get_dt(),
                          *quad_state_,
                          *quad_model_,
                          fix_standing_position,
                          fix_position_distance_threshold,
                          fix_position_angular_threshold,
                          fix_position_velocity_threshold),
      phase_(0.0),
      time_(quad_state_->GetTime()),
      time_initialized_(false),
      early_contact_detection_(early_contact_detection){};

void SimpleGaitSequencer::GetGaitSequence(GaitSequence& gait_sequence) {
  // update MPC trajectory
  trajectory_planner_.plan_trajectory(gait_sequence, target_);
  // update contact and swing timings
  if (early_contact_detection_) {
    gait_.update_sequence(gait_sequence, phase_, quad_state_->GetFeetContacts());
  } else {
    gait_.update_sequence(gait_sequence, phase_);
  }
  // // update contact sequence
  // gait_.get_contact_sequence(gait_sequence, phase_);
  // // update swing timings
  // gait_.get_swing_time_sequence(gait_sequence, phase_);
  // gait_.get_t_swing(gait_sequence);
  // update footstep positions
  foot_step_planner_.get_foot_position_sequence(gait_sequence, gait_);
  gait_sequence.time_stamp = quad_state_->GetTime();
}

void SimpleGaitSequencer::UpdateState(const StateInterface& quad_state) {
  // update model
  *quad_state_ = quad_state;  // footstep planner and trajplanner update automatically as they hold refrence to object
  // update phase
  std::array<double, N_LEGS> duty_factor_ = gait_.get_duty_factor();
  // don't update phase for pure standing gaits
  if (time_initialized_ && std::any_of(duty_factor_.begin(), duty_factor_.end(), [](double value) {
        return value != 0.0 && value != 1.0;
      })) {
    double time_in_phase =
        std::chrono::duration_cast<std::chrono::duration<double>>(quad_state_->GetTime() - time_).count();
    double T = gait_.get_period();
    phase_ = fmod(time_in_phase / T, 1.0);
  } else {
    phase_ = 0.0;
    time_ = quad_state_->GetTime();
    time_initialized_ = true;
  }
}

void SimpleGaitSequencer::UpdateTarget(const Target& target) { target_ = target; }

void SimpleGaitSequencer::GetGaitState(interfaces::msg::GaitState& state) {
  std::array<bool, N_LEGS> contact_state = quad_state_->GetFeetContacts();

  state.period = gait_.get_period();
  for (int i = 0; i < N_LEGS; ++i) {
    state.duty_factor[i] = gait_.get_duty_factor(i);
    state.phase_offset[i] = gait_.get_phase_offset(i);
    state.phase[i] = fmod(phase_ + 1.0 - state.phase_offset[i], 1.0);
    if (early_contact_detection_) {
      // includes early contact detection
      state.contact[i] = state.phase[i] < state.duty_factor[i]
                         || (contact_state[i] && state.phase[i] > ((state.duty_factor[i] + 1.0) / 2.0));
    } else {
      state.contact[i] = state.phase[i] < state.duty_factor[i];
    }
  }
  state.gait_sequencer = (uint8_t)GetType();
}

void SimpleGaitSequencer::UpdateModel(const ModelInterface& model) { *quad_model_ = model; }
GS_Type SimpleGaitSequencer::GetType() const { return SIMPLE; }

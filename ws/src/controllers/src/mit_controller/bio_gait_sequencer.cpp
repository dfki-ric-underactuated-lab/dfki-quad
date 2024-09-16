#include "bio_gait_sequencer.hpp"

#include <math.h>

#include <algorithm>
#include <array>

#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "mpc_trajectory_planner.hpp"
#include "raibert_foot_step_planner.hpp"
#include "target.hpp"

BioGaitSequencer::BioGaitSequencer(double k,
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
      database_(BioGaitDatabase()),
      gait_(database_.getGait(BioGaitDatabase::STAND, MPC_DT)),
      new_gait_(database_.getGait(BioGaitDatabase::STATIC_WALK, MPC_DT)),
      foot_step_planner_(shoulder_positions, *quad_state_, *quad_model_, raibert_filtersize, raibert_z_on_plane, k),
      trajectory_planner_(MPC_DT,
                          *quad_state_,
                          *quad_model_,
                          fix_standing_position,
                          fix_position_distance_threshold,
                          fix_position_angular_threshold,
                          fix_position_velocity_threshold),
      phase_(0.0),
      time_{quad_state_->GetTime()},
      time_initialized_(false),
      transition_(false),
      froude_(0.0),
      transition_start_(time_),
      transition_time_(0.0),
      early_contact_detection_(early_contact_detection){};

void BioGaitSequencer::GetGaitSequence(GaitSequence& gait_sequence) {
  // update MPC trajectory
  trajectory_planner_.plan_trajectory(gait_sequence, target_);
  // check for transition
  if (!transition_) {
    // use planned velocity for froude number
    double v = gait_sequence.reference_trajectory_velocity[1].segment(0, 2).norm();
    double h = gait_sequence.reference_trajectory_position[1].z();
    froude_ = gait_.get_froude(v, h);
    new_gait_ = database_.getGait(froude_, MPC_DT);
    if (new_gait_ != gait_) {
      transition_ = true;
      // hard switching from stand to static walk // TODO: maybe hard switch to
      // new gait instead
      if (gait_.get_name() == "STAND") {
        gait_ = database_.getGait(BioGaitDatabase::STATIC_WALK, MPC_DT);
        transition_ = false;
      }
    }
  }
  if (transition_) {
    // update contact and swing timings
    if (early_contact_detection_) {
      gait_.update_sequence_transition(
          gait_sequence, phase_, transition_time_, new_gait_, quad_state_->GetFeetContacts());
    } else {
      gait_.update_sequence_transition(gait_sequence, phase_, transition_time_, new_gait_);
    }
    // // update contact sequence
    // gait_.get_contact_transition(gait_sequence, phase_, transition_time_, new_gait_);
    // // update swing timings
    // gait_.get_swing_time_transition(gait_sequence, phase_, transition_time_, new_gait_);
    // gait_.get_t_swing(gait_sequence);

  } else {
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
  }
  // update footstep positions
  foot_step_planner_.get_foot_position_sequence(gait_sequence, gait_);
  gait_sequence.time_stamp = quad_state_->GetTime();
}

void BioGaitSequencer::UpdateState(const StateInterface& quad_state) {
  // update model
  *quad_state_ = quad_state;  // The others update automatically as they hold a reference
  // update phase
  std::array<double, N_LEGS> duty_factor_ = gait_.get_duty_factor();
  // don't update phase for pure standing gaits
  if (time_initialized_ && std::any_of(duty_factor_.begin(), duty_factor_.end(), [](double value) {
        return value != 0.0 && value != 1.0;
      })) {
    double T;
    if (transition_) {
      // transition_time_ =
      //     double((quad_state.GetTime() - transition_start_).count())
      //     / std::nano::den;
      double transition_time_ =
          std::chrono::duration_cast<std::chrono::duration<double>>(quad_state_->GetTime() - transition_start_).count();
      double w1 = gait_.get_w1(new_gait_, transition_time_, froude_);
      double w2 = gait_.get_w2(new_gait_, transition_time_, froude_);
      T = w1 * gait_.get_period() + w2 * new_gait_.get_period();
      if (w2 == 1.0) {
        transition_ = false;
        gait_ = new_gait_;
      }
    } else {
      T = gait_.get_period();
    }
    double time_in_phase =
        std::chrono::duration_cast<std::chrono::duration<double>>(quad_state_->GetTime() - time_).count();
    phase_ = fmod(time_in_phase / T, 1.0);
  } else {
    phase_ = 0.0;
    time_ = quad_state.GetTime();
    time_initialized_ = true;
  }
  // save time
  // time_ = quad_state.GetTime();
  if (!transition_) {
    transition_start_ = quad_state.GetTime();
    transition_time_ = 0.0;
  }
}

void BioGaitSequencer::UpdateTarget(const Target& target) { target_ = target; }

void BioGaitSequencer::GetGaitState(interfaces::msg::GaitState& state) {
  std::array<bool, N_LEGS> contact_state = quad_state_->GetFeetContacts();
  if (transition_) {
    double w1 = gait_.get_w1(new_gait_, transition_time_, froude_);
    double w2 = gait_.get_w2(new_gait_, transition_time_, froude_);

    state.period = w1 * gait_.get_period() + w2 * new_gait_.get_period();
    for (int i = 0; i < N_LEGS; ++i) {
      double phase_error = fmod((new_gait_.get_phase_offset(i) - gait_.get_phase_offset(i)) + 0.5, 1.0) - 0.5;
      state.phase_offset[i] = gait_.get_phase_offset(i) + (phase_error)*w2;
      ;
      state.duty_factor[i] = w1 * gait_.get_duty_factor(i) + w2 * new_gait_.get_duty_factor(i);
    }
  } else {
    state.period = gait_.get_period();
    for (int i = 0; i < N_LEGS; ++i) {
      state.duty_factor[i] = gait_.get_duty_factor(i);
      state.phase_offset[i] = gait_.get_phase_offset(i);
    }
  }
  for (int i = 0; i < N_LEGS; ++i) {
    state.phase[i] = fmod(phase_ + 1.0 - state.phase_offset[i], 1.0);
    if (early_contact_detection_) {
      // includes early footstep detection
      state.contact[i] = state.phase[i] < state.phase_offset[i]
                         || (contact_state[i] && state.phase[i] > ((state.duty_factor[i] + 1.0) / 2.0));
    } else {
      state.contact[i] = state.phase[i] < state.phase_offset[i];
    }
  }
  state.gait_sequencer = (uint8_t)GetType();
}

void BioGaitSequencer::UpdateModel(const ModelInterface& model) { *quad_model_ = model; }
GS_Type BioGaitSequencer::GetType() const { return BIOINSPIRED; }

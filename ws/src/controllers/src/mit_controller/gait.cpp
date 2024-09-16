#include "gait.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>

#include "gait_sequence.hpp"
#include "mit_controller/gait.hpp"
#include "mit_controller_params.hpp"

// ------------- Gait -------------//
Gait::Gait(double period,
           const std::array<double, N_LEGS>& duty_factor,
           const std::array<double, N_LEGS>& phase_offset,
           double dt,
           const std::string& name)
    : name_(name), dt_(dt), T_(period), duty_factor_(duty_factor), phase_offset_(phase_offset){};

Gait::Gait(double period,
           double duty_factor,
           const std::array<double, N_LEGS>& phase_offset,
           double dt,
           const std::string& name)
    : name_(name), dt_(dt), T_(period), phase_offset_(phase_offset) {
  std::fill(duty_factor_.begin(), duty_factor_.end(), duty_factor);
};

void Gait::get_contact_sequence(std::array<std::array<bool, N_LEGS>, GAIT_SEQUENCE_SIZE>& contact_sequence,
                                double phase) const {
  // Iterate over time
  for (unsigned int i = 0; i < GAIT_SEQUENCE_SIZE; i++) {
    //   Iterate over legs
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      bool contact;
      // contact if leg phase < duty factor
      contact = fmod((phase + 1.0 - phase_offset_[leg]) + i * dt_ / T_, 1.0) < duty_factor_[leg];
      contact_sequence[i][leg] = contact;
    }
  }
};

// remaining swing time per leg
void Gait::get_swing_time_sequence(std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE>& swing_time_sequence,
                                   double phase) const {
  // iterate over time
  for (unsigned int i = 0; i < GAIT_SEQUENCE_SIZE; i++) {
    // Iterate over legs
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      double time;
      double leg_phase;
      // current phase of the leg
      leg_phase = fmod((phase + 1.0 - phase_offset_[leg]) + i * dt_ / T_, 1.0);
      // if in swing phase
      if (leg_phase >= duty_factor_[leg] && duty_factor_[leg] > 0) {
        time = (1 - leg_phase) * T_;
      } else {
        // no remaining swing time in stance phase
        time = 0.0;
      }
      swing_time_sequence[i][leg] = time;
    }
  }
}

void Gait::update_sequence(GaitSequence& gait_sequence,
                           double phase,
                           const std::optional<std::array<bool, N_LEGS>>& contact_state) {
  // check for standing gait
  if (std::all_of(duty_factor_.begin(), duty_factor_.end(), [](double i) {
        return i >= (1.0 - std::numeric_limits<double>::epsilon());
      })) {
    gait_sequence.sequence_mode = GaitSequence::KEEP;
  }
  std::array<bool, N_LEGS> early_contact{};
  // iterate over time
  for (unsigned int i = 0; i < GAIT_SEQUENCE_SIZE; i++) {
    // Iterate over legs
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      double time;
      double leg_phase;
      bool contact;
      // current phase of the leg
      leg_phase = fmod((phase + 1.0 - phase_offset_[leg]) + i * dt_ / T_, 1.0);
      contact = leg_phase < duty_factor_[leg];
      // check for early contact in second half of swing phase
      if (i == 0 && contact_state.has_value()) {
        auto& contact_state_array = contact_state.value();
        if (contact_state_array[leg] && !contact && (leg_phase > ((duty_factor_[leg] + 1.0) / 2.0))) {
          early_contact[leg] = true;
        }
      }
      // apply early contact until first planned contact
      if (contact) {
        early_contact[leg] = false;
      } else if (early_contact[leg]) {
        contact = true;
      }
      // if in swing phase
      if (!contact && duty_factor_[leg] > 0) {
        time = (1 - leg_phase) * T_;
      } else {
        // no remaining swing time in stance phase
        time = 0.0;
      }
      gait_sequence.contact_sequence[i][leg] = contact;
      gait_sequence.swing_time_sequence[i][leg] = time;
    }
  }
  get_t_swing(gait_sequence);
}

void Gait::get_contact_sequence(GaitSequence& gait_sequence, double phase) const {
  Gait::get_contact_sequence(gait_sequence.contact_sequence, phase);
};
void Gait::get_swing_time_sequence(GaitSequence& gait_sequence, double phase) const {
  Gait::get_swing_time_sequence(gait_sequence.swing_time_sequence, phase);
};

// Getters
double Gait::get_period() const { return T_; }
std::array<double, N_LEGS> Gait::get_duty_factor() const { return duty_factor_; }
double Gait::get_duty_factor(unsigned int leg) const { return duty_factor_[leg]; }
std::array<double, N_LEGS> Gait::get_phase_offset() const { return this->phase_offset_; }
double Gait::get_phase_offset(unsigned int leg) const { return phase_offset_[leg]; }
double Gait::get_dt() const { return dt_; }

std::string Gait::get_name() const { return name_; }

double Gait::get_t_stance(unsigned int leg) const { return duty_factor_[leg] * T_; }
double Gait::get_t_swing(unsigned int leg) const { return (1 - duty_factor_[leg]) * T_; }
void Gait::get_t_swing(GaitSequence& gait_sequence) const {
  std::array<double, N_LEGS>& t_swing = gait_sequence.gait_swing_time;
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    t_swing[leg] = get_t_swing(leg);
  }
}

// -------------- BioInspiredGait ----------------- //
BioInspiredGait::BioInspiredGait(double period,
                                 const std::array<double, N_LEGS>& duty_factor,
                                 const std::array<double, N_LEGS>& phase_offset,
                                 double dt,
                                 double froude_lb,
                                 double froude_ub,
                                 const std::string& name)
    : Gait(period, duty_factor, phase_offset, dt, name), froude_lb_(froude_lb), froude_ub_(froude_ub){};

BioInspiredGait::BioInspiredGait(double period,
                                 double duty_factor,
                                 const std::array<double, N_LEGS>& phase_offset,
                                 double dt,
                                 double froude_lb,
                                 double froude_ub,
                                 const std::string& name)
    : Gait(period, duty_factor, phase_offset, dt, name), froude_lb_(froude_lb), froude_ub_(froude_ub) {}

void BioInspiredGait::update_sequence_transition(GaitSequence& gait_sequence,
                                                 double phase,
                                                 double time,
                                                 const BioInspiredGait& new_gait,
                                                 const std::optional<std::array<bool, N_LEGS>>& contact_state) {
  std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE>& swing_time_sequence = gait_sequence.swing_time_sequence;
  std::array<std::array<bool, N_LEGS>, GAIT_SEQUENCE_SIZE>& contact_sequence = gait_sequence.contact_sequence;

  double phase_offset;
  double duty_factor;
  double period;
  bool contact;
  std::array<bool, N_LEGS> early_contact_possible{};

  // backward pass to ensure linearity of timings
  for (int i = GAIT_SEQUENCE_SIZE; i >= 0; --i) {
    double v = gait_sequence.reference_trajectory_velocity[i].segment(0, 2).norm();
    double h =
        gait_sequence.reference_trajectory_position[i].z();  // TODO: maybe fix to max leg length, as proposed in paper
    double froude = get_froude(v, h);

    double w1 = get_w1(new_gait, time + i * dt_, froude);
    double w2 = get_w2(new_gait, time + i * dt_, froude);

    period = w1 * T_ + w2 * new_gait.get_period();

    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      duty_factor = w1 * duty_factor_[leg] + w2 * new_gait.get_duty_factor(leg);
      // TODO: evaluate if this is the way to go
      double phase_error = fmod((new_gait.get_phase_offset(leg) - this->get_phase_offset(leg)) + 0.5, 1.0) - 0.5;
      phase_offset = phase_offset_[leg] + (phase_error)*w2;
      double leg_phase = fmod((phase + 1.0 - phase_offset) + i * dt_ / period, 1.0);
      double swing_time;
      contact = leg_phase < duty_factor;

      if (!contact && (leg_phase < ((duty_factor + 1.0) / 2.0))) {
        early_contact_possible[leg] = true;
      } else {
        early_contact_possible[leg] = false;
      }

      if (!contact && duty_factor > 0) {
        if (i < GAIT_SEQUENCE_SIZE - 1 && swing_time_sequence[i + 1][leg] != 0.0) {
          swing_time = swing_time_sequence[i + 1][leg] - dt_;
        } else {
          swing_time = (1 - leg_phase) * period;
        }
      } else {
        // no remaining swing time in stance phase
        swing_time = 0.0;
      }

      contact_sequence[i][leg] = contact;
      swing_time_sequence[i][leg] = swing_time;
    }
  }
  // early contact forward pass
  if (contact_state.has_value()) {
    auto& contact_state_array = contact_state.value();
    for (int leg = 0; leg < N_LEGS; ++leg) {
      int i = 0;
      while (early_contact_possible[leg] && contact_state_array[leg] && i < GAIT_SEQUENCE_SIZE) {
        if (contact_sequence[i][leg]) {
          early_contact_possible[leg] = false;
        } else {
          contact_sequence[i][leg] = true;
          swing_time_sequence[i][leg] = 0.0;
        }
      }
    }
  }
  get_t_swing(gait_sequence);
}

void BioInspiredGait::get_contact_transition(GaitSequence& gait_sequence,
                                             double phase,
                                             double time,
                                             const BioInspiredGait& new_gait) const {
  std::array<std::array<bool, N_LEGS>, GAIT_SEQUENCE_SIZE>& contact_sequence = gait_sequence.contact_sequence;

  double phase_offset;
  double duty_factor;
  double period;
  bool contact;

  for (unsigned int i = 0; i < GAIT_SEQUENCE_SIZE; i++) {
    double v = gait_sequence.reference_trajectory_velocity[i].segment(0, 2).norm();
    double h =
        gait_sequence.reference_trajectory_position[i].z();  // TODO: maybe fix to max leg length, as proposed in paper
    double froude = get_froude(v, h);

    double w1 = get_w1(new_gait, time + i * dt_, froude);
    double w2 = get_w2(new_gait, time + i * dt_, froude);

    period = w1 * T_ + w2 * new_gait.get_period();

    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      duty_factor = w1 * duty_factor_[leg] + w2 * new_gait.get_duty_factor(leg);
      // TODO: evaluate if this is the way to go
      double phase_error = (new_gait.get_phase_offset(leg) - this->get_phase_offset(leg));
      if (phase_error < -0.5)
        phase_error += 1.0;
      else if (phase_error > 0.5) {
        phase_error -= 1.0;
      }
      phase_offset = phase_offset_[leg] + (phase_error)*w2;
      contact = fmod((phase + 1.0 - phase_offset) + i * dt_ / period, 1.0) < duty_factor;
      contact_sequence[i][leg] = contact;
    }
  }
}

void BioInspiredGait::get_swing_time_transition(GaitSequence& gait_sequence,
                                                double phase,
                                                double time,
                                                const BioInspiredGait& new_gait) const {
  std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE>& swing_time_sequence = gait_sequence.swing_time_sequence;

  double phase_offset;
  double duty_factor;
  double period;
  bool contact;
  // backward pass to ensure linearity of timings
  for (int i = GAIT_SEQUENCE_SIZE; i >= 0; --i) {
    double v = gait_sequence.reference_trajectory_velocity[i].segment(0, 2).norm();
    double h =
        gait_sequence.reference_trajectory_position[i].z();  // TODO: maybe fix to max leg length, as proposed in paper
    double froude = get_froude(v, h);

    double w1 = get_w1(new_gait, time + i * dt_, froude);
    double w2 = get_w2(new_gait, time + i * dt_, froude);

    period = w1 * T_ + w2 * new_gait.get_period();

    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      duty_factor = w1 * duty_factor_[leg] + w2 * new_gait.get_duty_factor(leg);
      // TODO: evaluate if this is the way to go
      double phase_error = fmod((new_gait.get_phase_offset(leg) - this->get_phase_offset(leg)) + 0.5, 1.0) - 0.5;
      phase_offset = phase_offset_[leg] + (phase_error)*w2;
      double leg_phase = fmod((phase + 1.0 - phase_offset) + i * dt_ / period, 1.0);
      double swing_time;
      contact = leg_phase < duty_factor;
      if (!contact && duty_factor > 0) {
        if (i < GAIT_SEQUENCE_SIZE - 1 && swing_time_sequence[i + 1][leg] != 0.0) {
          swing_time = swing_time_sequence[i + 1][leg] - dt_;
        } else {
          swing_time = (1 - leg_phase) * period;
        }
      } else {
        // no remaining swing time in stance phase
        swing_time = 0.0;
      }

      swing_time_sequence[i][leg] = swing_time;
    }
  }
}

// Getters
double BioInspiredGait::get_D() const { return T_ / dt_; }  // TODO:: ggf remove dt_
double BioInspiredGait::get_Dtrans(const BioInspiredGait& new_gait) const {
  double C;             // number of gait cycles in transition
  double Cmax = 2.0;    // max number of gait cycles in transition
  double Frcrit = 1.0;  // max froude number
  if (froude_ub_ < new_gait.get_froude_ub()) {
    C = (1 - (froude_ub_ / Frcrit)) * Cmax;
  } else {
    C = (1 - (froude_lb_ / Frcrit)) * Cmax;
  }
  return ((get_D() + new_gait.get_D()) / 2.0) * C;
}

double BioInspiredGait::get_ntrans(const BioInspiredGait& new_gait, double time, double froude) const {
  // TODO: make sense out of equation 7 (maybe like this?)
  double n_trans;
  if (new_gait > *this) {  // acceleration
    n_trans = 1.0 + (froude - new_gait.get_froude_lb()) / (new_gait.get_froude_ub() - new_gait.get_froude_lb());
  } else if (new_gait < *this) {  // deceleration
    n_trans = 1.0 + (new_gait.get_froude_ub() - froude) / (new_gait.get_froude_ub() - new_gait.get_froude_lb());
  } else {  // probably not occurring
    n_trans = 1.0;
  }
  return n_trans * (time / dt_);  // I guess it is n_trans * n
}

double BioInspiredGait::get_w1(const BioInspiredGait& new_gait, double time, double froude) const {
  return 1.0 - get_w2(new_gait, time, froude);
}
double BioInspiredGait::get_w2(const BioInspiredGait& new_gait, double time, double froude) const {
  return std::min(get_ntrans(new_gait, time, froude) / get_Dtrans(new_gait), 1.0);
}

double BioInspiredGait::get_froude_lb() const { return froude_lb_; }
double BioInspiredGait::get_froude_ub() const { return froude_ub_; }

double BioInspiredGait::get_froude(double v, double h) const { return std::abs((v * v) / (GRAVITY_CONSTANT * h)); }

// operators
bool BioInspiredGait::operator==(const BioInspiredGait& other) const {
  return (froude_lb_ == other.froude_lb_ && froude_ub_ == other.froude_ub_);
}
bool BioInspiredGait::operator!=(const BioInspiredGait& other) const { return !(*this == other); }
bool BioInspiredGait::operator<(const BioInspiredGait& other) const { return (froude_lb_ < other.froude_lb_); }
bool BioInspiredGait::operator>(const BioInspiredGait& other) const { return (froude_ub_ > other.froude_ub_); }

// ------------------------- AdaptiveGait ------------------------- //
AdaptiveGait::AdaptiveGait(const std::array<double, N_LEGS>& phase_offset,
                           double dt,
                           double swing_time,
                           unsigned int filter_size,
                           double zero_velocity_threshold,
                           bool switch_offsets,
                           double standing_foot_position_threshold,
                           double min_v_cmd_factor,
                           double max_correction_cycles,
                           bool correct_all,
                           double correction_period,
                           double disturbance_correction,
                           double min_v,
                           double offset_delay,
                           double max_stride_length,
                           const std::string& name)
    : swing_time_(swing_time),
      name_(name),
      dt_(dt),
      T_(correction_period),
      duty_factor_({1.0, 1.0, 1.0, 1.0}),
      phase_offset_(phase_offset),
      new_phase_offset_(phase_offset),
      velocity_filter_(MovingAverage<Eigen::Vector2d>(filter_size)),
      zero_velocity_threshold_(zero_velocity_threshold),
      switch_offsets_(switch_offsets),
      offset_delay_(offset_delay),
      remaining_offset_delay_(-1.0),
      standing_foot_position_threshold_(standing_foot_position_threshold),
      max_correction_cycles_(max_correction_cycles),
      min_v_cmd_factor_(min_v_cmd_factor),
      correct_all_(correct_all),
      correction_period_(correction_period),
      disturbance_correction_(disturbance_correction),
      min_v_(min_v),
      max_stride_length_(max_stride_length) {
  for (int i = 0; i < N_LEGS; ++i) {
    leg_phase_[i] = fmod(1.0 - phase_offset[i], 1.0);
  }
}

double AdaptiveGait::get_t_stance(unsigned int leg) const {
  (void)leg;
  return T_ - swing_time_;
}

void AdaptiveGait::update_sequence(GaitSequence& gait_sequence,
                                   double dt,
                                   const std::optional<std::array<bool, N_LEGS>>& contact_state,
                                   const std::optional<RaibertFootStepPlanner>& raibert) {
  double v, v_cmd, h, wz;
  std::array<double, N_LEGS> df_old = duty_factor_;
  wz = gait_sequence.reference_trajectory_twist.at(0).z();
  // filter 2D velocity vector and take norm from it
  v = velocity_filter_.update(gait_sequence.reference_trajectory_velocity.at(0).segment(0, 2)).norm();
  v_cmd = gait_sequence.target_velocity.segment(0, 2).norm();
  // use velocity of legs due to twist if > v
  v = std::max(v, std::abs(wz * 0.3));  // FIXME: use correct distance
  v = std::max(v, v_cmd * min_v_cmd_factor_);
  v += disturbance_correction_ * disturbance_factor(gait_sequence);  // FIXME: remove/ add param
  v = std::max(v, min_v_);
  h = gait_sequence.current_height;  // TODO: maybe take target height instead
  // h = 0.3;
  update_params(
      T_, duty_factor_, phase_offset_, remaining_offset_delay_, dt, v, h, switch_offsets_, gait_sequence, raibert);
  nextPhase(leg_phase_, dt, T_, phase_offset_, df_old, duty_factor_, contact_state);

  std::array<double, N_LEGS> df = duty_factor_;
  std::array<double, N_LEGS> phase = leg_phase_;
  std::array<double, N_LEGS> phase_offset = phase_offset_;
  double T = T_;
  double remaining_offset_delay = remaining_offset_delay_;

  for (int i = 0; i < GAIT_SEQUENCE_SIZE; ++i) {
    if (i > 0) {
      v = gait_sequence.reference_trajectory_velocity.at(i).segment(0, 2).norm();
      v = std::max(v, min_v_);
      h = gait_sequence.current_height;
      // h = 0.3;
      df_old = df;
      update_params(T, df, phase_offset, remaining_offset_delay, dt_, v, h, false);
      nextPhase(phase, dt_, T, phase_offset, df_old, df);
    }
    for (int leg = 0; leg < N_LEGS; ++leg) {
      // std::cout << "phase[" << i << "," << leg << "]: " << phase[leg]
      //           << " df: " << df[leg] << " T: " << T << std::endl;
      // get contact
      bool contact;
      contact = phase[leg] < df[leg];
      // fill contacts if first timestep initiates swing phase
      if (contact && i > 0 && !gait_sequence.contact_sequence[i - 1][leg]) {
        contact = gait_sequence.swing_time_sequence[i - 1][leg] <= dt_;
      }
      gait_sequence.contact_sequence[i][leg] = contact;
      // get remaining swing time
      double swing_time;
      if (contact) {
        swing_time = 0.0;
      } else {
        if (i == 0 || gait_sequence.contact_sequence[i - 1][leg]) {
          swing_time = (1 - phase[leg]) * T;
        } else {
          swing_time = gait_sequence.swing_time_sequence[i - 1][leg] - dt_;
          if (swing_time < 0.0) {  // TODO: ugly fix. Find reason why sometimes negative
            gait_sequence.contact_sequence[i][leg] = true;
            swing_time = 0.0;
          }
        }
      }
      gait_sequence.swing_time_sequence[i][leg] = swing_time;
    }
  }
  // update max swing time (should be constant)
  for (int leg = 0; leg < N_LEGS; ++leg) {
    gait_sequence.gait_swing_time[leg] = swing_time_;
  }
  // check for standing gait
  if (std::all_of(duty_factor_.begin(), duty_factor_.end(), [](double i) {
        return i >= (1.0 - std::numeric_limits<double>::epsilon());
      })) {
    gait_sequence.sequence_mode = GaitSequence::KEEP;
  }
}

double AdaptiveGait::disturbance_factor(const GaitSequence& gait_sequence) const {
  Eigen::Quaterniond Rwb = gait_sequence.reference_trajectory_orientation.at(0);
  Eigen::Vector3d v = Rwb.conjugate() * gait_sequence.reference_trajectory_velocity.at(0);
  Eigen::Vector3d v_cmd = Rwb.conjugate() * gait_sequence.target_velocity;
  Eigen::Vector3d v_dif;
  for (int i = 0; i < 3; ++i) {
    // if (std::signbit(v(i)) == std::signbit(v_cmd[i])){
    v_dif(i) = std::max(std::abs(v(i)) - std::abs(v_cmd(i)), 0.0);
    // } else {
    // v_dif(i) = std::abs(v(i)) + std::abs(v_cmd(i));
    // }
  }
  v_dif.z() = 0.0;
  return v_dif.norm();
}

void AdaptiveGait::update_params(double& T,
                                 std::array<double, N_LEGS>& duty_factor,
                                 std::array<double, N_LEGS>& phase_offset,
                                 double& remaining_offset_delay,
                                 double dt,
                                 double v,
                                 double h,
                                 bool switch_offsets,
                                 const std::optional<GaitSequence>& gait_sequence,
                                 const std::optional<RaibertFootStepPlanner>& raibert) {
  double Fr = get_froude(v, h);
  (void)Fr;
  double stride_length = (2.3 * std::pow(Fr, 0.3)) * h;
  // double stride_length =
  //     (0.489 + 0.297 * v) * (h / 0.61);  // TODO: choose function
  if (max_stride_length_ > std::numeric_limits<double>::epsilon()) {
    stride_length = std::min(stride_length, max_stride_length_);
  }
  // adapt phase offsets // TODO: tune thresholds!
  if (switch_offsets && remaining_offset_delay < 0.0) {  // FIXME: change
    if (Fr < 0.02) {
      set_offset({0.0, 0.5, 0.75, 0.25}, true);
    } else {
      set_offset({0.0, 0.5, 0.5, 0.0}, true);
    }
  }
  if (remaining_offset_delay > 0.0) {
    remaining_offset_delay -= dt;
    if (remaining_offset_delay <= 0.0) {
      phase_offset = new_phase_offset_;
    }
  }

  if (std::abs(v) > (zero_velocity_threshold_ + std::numeric_limits<double>::epsilon())) {
    T = stride_length / (std::abs(v));  // TODO: maybe clip
    for (int i = 0; i < N_LEGS; ++i) {
      duty_factor[i] = (T - swing_time_) / T;
    }
  } else {  // standing mode
    // move foot with distance of foot to planned raibert position
    std::array<bool, N_LEGS> move{};
    bool move_any = false;
    if (raibert.has_value()) {
      const auto gs = gait_sequence.value();
      const auto rb = raibert.value();
      for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
        bool in_swing_phase = leg_phase_[leg] >= duty_factor[leg];

        Eigen::Vector3d p_raibert = rb.get_foothold(leg,
                                                    gs.reference_trajectory_position[0],
                                                    gs.desired_reference_trajectory_orientation[0],
                                                    get_t_stance(leg),
                                                    gs.reference_trajectory_velocity[0],
                                                    gs.target_velocity,
                                                    gs.target_twist);
        Eigen::Vector3d p_current = rb.get_current_foot_position(leg);
        double dist = (p_raibert - p_current).segment(0, 2).norm();
        move[leg] = in_swing_phase
                    || (dist > standing_foot_position_threshold_
                        && standing_foot_position_threshold_ > std::numeric_limits<double>::epsilon());
        move_any = move_any || move[leg];
      }
    }
    T = correction_period_;
    for (int i = 0; i < N_LEGS; ++i) {
      if (move[i] || (move_any && correct_all_)) {
        duty_factor[i] = (T - swing_time_) / T;
      } else {
        duty_factor[i] = 1.0;
      }
    }
  }
}

void AdaptiveGait::nextPhase(std::array<double, N_LEGS>& phase,
                             double dt,
                             double T,
                             const std::array<double, N_LEGS>& phase_offset,
                             const std::array<double, N_LEGS>& df_old,
                             const std::array<double, N_LEGS>& df_new,
                             const std::optional<std::array<bool, N_LEGS>>& contact_state) {
  for (int i = 0; i < N_LEGS; ++i) {
    if (T <= std::numeric_limits<double>::epsilon() || df_old[i] <= std::numeric_limits<double>::epsilon()
        || df_old[i] >= 1.0 - std::numeric_limits<double>::epsilon()) {
      bool all = true;
      for (int j = 0; j < N_LEGS; ++j) {
        if (T >= std::numeric_limits<double>::epsilon()
            && (df_old[j] <= std::numeric_limits<double>::epsilon()
                || df_old[j] >= 1.0 - std::numeric_limits<double>::epsilon())) {
          all = false;
          break;
        }
      }
      if (!all) {
        phase[i] = std::fmod(phase[i] + dt / T, 1.0);  // try to fix correction steps by adding this
        // phase[i] = fmod(1.0 - phase_offset[i], 1.0);  // FIXME: decide wether this fixes the bug (same foot moves
        // in a row, and if we need to add something similar again)
      }
    } else {
      phase[i] = std::fmod(phase[i] + dt / T, 1.0);

      if ((phase[i]) < df_old[i]) {
        phase[i] = std::fmod(phase[i] * df_new[i] / df_old[i], 1.0);
      } else {
        phase[i] = std::fmod((phase[i] - df_old[i]) * (1.0 - df_new[i]) / (1.0 - df_old[i]) + df_new[i], 1.0);
      }
    }
  }

  // early contact detection (contact before planned contact)
  if (contact_state.has_value()) {
    const auto& contact_state_array = contact_state.value();
    (void)contact_state_array;
    for (int i = 0; i < N_LEGS; ++i) {
      if (contact_state_array[i] && (phase[i] > (df_new[i]))) {
        if (phase[i] > ((df_new[i] + 1.0) / 2.0)) {
          phase[i] = 0.0;  // set in standing phase
        }
        // else {
        //   // Late contact TODO: decide wether to keep late contact detection
        //   phase[i] = df_new[i];  // set in standing phase
        // }
      }
    }
  }

  // TODO: correct phase offsets here: find phases where offset error is lowest
  // and go towards this. (only for legs in stance phase)

  // find index of leg which has the longes swing time left.
  // if no leg in swing phase, take the shortest stance time left.
  int ground_idx = 0;
  double min_dif = 1.0;
  bool swing_found = false;
  for (int i = 0; i < N_LEGS; ++i) {
    double dif = phase[i] - df_new[i];
    if (swing_found && dif >= 0.0 && dif < min_dif) {
      ground_idx = i;
      min_dif = dif;
    } else if (!swing_found && dif >= 0) {
      ground_idx = i;
      min_dif = dif;
      swing_found = true;
    } else if (!swing_found && abs(dif) < min_dif) {
      ground_idx = i;
      min_dif = abs(dif);
    }
  }
  // find current phase offsets
  std::array<double, N_LEGS> current_phase_offset;
  std::array<double, N_LEGS> phase_offset_error{};  // compiler error without initialization
  current_phase_offset[ground_idx] = phase_offset[ground_idx];
  phase_offset_error[ground_idx] = 0.0;
  for (int i = 0; i < N_LEGS; ++i) {
    if (i != ground_idx) {
      current_phase_offset[i] = fmod((phase[ground_idx] + phase_offset[ground_idx]) - phase[i], 1.0);
      // error between -0.5 and 0.5
      phase_offset_error[i] = fmod((current_phase_offset[i] - phase_offset[i]) + 0.5, 1.0) - 0.5;
    }
  }
  // interpolate phases
  double max_error_correction = dt / (max_correction_cycles_ * T);  // TODO: adapt value base inspired by w2 of
                                                                    // transition paper
  for (int i = 0; i < N_LEGS; ++i) {
    // only update stance legs
    if (phase[i] < df_new[i] - max_error_correction) {
      // don't go in swing phase
      if (phase_offset_error[i] >= 0.0) {
        phase[i] = std::min(phase[i] + std::min(phase_offset_error[i], max_error_correction), df_new[i]);
      } else {
        phase[i] = std::max(phase[i] + std::max(phase_offset_error[i], -max_error_correction), 0.0);
      }
    }
  }
}

double AdaptiveGait::get_froude(double v, double h) const { return std::abs((v * v) / (GRAVITY_CONSTANT * h)); }

void AdaptiveGait::get_current_period(double& period) const { period = T_; }
void AdaptiveGait::get_current_contact(std::array<bool, 4>& contact) const {
  for (int i = 0; i < N_LEGS; ++i) {
    contact[i] = leg_phase_[i] < duty_factor_[i];
  }
}
void AdaptiveGait::get_current_phase(std::array<double, 4>& phase) const {
  for (int i = 0; i < N_LEGS; ++i) {
    phase[i] = leg_phase_[i];
  }
}
void AdaptiveGait::get_current_duty_factor(std::array<double, 4>& duty_factor) const {
  for (int i = 0; i < N_LEGS; ++i) {
    duty_factor[i] = duty_factor_[i];
  }
}
void AdaptiveGait::get_current_phase_offset(std::array<double, 4>& phase_offset) const {
  for (int i = 0; i < N_LEGS; ++i) {
    phase_offset[i] = phase_offset_[i];
  }
}
void AdaptiveGait::set_offset(const std::array<double, 4>& phase_offset, bool switch_offsets) {
  switch_offsets_ = switch_offsets;
  if (offset_delay_ <= std::numeric_limits<double>::epsilon()) {
    phase_offset_ = phase_offset;
    new_phase_offset_ = phase_offset;
  } else if (phase_offset != new_phase_offset_) {
    new_phase_offset_ = phase_offset;
    remaining_offset_delay_ = offset_delay_;
  }
}
void AdaptiveGait::set_offset_switch(bool switch_offsets) { switch_offsets_ = switch_offsets; }
void AdaptiveGait::set_swing_time(double swing_time) { swing_time_ = swing_time; }
void AdaptiveGait::set_filter_size(unsigned int filter_size) { velocity_filter_.resize(filter_size); }
void AdaptiveGait::set_zero_velocity_threshold(double threshold) { zero_velocity_threshold_ = threshold; }
void AdaptiveGait::set_standing_foot_position_threshold(double threshold) {
  standing_foot_position_threshold_ = threshold;
}
void AdaptiveGait::set_max_correction_cycles(double cycles) { max_correction_cycles_ = cycles; }

void AdaptiveGait::set_min_v_cmd_factor(double min_v_cmd_factor) { min_v_cmd_factor_ = min_v_cmd_factor; }
void AdaptiveGait::set_correct_all(bool correct_all) { correct_all_ = correct_all; }
void AdaptiveGait::set_correction_period(double period) { correction_period_ = period; }
void AdaptiveGait::set_disturbance_correction(double factor) { disturbance_correction_ = factor; }
void AdaptiveGait::set_min_v(double v) { min_v_ = v; }
void AdaptiveGait::set_offset_delay(double delay) { offset_delay_ = delay; }
void AdaptiveGait::set_max_stride_length(double stride_length) { max_stride_length_ = stride_length; }

// ------------------------- GaitDatabase ------------------------- //
Gait GaitDatabase::getGait(GaitType type, double dt) {
  switch (type) {
    case GaitType::STAND:
    default:
      return Gait(0.5, 1.0, {0.0, 0.0, 0.0, 0.0}, dt, "STAND");
      break;
    case GaitType::STATIC_WALK:
      // != MIT code, as they use wrong phase offsets!
      return Gait(1.25, 0.8, {0.0, 0.5, 0.75, 0.25}, dt, "STATIC_WALK");
      break;
    case GaitType::WALKING_TROT:
      return Gait(0.5, 0.6, {0.0, 0.5, 0.5, 0.0}, dt, "WALKING_TROT");
      break;
    case GaitType::TROT:
      return Gait(0.5, 0.5, {0.0, 0.5, 0.5, 0.0}, dt, "TROT");
      break;
    case GaitType::FLYING_TROT:
      return Gait(0.4, 0.4, {0.0, 0.5, 0.5, 0.0}, dt, "FLYING_TROT");
      break;
    case GaitType::PACE:
      return Gait(0.35, 0.5, {0.0, 0.5, 0.0, 0.5}, dt, "PACE");
      break;
    case GaitType::BOUND:
      return Gait(0.4, 0.4, {0.0, 0.0, 0.5, 0.5}, dt, "BOUND");
      break;
    case GaitType::ROTARY_GALLOP:
      return Gait(0.4, 0.2, {0.0, 0.8571, 0.3571, 0.5}, dt, "ROTARY_GALLOP");
      break;
    case GaitType::TRAVERSE_GALLOP:
      return Gait(0.5, 0.2, {0.0, 0.8571, 0.3571, 0.5}, dt, "TRAVERSE_GALLOP");
      break;
    case GaitType::PRONK:
      return Gait(0.5, 0.5, {0.0, 0.0, 0.0, 0.0}, dt, "PRONK");
      break;
  }
}

// ---------------------------------- BioGaitDatabase
// ---------------------------------- //
BioInspiredGait BioGaitDatabase::getGait(GaitType type, double dt) {
  switch (type) {
    case GaitType::STAND:
    default:
      return BioInspiredGait(0.5, 1.0, {0.0, 0.5, 0.75, 0.25}, dt, 0.0, 0.0009, "STAND");
      break;
    case GaitType::STATIC_WALK:
      return BioInspiredGait(0.6, 0.8, {0.0, 0.5, 0.75, 0.25}, dt, 0.0009, 0.0024, "STATIC_WALK");
      break;
    case GaitType::WALKING_TROT:
      return BioInspiredGait(0.5, 0.6, {0.0, 0.5, 0.5, 0.0}, dt, 0.0024, 0.1517, "WALKING_TROT");
      break;
    case GaitType::TROT:
      return BioInspiredGait(0.4, 0.5, {0.0, 0.5, 0.5, 0.0}, dt, 0.1517, 0.74, "TROT");
      break;
    case GaitType::FLYING_TROT:
      return BioInspiredGait(0.3, 0.4, {0.0, 0.5, 0.5, 0.0}, dt, 0.74, 1.0, "FLYING_TROT");
      break;
  }
}

BioInspiredGait BioGaitDatabase::getGait(double froude, double dt) {
  if (froude < 0.0009) {
    return BioInspiredGait(0.5, 1.0, {0.0, 0.5, 0.75, 0.25}, dt, 0.0, 0.0009, "STAND");
  } else if (froude < 0.0024) {
    return BioInspiredGait(0.6, 0.75, {0.0, 0.5, 0.75, 0.25}, dt, 0.0009, 0.0024, "STATIC_WALK");
  } else if (froude < 0.1517) {
    return BioInspiredGait(0.5, 0.6, {0.0, 0.5, 0.5, 0.0}, dt, 0.0024, 0.1517, "WALKING_TROT");
  } else if (froude < 0.74) {
    return BioInspiredGait(0.4, 0.5, {0.0, 0.5, 0.5, 0.0}, dt, 0.1517, 0.74, "TROT");
  } else {
    return BioInspiredGait(0.3, 0.4, {0.0, 0.5, 0.5, 0.0}, dt, 0.74, 1.0, "FLYING_TROT");
  }
}
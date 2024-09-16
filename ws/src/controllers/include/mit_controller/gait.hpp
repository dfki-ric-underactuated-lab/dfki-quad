#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "common/filters.hpp"
#include "mit_controller/gait_interface.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/raibert_foot_step_planner.hpp"
#include "mit_controller_params.hpp"

class Gait : public GaitInterface {
 public:
  Gait(double period,
       double duty_factor,
       const std::array<double, N_LEGS>& phase_offset,
       double dt,
       const std::string& name = "");
  Gait(double period,
       const std::array<double, N_LEGS>& duty_factor,
       const std::array<double, N_LEGS>& phase_offset,
       double dt,
       const std::string& name = "");
  virtual void update_sequence(GaitSequence& gait_sequence,
                               double phase,
                               const std::optional<std::array<bool, N_LEGS>>& contact_state = std::nullopt);
  virtual void get_contact_sequence(std::array<std::array<bool, N_LEGS>, GAIT_SEQUENCE_SIZE>& contact_sequence,
                                    double phase) const;
  virtual void get_swing_time_sequence(std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE>& swing_time_sequence,
                                       double phase) const;
  virtual void get_contact_sequence(GaitSequence& gait_sequence, double phase) const;
  virtual void get_swing_time_sequence(GaitSequence& gait_sequence, double phase) const;

  virtual double get_period() const;
  virtual std::array<double, N_LEGS> get_duty_factor() const;
  virtual double get_duty_factor(unsigned int leg) const;
  virtual std::array<double, N_LEGS> get_phase_offset() const;
  virtual double get_phase_offset(unsigned int leg) const;
  virtual double get_dt() const;
  virtual std::string get_name() const;
  virtual double get_t_stance(unsigned int leg) const;
  virtual double get_t_swing(unsigned int leg) const;
  virtual void get_t_swing(GaitSequence& gait_sequence) const;

 protected:
  std::string name_;
  double dt_;
  double T_;
  std::array<double, N_LEGS> duty_factor_;
  std::array<double, N_LEGS> phase_offset_;
};

class BioInspiredGait : public Gait {
 public:
  BioInspiredGait(double period,
                  const std::array<double, N_LEGS>& duty_factor,
                  const std::array<double, N_LEGS>& phase_offset,
                  double dt,
                  double froude_lb,
                  double froude_ub,
                  const std::string& name = "");
  BioInspiredGait(double period,
                  double duty_factor,
                  const std::array<double, N_LEGS>& phase_offset,
                  double dt,
                  double froude_lb,
                  double froude_ub,
                  const std::string& name = "");

  void update_sequence_transition(GaitSequence& gait_sequence,
                                  double phase,
                                  double time,
                                  const BioInspiredGait& new_gait,
                                  const std::optional<std::array<bool, N_LEGS>>& contact_state = std::nullopt);

  void get_contact_transition(GaitSequence& gait_sequence,
                              double phase,
                              double time,
                              const BioInspiredGait& new_gait) const;

  //   void get_contact_transition(std::array<std::array<bool, N_LEGS>,
  //                                          GAIT_SEQUENCE_SIZE>&
  //                                          contact_sequence,
  //                               double phase,
  //                               const BioInspiredGait& new_gait) const;

  // void get_swing_time_transition(
  //     std::array<std::array<double, N_LEGS>, GAIT_SEQUENCE_SIZE>&
  //         swing_time_transition,
  //     double phase,
  //     const BioInspiredGait& new_gait) const;

  void get_swing_time_transition(GaitSequence& gait_sequence,
                                 double phase,
                                 double time,
                                 const BioInspiredGait& new_gait) const;

  double get_D() const;
  double get_Dtrans(const BioInspiredGait& new_gait) const;
  double get_ntrans(const BioInspiredGait& new_gait, double time, double froude) const;
  double get_w1(const BioInspiredGait& new_gait, double time, double froude) const;
  double get_w2(const BioInspiredGait& new_gait, double time, double froude) const;
  double get_froude_lb() const;
  double get_froude_ub() const;
  double get_froude(double v, double h) const;

  // operators
  bool operator==(const BioInspiredGait& other) const;
  bool operator!=(const BioInspiredGait& other) const;
  bool operator<(const BioInspiredGait& other) const;
  bool operator>(const BioInspiredGait& other) const;

 private:
  static constexpr double GRAVITY_CONSTANT = 9.8067;
  double froude_lb_;
  double froude_ub_;
};

class AdaptiveGait : public GaitInterface {
 public:
  AdaptiveGait(const std::array<double, N_LEGS>& phase_offset,
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
               const std::string& name = "");

  virtual void update_sequence(GaitSequence& gait_sequence,
                               double dt,
                               const std::optional<std::array<bool, N_LEGS>>& contact_state = std::nullopt,
                               const std::optional<RaibertFootStepPlanner>& raibert = std::nullopt);

  //   void UpdateState();
  virtual void nextPhase(std::array<double, N_LEGS>& phase,
                         double dt,
                         double T,
                         const std::array<double, N_LEGS>& phase_offset,
                         const std::array<double, N_LEGS>& df_old,
                         const std::array<double, N_LEGS>& df_new,
                         const std::optional<std::array<bool, N_LEGS>>& contact_state = std::nullopt);

  virtual double get_t_stance(unsigned int leg) const;
  // virtual double get_t_swing(unsigned int leg = 0) const;
  // virtual void get_t_swing(GaitSequence& gait_sequence) const;
  double get_froude(double v, double h) const;
  void get_current_period(double& period) const;
  void get_current_contact(std::array<bool, 4>& contact) const;
  void get_current_phase(std::array<double, 4>& phase) const;
  void get_current_duty_factor(std::array<double, 4>& duty_factor) const;
  void get_current_phase_offset(std::array<double, 4>& phase_offset) const;
  double disturbance_factor(const GaitSequence& gait_sequence) const;

  void set_offset(const std::array<double, 4>& phase_offset, bool switch_offsets = false);
  void set_offset_switch(bool switch_offsets);
  void set_swing_time(double swing_time);
  void set_filter_size(unsigned int filter_size);
  void set_zero_velocity_threshold(double threshold);
  void set_standing_foot_position_threshold(double threshold);
  void set_min_v_cmd_factor(double min_v_cmd_factor);
  void set_max_correction_cycles(double cycles);
  void set_correct_all(bool correct_all);
  void set_correction_period(double period);
  void set_disturbance_correction(double factor);
  void set_min_v(double v);
  void set_offset_delay(double delay);
  void set_max_stride_length(double stride_length);

 protected:
  static constexpr double GRAVITY_CONSTANT = 9.8067;
  virtual void update_params(double& T,
                             std::array<double, N_LEGS>& duty_factor,
                             std::array<double, N_LEGS>& phase_offset,
                             double& remaining_offset_delay,
                             double dt,
                             double v,
                             double h,
                             bool switch_offsets,
                             const std::optional<GaitSequence>& gait_sequence = std::nullopt,
                             const std::optional<RaibertFootStepPlanner>& raibert = std::nullopt);
  std::array<double, N_LEGS> leg_phase_;
  double swing_time_;
  std::string name_;
  double dt_;
  double T_;
  std::array<double, N_LEGS> duty_factor_;
  std::array<double, N_LEGS> phase_offset_;
  std::array<double, N_LEGS> new_phase_offset_;
  MovingAverage<Eigen::Vector2d> velocity_filter_;
  double zero_velocity_threshold_;
  bool switch_offsets_;
  double offset_delay_;
  double remaining_offset_delay_;
  double standing_foot_position_threshold_;
  double max_correction_cycles_;  // maximum required gait cycles to correct phase offsets
  double min_v_cmd_factor_;       // factor * v_cmd is the min velocity for finding gait parameters
  bool correct_all_;
  double correction_period_;
  double disturbance_correction_;
  double min_v_;
  double max_stride_length_;
};

struct GaitDatabase {
  enum GaitType {
    STAND,
    STATIC_WALK,
    WALKING_TROT,
    TROT,
    FLYING_TROT,
    PACE,
    BOUND,
    ROTARY_GALLOP,
    TRAVERSE_GALLOP,
    PRONK,
  };

  static Gait getGait(GaitType type, double dt);
};

struct BioGaitDatabase {
  enum GaitType {
    STAND,
    STATIC_WALK,
    WALKING_TROT,
    TROT,
    FLYING_TROT,
  };

  static BioInspiredGait getGait(GaitType type, double dt);
  static BioInspiredGait getGait(double froude, double dt);
};
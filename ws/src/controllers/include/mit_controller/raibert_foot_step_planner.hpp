#pragma once

#include <array>
#include <common/filters.hpp>
#include <common/model_interface.hpp>
#include <common/state_interface.hpp>
#include <string>

#include "common/quaternion_operations.hpp"
#include "eigen3/Eigen/Dense"
#include "gait_interface.hpp"
#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "potato_sim/potato_model.hpp"
#include "target.hpp"

class RaibertFootStepPlanner {
 public:
  RaibertFootStepPlanner(const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                         const StateInterface& quad_state,
                         const ModelInterface& quad_model,
                         unsigned int filtersize,
                         bool z_on_plane,
                         double k,
                         double g = 9.81);

  void get_foot_position_sequence(GaitSequence& gait_sequence, const GaitInterface& gait);
  Eigen::Vector3d get_current_foot_position(unsigned int leg) const;

  Eigen::Vector3d get_foothold(unsigned int leg,
                               const Eigen::Vector3d position,
                               const Eigen::Quaterniond& orientation,
                               double t_stance,
                               const Eigen::Vector3d& v,
                               const Eigen::Vector3d& v_cmd,
                               const Eigen::Vector3d& omega_cmd) const;
  Eigen::Vector3d calc_p_shoulder(unsigned int leg,
                                  const Eigen::Vector3d& position,
                                  const Eigen::Quaterniond& orientation) const;
  Eigen::Vector3d calc_p_symmetry(double t_stance, const Eigen::Vector3d& v, const Eigen::Vector3d& v_cmd) const;
  Eigen::Vector3d calc_p_centrifugal(const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& v,
                                     const Eigen::Vector3d& omega_cmd) const;

 private:
  void set_z_to_plane(Eigen::Vector3d& pos);

  const std::array<const Eigen::Vector3d, N_LEGS> shoulder_positions_;  // in body frame
  const StateInterface& quad_state_;
  const ModelInterface& quad_model_;
  double k_;  // Raibert feedback gain
  double g_;
  MovingAverage<Eigen::Vector3d> velocity_filter_;
  double max_leg_length_;
  std::array<double, N_LEGS> last_foot_heights_;
  bool z_on_plane_;
  std::array<Eigen::Vector3d, N_LEGS> last_foot_positions_;
  Eigen::Vector3d plane_coefficients_;
};
#include "raibert_foot_step_planner.hpp"

#include <cmath>
#include <common/model_interface.hpp>
#include <common/state_interface.hpp>
#include <limits>
#include <vector>

#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"

RaibertFootStepPlanner::RaibertFootStepPlanner(const std::array<const Eigen::Vector3d, N_LEGS> &shoulder_positions,
                                               const StateInterface &quad_state,
                                               const ModelInterface &quad_model,
                                               unsigned int filtersize,
                                               bool z_on_plane,
                                               double k,
                                               double g)
    : shoulder_positions_(shoulder_positions),
      quad_state_(quad_state),
      quad_model_(quad_model),
      k_(k),
      g_(g),
      velocity_filter_(filtersize),
      z_on_plane_(z_on_plane) {
  // initialize z height with min value of current foot heights
  for (int i = 0; i < N_LEGS; ++i) {
    last_foot_heights_[i] = quad_model_.CalcFootPositionInWorld(i, quad_state_).z();
    last_foot_positions_[i] = quad_model_.CalcFootPositionInWorld(i, quad_state_);
  }
  double min_z = *std::min_element(last_foot_heights_.begin(), last_foot_heights_.end());
  for (int i = 0; i < N_LEGS; ++i) {
    last_foot_heights_[i] = min_z;
    last_foot_positions_[i].z() = min_z;
  }

  Eigen::Matrix4d T;
  Eigen::Vector3d pos;
  Eigen::Vector3d q{0.0, 0.0, 0.0};
  quad_model_.CalcFwdKinLegBody(0, q, T, pos);
  max_leg_length_ = -pos.z() - 0.02;
};

void RaibertFootStepPlanner::get_foot_position_sequence(GaitSequence &gait_sequence, const GaitInterface &gait) {
  std::array<std::array<Eigen::Vector3d, N_LEGS>, GAIT_SEQUENCE_SIZE> &foot_position_sequence =
      gait_sequence.foot_position_sequence;
  std::array<bool, N_LEGS> resume_contact{true};
  Eigen::Vector3d v = velocity_filter_.update(quad_state_.GetLinearVelInWorld());

  for (unsigned int i = 0; i < GAIT_SEQUENCE_SIZE; i++) {
    for (unsigned int leg = 0; leg < N_LEGS; leg++) {
      // after saving of new foot positions
      if (z_on_plane_ && i == 1 && leg == 0) {
        // plane fitting after https://www.janssenprecisionengineering.com/downloads/Fit-plane-through-data-points.pdf
        Eigen::Matrix3d planeA = Eigen::Matrix3d::Zero();
        Eigen::Vector3d planeB = Eigen::Vector3d::Zero();
        for (int j = 0; j < N_LEGS; ++j) {
          planeA(0, 0) += last_foot_positions_[j].x() * last_foot_positions_[j].x();
          planeA(0, 1) += last_foot_positions_[j].x() * last_foot_positions_[j].y();
          planeA(0, 2) += last_foot_positions_[j].x();
          planeA(1, 0) += last_foot_positions_[j].x() * last_foot_positions_[j].y();
          planeA(1, 1) += last_foot_positions_[j].y() * last_foot_positions_[j].y();
          planeA(1, 2) += last_foot_positions_[j].y();
          planeA(2, 0) += last_foot_positions_[j].x();
          planeA(2, 1) += last_foot_positions_[j].y();
          planeA(2, 2) += 1.0;
          planeB(0) += last_foot_positions_[j].x() * last_foot_positions_[j].z();
          planeB(1) += last_foot_positions_[j].y() * last_foot_positions_[j].z();
          planeB(2) += last_foot_positions_[j].z();
        }
        plane_coefficients_ = planeA.completeOrthogonalDecomposition().pseudoInverse() * planeB;
        // TODO: maybe evaluate quality of plane fit
      }
      // if in contact
      if (gait_sequence.contact_sequence[i][leg]) {
        if (i == 0) {  // use current position
          foot_position_sequence[i][leg] = quad_model_.CalcFootPositionInWorld(leg, quad_state_);
          last_foot_heights_[leg] = foot_position_sequence[i][leg].z();
          last_foot_positions_[leg] = foot_position_sequence[i][leg];
        } else if (resume_contact[leg]) {  // use last position
          foot_position_sequence[i][leg] = foot_position_sequence[i - 1][leg];
        } else {  // calc new position
          Eigen::Vector3d position = gait_sequence.reference_trajectory_position[i];

          Eigen::Quaterniond orientation =
              yaw_quaternion_from_quaternion(gait_sequence.reference_trajectory_orientation[i]);
          Eigen::Vector3d v_cmd = gait_sequence.reference_trajectory_velocity[i];
          Eigen::Vector3d omega_cmd = gait_sequence.reference_trajectory_twist[i];
          double t_stance = gait.get_t_stance(leg);

          foot_position_sequence[i][leg] = get_foothold(leg, position, orientation, t_stance, v, v_cmd, omega_cmd);

          if (z_on_plane_) {
            set_z_to_plane(foot_position_sequence[i][leg]);
          } else {
            foot_position_sequence[i][leg].z() =
                last_foot_heights_[leg];  // TODO: this is an slightly improved ugly hotfix
          }
          // clip foot position to reachable position: scale the horizontal offset from the shoulder onto the
          // reachable circle at the foot height, keeping its direction
          Eigen::Vector3d p_shoulder = calc_p_shoulder(leg, position, orientation);
          if ((p_shoulder - foot_position_sequence[i][leg]).norm() > max_leg_length_) {
            Eigen::Vector3d dif = foot_position_sequence[i][leg] - p_shoulder;
            if (std::abs(dif.z()) <= max_leg_length_) {
              double xy_len = std::sqrt(max_leg_length_ * max_leg_length_ - dif.z() * dif.z());
              double dxy = dif.segment<2>(0).norm();
              if (dxy > std::numeric_limits<double>::epsilon()) {
                dif.segment<2>(0) *= xy_len / dxy;
              }
              foot_position_sequence[i][leg] = p_shoulder + dif;
            } else {
              // FIXME: find workaround in this case (floor not reachable).
            }
          }
        }
        resume_contact[leg] = true;
        // swing phasex
      } else {
        resume_contact[leg] = false;
        foot_position_sequence[i][leg] = Eigen::Vector3d::Zero();
      }
    }
  }
}

Eigen::Vector3d RaibertFootStepPlanner::calc_p_shoulder(unsigned int leg,
                                                        const Eigen::Vector3d &position,
                                                        const Eigen::Quaterniond &orientation) const {
  return position + orientation * shoulder_positions_[leg];
  // return T_wb.block<3, 1>(0, 3) + T_wb.block<3, 3>(0, 0) *
  // shoulder_positions_[leg];
}

Eigen::Vector3d RaibertFootStepPlanner::calc_p_symmetry(double t_stance,
                                                        const Eigen::Vector3d &v,
                                                        const Eigen::Vector3d &v_cmd) const {
  return (t_stance / 2.0) * v + k_ * (v - v_cmd);
}

Eigen::Vector3d RaibertFootStepPlanner::calc_p_centrifugal(const Eigen::Vector3d &position,
                                                           const Eigen::Vector3d &v,
                                                           const Eigen::Vector3d &omega_cmd) const {
  double h = position(2) - *std::min_element(last_foot_heights_.begin(), last_foot_heights_.end());  // base height
  if (h < 0.0) {
    std::cout << "Warning: Raibert detects negative base height" << std::endl;
  }
  h = std::clamp(h, 0.0, max_leg_length_);
  return 0.5 * std::sqrt(h / g_) * v.cross(omega_cmd);
}

Eigen::Vector3d RaibertFootStepPlanner::get_foothold(unsigned int leg,
                                                     const Eigen::Vector3d position,
                                                     const Eigen::Quaterniond &orientation,
                                                     double t_stance,
                                                     const Eigen::Vector3d &v,
                                                     const Eigen::Vector3d &v_cmd,
                                                     const Eigen::Vector3d &omega_cmd) const {
  (void)v;
  (void)omega_cmd;
  (void)t_stance;
  (void)v_cmd;
  return calc_p_shoulder(leg, position, orientation) + calc_p_centrifugal(position, v, omega_cmd)
         + calc_p_symmetry(t_stance, v, v_cmd);
}

Eigen::Vector3d RaibertFootStepPlanner::get_current_foot_position(unsigned int leg) const {
  return quad_model_.CalcFootPositionInWorld(leg, quad_state_);
}

void RaibertFootStepPlanner::set_z_to_plane(Eigen::Vector3d &pos) {
  pos.z() = plane_coefficients_.x() * pos.x() + plane_coefficients_.y() * pos.y() + plane_coefficients_.z();
}

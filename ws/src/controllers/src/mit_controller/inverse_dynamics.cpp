#include "mit_controller/inverse_dynamics.hpp"

InverseDynamics::InverseDynamics(std::unique_ptr<ModelInterface> model,
                                 std::unique_ptr<StateInterface> state,
                                 bool foot_position_based_on_target_height,
                                 bool foot_position_based_on_target_orientation,
                                 unsigned int transformation_filter_size,
                                 double target_velocity_blend)
    : quad_model_(std::move(model)),
      quad_state_(std::move(state)),
      foot_position_based_on_target_height_(foot_position_based_on_target_height),
      foot_position_based_on_target_orientation_(foot_position_based_on_target_orientation),
      body_velocity_filter_(transformation_filter_size),
      body_twist_filter_(transformation_filter_size) {
  setTargetVelocityBlend(target_velocity_blend);
}

void InverseDynamics::UpdateState(const StateInterface& quad_state) { *quad_state_ = quad_state; }
void InverseDynamics::UpdateModel(const ModelInterface& quad_model) { *quad_model_ = quad_model; }
void InverseDynamics::UpdateFeetTarget(const FeetTargets& feet_targets) { feet_targets_ = feet_targets; }
void InverseDynamics::UpdateWrenches(const Wrenches& wrenches) { wrench_sequence_ = wrenches; }

void InverseDynamics::UpdateFootContact(const FootContact& foot_contact) { gait_sequence_ = foot_contact; }
WBCReturn InverseDynamics::GetJointCommand(CartesianCommands& joint_command) {
  auto target_pos_in_world = quad_state_->GetPositionInWorld();
  if (foot_position_based_on_target_height_) {
    target_pos_in_world.z() = target_position_.z();
  }
  Eigen::Quaterniond target_orient_in_world;
  if (foot_position_based_on_target_orientation_) {
    target_orient_in_world = target_orientation_;
  } else {
    target_orient_in_world = quad_state_->GetOrientationInWorld();
  }
  body_velocity_filter_.update(quad_state_->GetLinearVelInWorld());
  body_twist_filter_.update(quad_state_->GetAngularVelInWorld());

  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    // Check who is in charge of that leg
    if (gait_sequence_[leg_idx]) {
      joint_command.velocity[leg_idx].setZero();
      joint_command.position[leg_idx] = (Eigen::Translation3d(target_pos_in_world) * target_orient_in_world).inverse()
                                        * feet_targets_.positions[leg_idx];
      joint_command.force[leg_idx] = -1 * (quad_state_->GetOrientationInWorld().inverse() * wrench_sequence_[leg_idx]);

    } else {  // if planned no contact then slc
      // velocity in body

      auto q_array = quad_state_->GetJointPositions()[leg_idx];
      Eigen::Vector3d q;
      for (int i = 0; i < 3; ++i) {
        q(i) = q_array[i];
      }
      // foot position in body frame
      Eigen::Vector3d p_be = quad_model_->GetFootPositionInBodyFrame(leg_idx, q);

      Eigen::Vector3d v_bw = -(body_velocity_filter_.get() * (1.0 - target_velocity_blend_)
                               + target_linear_velocity_ * target_velocity_blend_);
      Eigen::Vector3d w_bw = -(body_twist_filter_.get() * (1.0 - target_velocity_blend_)
                               + target_angular_velocity_ * target_velocity_blend_);

      Eigen::Vector3d vel_world_in_body = quad_state_->GetOrientationInWorld().conjugate() * v_bw;
      Eigen::Vector3d vel_foot_in_body =
          quad_state_->GetOrientationInWorld().conjugate() * feet_targets_.velocities[leg_idx]
          + skew_matrix(p_be) * quad_state_->GetOrientationInWorld().conjugate().toRotationMatrix() * w_bw;
      Eigen::Vector3d vel_body = vel_world_in_body + vel_foot_in_body;  // TODO: check if correct now

      joint_command.velocity[leg_idx] = vel_body;
      joint_command.position[leg_idx] =
          (Eigen::Translation3d(quad_state_->GetPositionInWorld()) * quad_state_->GetOrientationInWorld()).inverse()
          * feet_targets_.positions[leg_idx];
      joint_command.force[leg_idx].setZero();
    }
  }
  return {true, 0., 0.};
}

void InverseDynamics::UpdateTarget(const Eigen::Quaterniond& orientation,
                                   const Eigen::Vector3d& position,
                                   const Eigen::Vector3d& lin_vel,
                                   const Eigen::Vector3d& ang_vel) {
  target_orientation_ = orientation;
  target_position_ = position;
  target_linear_velocity_ = lin_vel;
  target_angular_velocity_ = ang_vel;
}

void InverseDynamics::setFootPositionBasedOnTargetHeight(bool foot_position_based_on_target_height) {
  foot_position_based_on_target_height_ = foot_position_based_on_target_height;
}
void InverseDynamics::setFootPositionBasedOnTargetOrientation(bool foot_position_based_on_target_orientation) {
  foot_position_based_on_target_orientation_ = foot_position_based_on_target_orientation;
}
void InverseDynamics::setTransformationFilterSize(unsigned int filter_size) {
  body_twist_filter_.resize(filter_size);
  body_velocity_filter_.resize(filter_size);
}
void InverseDynamics::setTargetVelocityBlend(double blend) {
  target_velocity_blend_ = std::max(std::min(blend, 1.0), 0.0);
}

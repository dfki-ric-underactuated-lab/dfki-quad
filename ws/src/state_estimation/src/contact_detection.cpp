#include "contact_detection.hpp"

void ContactDetection::Update(const interfaces::msg::JointState& joint_state_msg, double dt) {
  static Eigen::Vector3d unused;
  static Eigen::Vector3d force;
  for (unsigned int foot_idx = 0; foot_idx < ModelInterface::N_LEGS; foot_idx++) {
    quad_model_->CalcFootForceVelocityInBodyFrame(
        foot_idx,
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.position.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.velocity.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.acceleration.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.effort.data() + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        force,
        unused);
    foot_contact_forces_[foot_idx] = -force;

    double kinetic_energy = quad_model_->ComputeKineticEnergy(
        foot_idx,
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.position.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.velocity.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG));
    double energy_derivative = quad_model_->ComputeEnergyDerivative(
        foot_idx,
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.velocity.data()
                                          + foot_idx * ModelInterface::N_JOINTS_PER_LEG),
        Eigen::Map<const Eigen::Vector3d>(joint_state_msg.effort.data() + foot_idx * ModelInterface::N_JOINTS_PER_LEG));

    enery_obs_[foot_idx] =
        std::abs(energy_obs_kd_ * (kinetic_energy - (energy_derivative + prev_energy_obs_[foot_idx]) * dt));

    prev_energy_obs_[foot_idx] = enery_obs_[foot_idx];
    leg_in_motion_[foot_idx] = (Eigen::Map<const Eigen::Vector3d>(joint_state_msg.velocity.data()
                                                                  + foot_idx * ModelInterface::N_JOINTS_PER_LEG)
                                    .array()
                                    .abs()
                                > leg_in_motion_joint_velocity_threshold_)
                                   .all();
  }
  energy_obs_filtered_.update(enery_obs_);
  foot_contact_forces_ = foot_contact_avg_.update(foot_contact_forces_);
  for (unsigned int foot_idx = 0; foot_idx < ModelInterface::N_LEGS; foot_idx++) {
    if (!energy_obs_rising_edge_detected_[foot_idx] and energy_obs_filtered_.get()[foot_idx] > energy_obs_threshold_) {
      // -> Rising edge, foot is moving up
      energy_obs_rising_edge_detected_[foot_idx] = true;
      energy_foot_contact_[foot_idx] = false;
      time_since_energy_obs_rising_edge_detected_[foot_idx] = 0.;
      energy_obs_state_[foot_idx] = 1;
    } else if (energy_obs_rising_edge_detected_[foot_idx]
               and (time_since_energy_obs_rising_edge_detected_[foot_idx] > (leg_swing_time_ / 2.0))
               and energy_obs_filtered_.get()[foot_idx] < energy_obs_threshold_) {
      energy_foot_contact_[foot_idx] = true;
      energy_obs_rising_edge_detected_[foot_idx] = false;
      energy_obs_state_[foot_idx] = 0;
    } else if (energy_obs_rising_edge_detected_[foot_idx]) {
      time_since_energy_obs_rising_edge_detected_[foot_idx] += dt;
    }
  }
}

void ContactDetection::GetContacts(SequenceView<bool, ModelInterface::N_LEGS> foot_in_contact,
                                   SequenceView<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> contact_forces,
                                   SequenceView<double, ModelInterface::N_LEGS> energy_obs) const {
  for (unsigned int foot_idx = 0; foot_idx < ModelInterface::N_LEGS; foot_idx++) {
    if (leg_in_motion_[foot_idx] and use_energy_obs_contact_detection_) {
      foot_in_contact[foot_idx] = energy_foot_contact_[foot_idx];
    } else {
      foot_in_contact[foot_idx] = (foot_contact_forces_[foot_idx].norm() >= contact_force_threshold_[foot_idx]);
    }

    energy_obs[foot_idx] = energy_obs_filtered_.get()[foot_idx];
    contact_forces[foot_idx].x() = foot_contact_forces_[foot_idx].x();
    contact_forces[foot_idx].y() = foot_contact_forces_[foot_idx].y();
    contact_forces[foot_idx].z() = foot_contact_forces_[foot_idx].z();
  }
}

void ContactDetection::GetContacts(SequenceView<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> contact_forces,
                                   SequenceView<double, ModelInterface::N_LEGS> energy_obs) const {
  for (unsigned int foot_idx = 0; foot_idx < ModelInterface::N_LEGS; foot_idx++) {
    contact_forces[foot_idx].x() = foot_contact_forces_[foot_idx].x();
    contact_forces[foot_idx].y() = foot_contact_forces_[foot_idx].y();
    contact_forces[foot_idx].z() = foot_contact_forces_[foot_idx].z();
    energy_obs[foot_idx] = energy_obs_filtered_.get()[foot_idx];
  }
}

ContactDetection::ContactDetection(std::shared_ptr<const ModelInterface> quadModel,
                                   const std::array<double, ModelInterface::N_LEGS>& contact_force_threshold,
                                   const double energy_obs_kd,
                                   const double energy_obs_threshold,
                                   const double leg_in_motion_joint_veloity_threshold,
                                   const bool use_energy_obs_contact_detection,
                                   const double leg_swing_time,
                                   const int energy_obs_filter_size)
    : quad_model_(quadModel),
      foot_contact_forces_(ModelInterface::N_LEGS),
      foot_contacts_(ModelInterface::N_LEGS),
      foot_contact_avg_(10),
      contact_force_threshold_(contact_force_threshold),
      use_energy_obs_contact_detection_(use_energy_obs_contact_detection),
      energy_obs_kd_(energy_obs_kd),
      energy_obs_filtered_(energy_obs_filter_size),
      energy_obs_threshold_(energy_obs_threshold),
      leg_in_motion_joint_velocity_threshold_(leg_in_motion_joint_veloity_threshold),
      leg_swing_time_(leg_swing_time) {
  enery_obs_.fill(0);
  prev_energy_obs_.fill(0);
  energy_obs_state_.fill(0);
  leg_in_motion_.fill(false);
  time_since_energy_obs_rising_edge_detected_.fill(0);
  energy_obs_rising_edge_detected_.fill(false);
  energy_foot_contact_.fill(false);
}
const std::array<uint8_t, ModelInterface::N_LEGS>& ContactDetection::GetEnergyObsState() const {
  return energy_obs_state_;
}

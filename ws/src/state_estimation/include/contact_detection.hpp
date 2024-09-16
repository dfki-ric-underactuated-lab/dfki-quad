#pragma once

#include "common/filters.hpp"
#include "common/model_interface.hpp"
#include "common/sequence_containers.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interfaces/msg/joint_state.hpp"
class ContactDetection {
 private:
  std::shared_ptr<const ModelInterface> quad_model_;
  std::vector<Eigen::Vector3d> foot_contact_forces_;
  std::vector<bool> foot_contacts_;
  MovingAverage<std::vector<Eigen::Vector3d>> foot_contact_avg_;

  const std::array<double, ModelInterface::N_LEGS> contact_force_threshold_;

  const bool use_energy_obs_contact_detection_;
  std::array<double, ModelInterface::N_LEGS> enery_obs_;
  std::array<double, ModelInterface::N_LEGS> prev_energy_obs_;
  std::array<bool, ModelInterface::N_LEGS> energy_obs_rising_edge_detected_;
  std::array<double, ModelInterface::N_LEGS> time_since_energy_obs_rising_edge_detected_;
  std::array<bool, ModelInterface::N_LEGS> energy_foot_contact_;

  const double energy_obs_kd_;  // tune to calc energy_obs
  MovingAverage<std::array<double, ModelInterface::N_LEGS>> energy_obs_filtered_;
  std::array<bool, ModelInterface::N_LEGS> leg_in_motion_;
  const double energy_obs_threshold_;  // threshold to sense contact
  const double leg_in_motion_joint_velocity_threshold_;
  const double leg_swing_time_;
  std::array<uint8_t, ModelInterface::N_LEGS> energy_obs_state_;

 public:
  ContactDetection(std::shared_ptr<const ModelInterface> quadModel,
                   const std::array<double, ModelInterface::N_LEGS>& contact_force_threshold,
                   const double energy_obs_kd,
                   const double energy_obs_threshold,
                   const double leg_in_motion_joint_veloity_threshold,
                   const bool use_energy_obs_contact_detection,
                   const double leg_swing_time,
                   const int energy_obs_filter_size);

  void Update(const interfaces::msg::JointState& joint_state_msg, double dt);
  void GetContacts(SequenceView<bool, ModelInterface::N_LEGS> foot_in_contact,
                   SequenceView<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> contact_forces,
                   SequenceView<double, ModelInterface::N_LEGS> energy_obs) const;
  void GetContacts(SequenceView<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> contact_forces,
                   SequenceView<double, ModelInterface::N_LEGS> energy_obs) const;

  const std::array<uint8_t, ModelInterface::N_LEGS>& GetEnergyObsState() const;
};
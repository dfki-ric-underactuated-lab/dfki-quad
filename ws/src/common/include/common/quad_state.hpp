#pragma once

#include "interfaces/msg/quad_state.hpp"
#include "state_interface.hpp"

class QuadState : public StateInterface {
 private:
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_acceleration_;
  Eigen::Vector3d angular_acceleration_;
  TimePoint time_point_;
  std::array<bool, NUM_FEET> feet_contacts_;
  std::array<Eigen::Vector3d, StateInterface::NUM_FEET> contact_forces_;
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_positions_;
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_velocities_;
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_accelerations_;
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_torques_;

 public:
  void UpdateFromMsg(const interfaces::msg::QuadState &quad_state_msg);
  QuadState(const interfaces::msg::QuadState &msg);
  QuadState();

  const Eigen::Vector3d &GetPositionInWorld() const override;

  const Eigen::Quaterniond &GetOrientationInWorld() const override;

  const Eigen::Vector3d &GetLinearVelInWorld() const override;

  const Eigen::Vector3d &GetAngularVelInWorld() const override;

  const Eigen::Vector3d &GetLinearAccInWorld() const override;

  const Eigen::Vector3d &GetAngularAccInWorld() const override;

  const TimePoint &GetTime() const override;

  const std::array<bool, NUM_FEET> &GetFeetContacts() const override;

  const std::array<Eigen::Vector3d, StateInterface::NUM_FEET> &GetContactForces() const override;

  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointPositions() const override;
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointVelocities() const override;
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointAccelerations() const override;
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointTorques() const override;

  StateInterface &operator=(const StateInterface &other) override;
  StateInterface &operator=(const interfaces::msg::QuadState &msg);
};
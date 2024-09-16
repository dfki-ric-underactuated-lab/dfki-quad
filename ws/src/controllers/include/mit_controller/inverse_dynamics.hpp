#pragma once
#include <iostream>
#include <memory>

#include "common/filters.hpp"
#include "common/quaternion_operations.hpp"
#include "wbc_interface.hpp"

class InverseDynamics : public WBCInterface<CartesianCommands> {
 private:
  // Model / State
  std::unique_ptr<ModelInterface> quad_model_;
  std::unique_ptr<StateInterface> quad_state_;
  // Targets
  FootContact gait_sequence_;
  Wrenches wrench_sequence_;
  FeetTargets feet_targets_;
  Eigen::Quaterniond target_orientation_;
  Eigen::Vector3d target_position_;
  Eigen::Vector3d target_linear_velocity_;
  Eigen::Vector3d target_angular_velocity_;
  // Params
  bool foot_position_based_on_target_height_;
  bool foot_position_based_on_target_orientation_;
  double target_velocity_blend_;
  // Filters
  MovingAverage<Eigen::Vector3d> body_velocity_filter_;
  MovingAverage<Eigen::Vector3d> body_twist_filter_;

 public:
  void setFootPositionBasedOnTargetHeight(bool foot_position_based_on_target_height);
  void setFootPositionBasedOnTargetOrientation(bool foot_position_based_on_target_orientation);
  void setTransformationFilterSize(unsigned int filter_size);
  void setTargetVelocityBlend(double blend);

 public:
  InverseDynamics(std::unique_ptr<ModelInterface> model,
                  std::unique_ptr<StateInterface> state,
                  bool foot_position_based_on_target_height,
                  bool foot_position_based_on_target_orientation,
                  unsigned int transformation_filter_size,
                  double target_velocity_blend);
  void UpdateState(const StateInterface& quad_state) override;
  void UpdateModel(const ModelInterface& quad_model) override;
  void UpdateFeetTarget(const FeetTargets& feet_targets) override;
  void UpdateWrenches(const Wrenches& wrenches) override;
  void UpdateFootContact(const FootContact& foot_contact) override;
  WBCReturn GetJointCommand(CartesianCommands& joint_command) override;
  void UpdateTarget(const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& position,
                    const Eigen::Vector3d& lin_vel,
                    const Eigen::Vector3d& ang_vel) override;
};
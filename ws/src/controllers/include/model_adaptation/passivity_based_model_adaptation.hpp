#pragma once

#include <memory>  // for the smart pointers

#include "model_adaptation_interface.hpp"

class PassivityBasedModelAdaptation : public ModelAdaptationInterface {
 public:
  PassivityBasedModelAdaptation(std::unique_ptr<ModelInterface> quad_model,
                                std::unique_ptr<StateInterface> initial_state);
  bool UpdateState(const StateInterface& state, ModelInterface& model) override;
  void RegressorMatrix(const Eigen::Vector3d& orientation,
                       const Eigen::Vector3d& linear_acceleration,
                       const Eigen::Vector3d& angular_acceleration,
                       const std::array<Eigen::Vector3d, 4>& contact_forces,
                       Eigen::Matrix<double, 6, 10>& y) const;
  void ComputeCompositeTrackingError(const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation,
                                     const Eigen::Vector3d& linear_velocity,
                                     const Eigen::Vector3d& angular_velocity,
                                     Eigen::Vector<double, 6>& composite_tracking_error) const;
  Eigen::Matrix3d ParamVec2InertiaTensor(const Eigen::Vector<double, 10>& phi) const;
  Eigen::Vector3d ParamVec2COMVector(const Eigen::Vector<double, 10>& phi) const;

 private:
  Eigen::Vector<double, 10> phi_;        // parametert vector
  Eigen::Matrix<double, 10, 10> Gamma_;  // adaptation gains
  Eigen::Matrix3d Lambda_l_;             // gains for linear tracking error
  Eigen::Matrix3d Lambda_a_;             // gains for angular tracking error
  GaitSequence gait_sequence_;           // target values
  const double g_ = 9.81;                // gravity TODO: make model parameter
};
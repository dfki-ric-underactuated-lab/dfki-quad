#pragma once
#include <Eigen/Dense>
#include <array>
#include <eigen3/Eigen/Core>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class BrickModel;  // foward declaration

/**
 * All in World Coordinates
 */
class BrickState : public StateInterface {
 public:
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d linear_vel_;
  Eigen::Vector3d angular_vel_;
  Eigen::Vector3d linear_acc_;
  Eigen::Vector3d angular_acc_;
  StateInterface::TimePoint time_stamp_;
  std::array<bool, StateInterface::NUM_FEET> feet_contacts_;
  std::array<Eigen::Vector3d, StateInterface::NUM_FEET> contact_forces_;
  std::array<Eigen::Vector3d, StateInterface::NUM_FEET> virt_feet_positions_;  // Private, since this will be normally
  // hidden behind the joint states
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
};

class BrickModel : public ModelInterface {
 public:
  Eigen::Matrix3d inertia_;
  double mass_;
  double g_;

  BrickModel(const Eigen::Matrix3d &inertia, double mass);
  Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx, const StateInterface &state) const override;

  Eigen::Matrix3d GetInertia() const override;
  double GetInertia(const int row, const int column) const override;

  double GetMass() const override;

  double GetG() const override;
  void SetInertia(const Eigen::Matrix3d &inertia_tensor) override;
  void SetInertia(const int row, const int column, const double value) override;
  void SetMass(double mass) override;
  void SetCOM(const Eigen::Vector3d &com) override;
  void SetCOM(const int idx, const double val) override;

  // void CalcFootForceVelocityInBodyFrame(
  //     int leg_index,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
  //     Eigen::Ref<Eigen::Vector3d> f_ee,
  //     Eigen::Ref<Eigen::Vector3d> v_ee) const override;
  void CalcFootForceVelocityInBodyFrame(
      int leg_index,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_accelerations,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
      Eigen::Ref<Eigen::Vector3d> f_ee,
      Eigen::Ref<Eigen::Vector3d> v_ee) const override;

  Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx,
                                         const Eigen::Vector3d &body_pos,
                                         const Eigen::Quaterniond &body_orientation,
                                         const Eigen::Vector3d &joint_positions) const override;

  Eigen::Vector3d GetFootPositionInBodyFrame(unsigned int foot_idx,
                                             const Eigen::Vector3d &joint_positions) const override;

  Eigen::Translation3d GetBodyToIMU() const override;
  void calcFootForceVelocityBodyFrame(int leg_index,
                                      const StateInterface &state,
                                      Eigen::Vector3d &f_ee,
                                      Eigen::Vector3d &v_ee) const override;
  void calcLegInverseKinematicsInBody(int leg_index,
                                      const Eigen::Vector3d &p_ee_B,
                                      const Eigen::Vector3d &joint_state_init_guess,
                                      Eigen::Vector3d &theta) const override;
  void calcLegDiffKinematicsBodyFrame(int leg_index,
                                      const StateInterface &state,
                                      Eigen::Vector3d &f_ee_goal,
                                      Eigen::Vector3d &v_ee_goal,
                                      Eigen::Vector3d &tau_goal,
                                      Eigen::Vector3d &qd_goal) const override;
  void calcJacobianLegBase(int leg_index, Eigen::Vector3d joint_pos, Eigen::Matrix3d &Jac_legBaseToFoot) const override;
  void calcFwdKinLegBody(int leg_indx,
                         const Eigen::Vector3d &joint_pos,
                         Eigen::Matrix4d &T_BodyToFoot,
                         Eigen::Vector3d &foot_pos_body) const override;

  void CalcBaseHeight(const std::array<bool, 4> &feet_contacts,
                      const std::array<const Eigen::Vector3d, 4> &joint_states,
                      const Eigen::Quaterniond &body_rpy,
                      double &base_height) const override;

  bool IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> &joint_states,
                   const std::array<double, ModelInterface::N_LEGS> &distance_threshold) const override;
  Eigen::Translation3d GetBodyToBellyBottom() const override;
  Eigen::Translation3d GetBodyToBellyBottom(unsigned int leg) const override;
  double ComputeKineticEnergy(int leg_index,
                              const Eigen::Vector3d &joint_pos,
                              const Eigen::Vector3d &joint_vel) const override;
  double ComputeEnergyDerivative(int leg_index,
                                 const Eigen::Vector3d &joint_vel,
                                 const Eigen::Vector3d &tau) const override;
  Eigen::Translation3d GetBodyToCOM() const override;
};

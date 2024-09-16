#pragma once

#include <Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include "state_interface.hpp"

class ModelInterface {
 protected:
  ModelInterface() = default;  // Abstract class cant be created
 public:
  static const int N_LEGS = 4;
  static const int N_JOINTS_PER_LEG = 3;
  static const int NUM_JOINTS = N_LEGS * N_JOINTS_PER_LEG;

  //   virtual void CalcFootForceVelocityInBodyFrame(
  //       int leg_index,
  //       const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
  //       const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
  //       const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
  //       Eigen::Ref<Eigen::Vector3d> f_ee,
  //       Eigen::Ref<Eigen::Vector3d> v_ee) const = 0;

  virtual void CalcFootForceVelocityInBodyFrame(
      int leg_index,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_accelerations,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
      Eigen::Ref<Eigen::Vector3d> f_ee,
      Eigen::Ref<Eigen::Vector3d> v_ee) const = 0;

  virtual void calcFootForceVelocityBodyFrame(int leg_index,
                                              const StateInterface &state,
                                              Eigen::Vector3d &f_ee,
                                              Eigen::Vector3d &v_ee) const = 0;

  virtual void calcLegInverseKinematicsInBody(int leg_index,
                                              const Eigen::Vector3d &p_ee_B,
                                              const Eigen::Vector3d &joint_state_init_guess,
                                              Eigen::Vector3d &theta) const = 0;

  virtual void calcLegDiffKinematicsBodyFrame(int leg_index,
                                              const StateInterface &state,
                                              Eigen::Vector3d &f_ee_goal,
                                              Eigen::Vector3d &v_ee_goal,
                                              Eigen::Vector3d &tau_goal,
                                              Eigen::Vector3d &qd_goal) const = 0;
  virtual void calcJacobianLegBase(int leg_index,
                                   Eigen::Vector3d joint_pos,
                                   Eigen::Matrix3d &Jac_legBaseToFoot) const = 0;

  virtual void calcFwdKinLegBody(int leg_indx,
                                 const Eigen::Vector3d &joint_pos,
                                 Eigen::Matrix4d &T_BodyToFoot,
                                 Eigen::Vector3d &foot_pos_body) const = 0;

  virtual Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx, const StateInterface &state) const = 0;
  virtual Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx,
                                                 const Eigen::Vector3d &body_pos,
                                                 const Eigen::Quaterniond &body_orientation,
                                                 const Eigen::Vector3d &joint_positions) const = 0;
  virtual Eigen::Vector3d GetFootPositionInBodyFrame(unsigned int foot_idx,
                                                     const Eigen::Vector3d &joint_positions) const = 0;
  virtual Eigen::Matrix3d GetInertia() const = 0;
  virtual double GetInertia(const int row, const int column) const = 0;
  virtual double GetMass() const = 0;
  virtual double GetG() const = 0;
  virtual Eigen::Translation3d GetBodyToIMU() const = 0;
  virtual Eigen::Translation3d GetBodyToBellyBottom() const = 0;
  virtual Eigen::Translation3d GetBodyToBellyBottom(unsigned int leg) const = 0;
  virtual Eigen::Translation3d GetBodyToCOM() const = 0;

  virtual void CalcBaseHeight(const std::array<bool, 4> &feet_contacts,
                              const std::array<const Eigen::Vector3d, 4> &joint_states,
                              const Eigen::Quaterniond &body_orientation,
                              double &base_height) const = 0;

  virtual bool IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> &joint_states,
                           const std::array<double, ModelInterface::N_LEGS> &distance_threshold) const = 0;

  virtual double ComputeKineticEnergy(int leg_index,
                                      const Eigen::Vector3d &joint_pos,
                                      const Eigen::Vector3d &joint_vel) const = 0;
  virtual double ComputeEnergyDerivative(int leg_index,
                                         const Eigen::Vector3d &joint_vel,
                                         const Eigen::Vector3d &tau) const = 0;
  virtual void SetInertia(const Eigen::Matrix3d &inertia_tensor) = 0;
  virtual void SetInertia(const int row, const int column, const double value) = 0;
  virtual void SetMass(double mass) = 0;
  virtual void SetCOM(const Eigen::Vector3d &com) = 0;
  virtual void SetCOM(const int idx, const double val) = 0;

  virtual ModelInterface &operator=(const ModelInterface &other) {
    (void)other;
    std::runtime_error("the assignment operator of ModelInterface must always be overridden by deriving classes");
    return *this;
  }
};
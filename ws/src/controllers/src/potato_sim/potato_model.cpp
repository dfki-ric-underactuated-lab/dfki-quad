#include "potato_sim/potato_model.hpp"

BrickModel::BrickModel(const Eigen::Matrix3d &inertia, double mass) : inertia_(inertia), mass_(mass), g_(9.81) {}

Eigen::Matrix3d BrickModel::GetInertia() const { return inertia_; }
double BrickModel::GetInertia(const int row, const int column) const { return inertia_(row, column); }

double BrickModel::GetMass() const { return mass_; }

double BrickModel::GetG() const { return g_; }

Eigen::Vector3d BrickModel::GetFootPositionInWorld(unsigned int foot_idx, const StateInterface &state) const {
  return dynamic_cast<const BrickState &>(state).virt_feet_positions_[foot_idx];
}
void BrickModel::SetInertia(const Eigen::Matrix3d &inertia_tensor) { inertia_ = inertia_tensor; }
void BrickModel::SetInertia(const int row, const int column, const double value) { inertia_(row, column) = value; }
void BrickModel::SetMass(double mass) { mass_ = mass; }
void BrickModel::SetCOM(const Eigen::Vector3d &com) {
  (void)com;
  // TODO: implement
  std::logic_error("Function not yet implemented");
}
void BrickModel::SetCOM(const int idx, const double val) {
  (void)idx;
  (void)val;
  // TODO: implement
  std::logic_error("Function not yet implemented");
}

const Eigen::Vector3d &BrickState::GetLinearAccInWorld() const { return linear_acc_; }
const Eigen::Vector3d &BrickState::GetAngularAccInWorld() const { return angular_acc_; }

const Eigen::Vector3d &BrickState::GetPositionInWorld() const { return position_; }

const Eigen::Quaterniond &BrickState::GetOrientationInWorld() const { return orientation_; }

const Eigen::Vector3d &BrickState::GetLinearVelInWorld() const { return linear_vel_; }

const Eigen::Vector3d &BrickState::GetAngularVelInWorld() const { return angular_vel_; }

const StateInterface::TimePoint &BrickState::GetTime() const { return time_stamp_; }

const std::array<bool, StateInterface::NUM_FEET> &BrickState::GetFeetContacts() const { return feet_contacts_; }

const std::array<Eigen::Vector3d, StateInterface::NUM_FEET> &BrickState::GetContactForces() const {
  static std::array<Eigen::Vector3d, StateInterface::NUM_FEET> nothing;
  return nothing;
}

const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET>
    &BrickState::GetJointPositions() const {
  static std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> nothing;
  return nothing;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET>
    &BrickState::GetJointVelocities() const {
  static std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> nothing;
  return nothing;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET>
    &BrickState::GetJointAccelerations() const {
  static std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> nothing;
  return nothing;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET>
    &BrickState::GetJointTorques() const {
  static std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> nothing;
  return nothing;
}

StateInterface &BrickState::operator=(const StateInterface &other) {
  return BrickState::operator=(dynamic_cast<const BrickState &>(other));
}

void BrickModel::CalcFootForceVelocityInBodyFrame(
    int leg_index,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_accelerations,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
    Eigen::Ref<Eigen::Vector3d> f_ee,
    Eigen::Ref<Eigen::Vector3d> v_ee) const {
  (void)leg_index;
  (void)joint_positions;
  (void)joint_velocities;
  (void)joint_accelerations;
  (void)joint_torques;
  (void)f_ee;
  (void)v_ee;
  throw std::runtime_error("CalcFootForceVelocityInBodyFrame Not implement, as this would be non sense");
}

Eigen::Vector3d BrickModel::GetFootPositionInWorld(unsigned int foot_idx,
                                                   const Eigen::Vector3d &body_pos,
                                                   const Eigen::Quaterniond &body_orientation,
                                                   const Eigen::Vector3d &joint_positions) const {
  (void)foot_idx;
  (void)body_pos;
  (void)body_orientation;
  (void)joint_positions;
  throw std::runtime_error("GetFootPositionInWorld Not implement, as this doesnt work");
}

Eigen::Vector3d BrickModel::GetFootPositionInBodyFrame(unsigned int foot_idx,
                                                       const Eigen::Vector3d &joint_positions) const {
  (void)foot_idx;
  (void)joint_positions;
  throw std::runtime_error("GetFootPositionInBodyFrame Not implement, as this doesnt work");
}

void BrickModel::calcFootForceVelocityBodyFrame(int leg_index,
                                                const StateInterface &state,
                                                Eigen::Vector3d &f_ee,
                                                Eigen::Vector3d &v_ee) const {
  (void)leg_index;
  (void)state;
  (void)f_ee;
  (void)v_ee;
  throw std::runtime_error("calcFootForceVelocityBodyFrame Not implement, as this doesnt work");
}
void BrickModel::calcLegInverseKinematicsInBody(int leg_index,
                                                const Eigen::Vector3d &p_ee_B,
                                                const Eigen::Vector3d &joint_state_init_guess,
                                                Eigen::Vector3d &theta) const {
  (void)leg_index;
  (void)p_ee_B;
  (void)theta;
  (void)joint_state_init_guess;
  throw std::runtime_error("calcLegInverseKinematicsInBody Not implement, as this doesnt work");
}
void BrickModel::calcLegDiffKinematicsBodyFrame(int leg_index,
                                                const StateInterface &state,
                                                Eigen::Vector3d &f_ee_goal,
                                                Eigen::Vector3d &v_ee_goal,
                                                Eigen::Vector3d &tau_goal,
                                                Eigen::Vector3d &qd_goal) const {
  (void)leg_index;
  (void)state;
  (void)f_ee_goal;
  (void)v_ee_goal;
  (void)tau_goal;
  (void)qd_goal;
  throw std::runtime_error("calcLegDiffKinematicsBodyFrame Not implement, as this doesnt work");
}
void BrickModel::calcJacobianLegBase(int leg_index,
                                     Eigen::Vector3d joint_pos,
                                     Eigen::Matrix3d &Jac_legBaseToFoot) const {
  (void)leg_index;
  (void)joint_pos;
  (void)Jac_legBaseToFoot;
  throw std::runtime_error("calcJacobianLegBase Not implement, as this doesnt work");
}
void BrickModel::calcFwdKinLegBody(int leg_indx,
                                   const Eigen::Vector3d &joint_pos,
                                   Eigen::Matrix4d &T_BodyToFoot,
                                   Eigen::Vector3d &foot_pos_body) const {
  (void)leg_indx;
  (void)joint_pos;
  (void)T_BodyToFoot;
  (void)foot_pos_body;
  if (joint_pos.isZero()) {
    // Done for max height calculation:
    foot_pos_body = {0.0, 0.0, -0.3};
  } else {
    throw std::runtime_error("calcFwdKinLegBody Not implement, as this doesnt work");
  }
}

Eigen::Translation3d BrickModel::GetBodyToIMU() const { return {0, 0, 0}; }
void BrickModel::CalcBaseHeight(const std::array<bool, 4> &feet_contacts,
                                const std::array<const Eigen::Vector3d, 4> &joint_states,
                                const Eigen::Quaterniond &body_rpy,
                                double &base_height) const {
  (void)feet_contacts;
  (void)joint_states;
  (void)body_rpy;
  (void)base_height;
  throw std::runtime_error("CalcBaseHeight Not implement: TODO");
}

bool BrickModel::IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> &joint_states,
                             const std::array<double, ModelInterface::N_LEGS> &distance_threshold) const {
  (void)joint_states;
  (void)distance_threshold;
  throw std::runtime_error("isLyingDonw Not implemented: TODO");
}
Eigen::Translation3d BrickModel::GetBodyToBellyBottom() const {
  throw std::runtime_error("GetBodyToBellyBottom Not implemented: TODO");
}
Eigen::Translation3d BrickModel::GetBodyToBellyBottom(unsigned int leg) const {
  (void)leg;
  throw std::runtime_error("GetBodyToBellyBottom (leg) Not implemented: TODO");
}

double BrickModel::ComputeKineticEnergy(int leg_index,
                                        const Eigen::Vector3d &joint_pos,
                                        const Eigen::Vector3d &joint_vel) const {
  (void)leg_index;
  (void)joint_pos;
  (void)joint_vel;
  throw std::runtime_error("ComputeEnergyDerivative Not implement, as this doesnt work");
}
double BrickModel::ComputeEnergyDerivative(int leg_index,
                                           const Eigen::Vector3d &joint_vel,
                                           const Eigen::Vector3d &tau) const {
  (void)leg_index;
  (void)joint_vel;
  (void)tau;
  throw std::runtime_error("ComputeEnergyDerivative Not implement, as this doesnt work");
}
Eigen::Translation3d BrickModel::GetBodyToCOM() const { return Eigen::Translation3d::Identity(); }

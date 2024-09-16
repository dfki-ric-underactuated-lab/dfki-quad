#include "quad_model_symbolic.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

QuadModelSymbolic::QuadModelSymbolic(RobotModel model) {
  legbase_offsets_.resize(4);
  hip_offsets_.resize(4);
  T_body_legbases_.resize(4);

  if (model == UNITREE_QUAD) {
    std::cout << "QuadModelSymbolic initialized with UNITREE_QUAD" << std::endl;
    base_mass_ = 15.019;
    //    base_inertia_ << 0.228452, 0.00012166, -0.00231383, 0.00012166, 0.515114, -3.12e-05, -0.00231383, -3.12e-05,
    //        0.500952;

    base_inertia_ << 0.202755, 0.00012166, -0.0143742, 0.00012166, 0.490926, -3.12e-05, -0.0143742, -3.12e-05, 0.52697;

    l0_ = 0.0955;
    l1_ = 0.213;
    l2_ = 0.213;

    bodyWidth_ = 0.0465 * 2;   // 0.0465*2
    bodyLength_ = 0.1934 * 2;  // 0.1934*2
    bodyHeight_ = 0.0;

    belly_bottom_in_base_frame_ = {0.0, 0.0, -0.057};
    com_offset_ = {0.0, 0.0, 0.0};  // TODO: SET!

    // HAA link
    m1 = 0.678, c1x = -0.0054, c1y = 0.00194, c1z = -0.000105;
    I1xx = 0.00048, I1yy = 0.000884, I1zz = 0.000596, I1yz = -1.42E-06, I1xz = 1.11E-06, I1xy = -3.01E-06;

    // HFE link
    m2 = 1.152, c2x = -0.00374, c2y = -0.0223, c2z = -0.0327;
    I2xx = 0.00584, I2yy = 0.0058, I2zz = 0.00103, I2yz = 0.000808, I2xz = -0.000289, I2xy = 8.72E-05;

    // KFE link
    m3 = 0.154, c3x = 0.00548, c3y = -0.000975, c3z = -0.115;
    I3xx = 0.00108, I3yy = 0.0011, I3zz = 3.29E-05, I3yz = 8.28E-06, I3xz = 1.72E-05, I3xy = 3.4E-07;

    Vector3d legbase_offset(bodyLength_ / 2, bodyWidth_ / 2, 0);
    Vector3d hip_offset(0, l0_, 0);

    // For front left leg, frame is same as world frame
    abad_offset_ = {0, 0, 0};
    // hip_offset_ = {0,l0_,0};
    knee_offset_ = {0, 0, -l1_};
    foot_offset_ = {0, 0, -l2_};
    for (uint i = 0; i < 4; i++) {
      legbase_offsets_[i] = withLegSigns(legbase_offset, i);
      hip_offsets_[i] = withLegSigns(hip_offset, i);
      T_body_legbases_[i].setIdentity();
      T_body_legbases_[i].block(0, 3, 3, 1) = legbase_offsets_[i];
    }
    joint_min_.resize(num_feet_);
    joint_max_.resize(num_feet_);

    // joint min max constraints for all 3 joints -> {abad, shoulder, knee}
    std::vector<double> joint_min_front = {-1.0472, -1.5708, -2.7227};
    std::vector<double> joint_max_front = {1.0472, 3.4907, -0.83776};

    joint_min_ = {joint_min_front, joint_min_front, joint_min_front, joint_min_front};
    joint_max_ = {joint_max_front, joint_max_front, joint_max_front, joint_max_front};
    imu_offset_ << -0.02557, 0.0, 0.04232;
  } else if (model == DFKI_QUAD) {
    std::cout << "QuadModelSymbolic initialized with DFKI_QUAD" << std::endl;
    l0_ = 0.0878;
    l1_ = 0.15;
    l2_ = 0.16852;
    bodyLength_ = 0.181 * 2;
    bodyWidth_ = 0.064645 * 2;
    bodyHeight_ = 0.0;

    belly_bottom_in_base_frame_ = {0.0, 0.0, -0.06};
    com_offset_ = {0.0, 0.0, 0.0};  // TODO: SET!

    // all COM values for the links are for the FRONT LEFT leg
    // HAA link
    m1 = 0.595958341886362, c1x = -0.00468975539369312, c1y = -0.00428992245344755, c1z = 0.0;
    I1xx = 0.000404243397536089, I1yy = 0.000680157569347147, I1zz = 0.000416108103085786;
    I1yz = 6.17856098177554E-14, I1xz = 7.31545153465743E-18, I1xy = 1.14966094557709E-06;

    // HFE link
    m2 = 0.839506618858938, c2x = -0.000473484460610213, c2y = -0.0313838995140919, c2z = -0.0139105757410481;
    I2xx = 0.000957798668104474, I2yy = 0.00131919990491172, I2zz = 0.000501025769916749;
    I2yz = -1.14960846692485E-06, I2xz = -4.16617902958463E-06, I2xy = -2.97990154477032E-11;

    // KFE link
    c3x = 0.00499709183309199, c3y = -3.10682654602878E-09, c3z = -0.0402416877508378;
    // I3xx = 0.000172039837493548, I3yy = 0.000196849936261391, I3zz = 2.70201230114763E-05;
    // I3yz = -1.07919183413223E-10, I3xz = 2.02054840363873E-05, I3xy = 1.12182134513165E-10;
    I3xx = 0.000181004, I3yy = 0.000237234, I3zz = 9.04278e-05;
    I3yz = 5.02176e-05, I3xz = -8.27224e-05, I3xy = 2.55891e-05;
    m3 = 0.12757490021694 + 0.00342797071343661;  // adding foot mass
    //  Joint names
    joint_names_ = {"fl_abad",
                    "fl_hip",
                    "fl_knee",
                    "fr_abad",
                    "fr_hip",
                    "fr_knee",
                    "bl_abad",
                    "bl_hip",
                    "bl_knee",
                    "br_abad",
                    "br_hip",
                    "br_knee"};

    // Base mass and inertia
    base_mass_ = 10.8475;
    base_inertia_ << 0.0989703, 0.00197077, -0.00641265, 0.00197077, 0.235549, -0.00250561, -0.00641265, -0.00250561,
        0.287646;
    Vector3d legbase_offset(bodyLength_ / 2, bodyWidth_ / 2, bodyHeight_);
    Vector3d hip_offset(0, l0_, 0);

    // For front left leg, frame is same as world frame
    abad_offset_ = {0, 0, 0};
    // hip_offset_ = {0,l0_,0};
    knee_offset_ = {0, 0, -l1_};
    foot_offset_ = {0, 0, -l2_};
    for (uint i = 0; i < 4; i++) {
      legbase_offsets_[i] = withLegSigns(legbase_offset, i);
      hip_offsets_[i] = withLegSigns(hip_offset, i);
      T_body_legbases_[i].setIdentity();
      T_body_legbases_[i].block(0, 3, 3, 1) = legbase_offsets_[i];
    }
    joint_min_.resize(num_feet_);
    joint_max_.resize(num_feet_);

    std::vector<double> joint_min_front = {-1.5708, -4.398, -2.5};
    // std::vector<double> joint_min_back = {-0.707, -M_PI, 0};
    std::vector<double> joint_max_front = {1.5708, 4.398, 1.5708};
    // std::vector<double> joint_max_back = {0.707, M_PI * 0.5, M_PI};

    joint_min_ = {joint_min_front, joint_min_front, joint_min_front, joint_min_front};
    joint_max_ = {joint_max_front, joint_max_front, joint_max_front, joint_max_front};
    imu_offset_ << -0.0005, 0.098729, 0.0;
  }

  T_base_imu_ = createTransformationMatrix(imu_offset_, Eigen::Quaterniond(1.0, 0., 0., 0.));
  std::cout << "Initialization complete" << std::endl;
}

Vector3d QuadModelSymbolic::withLegSigns(const Vector3d &v, int legID) {
  switch (legID) {
    case 0:
      return Vector3d(v[0], v[1], v[2]);
    case 1:
      return Vector3d(v[0], -v[1], v[2]);
    case 2:
      return Vector3d(-v[0], v[1], v[2]);
    case 3:
      return Vector3d(-v[0], -v[1], v[2]);
    default:
      throw std::runtime_error("Invalid leg id!");
  }
}

// void QuadModelSymbolic::calcFootForceVelocityBodyFrame(int leg_index,
//                                                        const StateInterface &state,
//                                                        Vector3d &f_ee,
//                                                        Vector3d &v_ee) const {
//   CalcFootForceVelocityInBodyFrame(leg_index,
//                                    Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[leg_index].data()),
//                                    Eigen::Map<const Eigen::Vector3d>(state.GetJointVelocities()[leg_index].data()),
//                                    Eigen::Map<const Eigen::Vector3d>(state.GetJointTorques()[leg_index].data()),
//                                    f_ee,
//                                    v_ee);
// }

void QuadModelSymbolic::calcFootForceVelocityBodyFrame(int leg_index,
                                                       const StateInterface &state,
                                                       Vector3d &f_ee,
                                                       Vector3d &v_ee) const {
  CalcFootForceVelocityInBodyFrame(leg_index,
                                   Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[leg_index].data()),
                                   Eigen::Map<const Eigen::Vector3d>(state.GetJointVelocities()[leg_index].data()),
                                   Eigen::Map<const Eigen::Vector3d>(state.GetJointAccelerations()[leg_index].data()),
                                   Eigen::Map<const Eigen::Vector3d>(state.GetJointTorques()[leg_index].data()),
                                   f_ee,
                                   v_ee);
}

// void QuadModelSymbolic::CalcFootForceVelocityInBodyFrame(
//     int leg_index,
//     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
//     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
//     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
//     Eigen::Ref<Eigen::Vector3d> f_ee,
//     Eigen::Ref<Eigen::Vector3d> v_ee) const {
//   Matrix3d J_leg, J_leg_inv;
//   // int leg_index = get_LegIndex(leg);

//   calcJacobianLegBase(leg_index, joint_positions, J_leg);
//   // J_leg_inv = J_leg.inverse();
//   J_leg_inv = J_leg.transpose().completeOrthogonalDecomposition().pseudoInverse();
//   // joint to cartesian mappings
//   f_ee = J_leg_inv * joint_torques;
//   v_ee = J_leg * joint_velocities;
// }

void QuadModelSymbolic::CalcFootForceVelocityInBodyFrame(
    int leg_index,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_positions,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_velocities,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_accelerations,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>> &joint_torques,
    Eigen::Ref<Eigen::Vector3d> f_ee,
    Eigen::Ref<Eigen::Vector3d> v_ee) const {
  Matrix3d J_leg, J_leg_inv;
  // int leg_index = get_LegIndex(leg);

  calcJacobianLegBase(leg_index, joint_positions, J_leg);
  J_leg_inv = J_leg.transpose().completeOrthogonalDecomposition().pseudoInverse();
  Vector3d idyn;
  computeInverseDynamics(leg_index, joint_positions, joint_velocities, joint_accelerations, idyn);

  // joint to cartesian mappings
  f_ee = J_leg_inv * (joint_torques - idyn);
  v_ee = J_leg * joint_velocities;
}

void QuadModelSymbolic::calcLegDiffKinematicsBodyFrame(int leg_index,
                                                       const StateInterface &state,
                                                       Vector3d &f_ee_goal,
                                                       Vector3d &v_ee_goal,
                                                       Vector3d &tau_goal,
                                                       Vector3d &qd_goal) const {
  Matrix3d J_leg, J_leg_inv;
  // int leg_index = get_LegIndex(leg);
  calcJacobianLegBase(leg_index, Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[leg_index].data()), J_leg);
  J_leg_inv = J_leg.inverse();

  // cartesian to joint mappings
  tau_goal = J_leg.transpose() * f_ee_goal;  // f must be in body frame
  qd_goal = J_leg_inv * v_ee_goal;
}
void shoulderTransform(
    const double &theta, const double &l1, const bool &right, const Eigen::Vector3d &p_ee, Eigen::Vector3d &p_ee_rot) {
  double angle, offset;
  if (right) {
    angle = theta + M_PI;
    offset = l1;
  } else {
    angle = theta;
    offset = -l1;
  }

  Eigen::Vector3d p_l;
  p_l << 0.0, offset, 0.0;
  Eigen::Matrix3d Rx;  // rotate around x clockwise
  Rx << 1.0, 0.0, 0.0, 0.0, std::cos(-angle), -std::sin(-angle), 0.0, std::sin(-angle), std::cos(-angle);
  p_ee_rot = Rx * p_ee;
  p_ee_rot += p_l;
}

void QuadModelSymbolic::calcLegInverseKinematicsInBody(int leg_index,
                                                       const Eigen::Vector3d &p_ee_B,
                                                       const Eigen::Vector3d &joint_state_init_guess,
                                                       Eigen::Vector3d &theta) const {
  // transform from shoulder to body
  Eigen::Vector3d p_ee_S = Eigen::Affine3d(T_body_legbases_[leg_index].inverse()) * p_ee_B;

  // get inverse Kinematics in shoulder frame
  this->calcLegInverseKinematicsInLegFrame(leg_index, p_ee_S, joint_state_init_guess, theta);

  // fix zero pos and set directions to face in the right direction (TODO:
  // take from conf)
  // int side = get_sideSignLeg(leg_index);
  // if (side == -1) {  // if right leg
  //   theta[0] += M_PI;
  // }

  // theta[1] = -theta[1];
  // theta[2] = -theta[2];

  // for (int i = 0; i < 3; i++) {
  //   theta[i] = map_angle(theta[i]);
  // }
}

// calculate inverse dynamics for a given leg
/*
void QuadModelSymbolic::calcLegInverseKinematicsInLegFrame(int leg_index,
                                                           const Vector3d &p_ee,
                                                           Vector3d &joint_state) const {
  double x, y, z, l1, l2, l3, gamma, theta1, theta2, theta3, A, alpha1, alpha2, alpha3, xhat, yhat, zhat, beta1, beta2,
      beta3, B;
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  bool right = false;
  if (sideSign == -1) {
    right = true;
  }

  // unpack ee position
  x = p_ee[0];
  y = p_ee[1];
  z = p_ee[2];

  gamma = M_PI_2;  // angle of the shoulder
  l1 = l0_;
  l2 = l1_;
  l3 = l2_;

  // calculate tehta1
  q0 = acos(temp) + atan2(z, y);
  A = std::sqrt(x * x + z * z);
  alpha1 = std::atan2(z, y);
  alpha2 = std::asin(std::clamp((l1 / A) * std::sin(gamma), -1.0, 1.0));
  alpha3 = M_PI - gamma - alpha2;

  if (right) {
    theta1 = alpha1 - alpha3;
  } else {
    theta1 = alpha1 + alpha3;
  }

  // Rotate coordinate system before calculating theta2 and theta3
  Eigen::Vector3d p_ee_rot;
  shoulderTransform(theta1,
                    l1,
                    right,
                    p_ee,
                    p_ee_rot);  // express p_ee in a different frame that sits
                                // at the "end" of the shoulder
  xhat = p_ee_rot[0];
  yhat = p_ee_rot[1];
  zhat = p_ee_rot[2];

  B = std::sqrt(xhat * xhat + zhat * zhat);
  beta1 = std::atan2(zhat, xhat);
  beta2 = std::acos(std::clamp((B * B + l2 * l2 - l3 * l3) / (2 * B * l2), -1., 1.));
  beta3 = std::acos(std::clamp((l2 * l2 + l3 * l3 - B * B) / (2 * l2 * l3), -1., 1.));

  theta2 = beta1 - beta2 + M_PI_2;
  theta3 = M_PI - beta3;

  joint_state << theta1, theta2, theta3;

  if (std::isnan(theta1) or std::isnan(theta2) or std::isnan(theta3)) {
    std::cout << "!!! NAN detected in calcLegInverseKinematicsInLegFrame: " << std::endl;  // TODO: handle
    std::printf(
        "\t leg_idx=%i p_ee=[%f,%f,%f] => [%f,%f,%f]", leg_index, p_ee(0), p_ee(1), p_ee(2), theta1, theta2, theta3);
  }
}
*/
/*
void QuadModelSymbolic::calcLegInverseKinematicsInLegFrame(int leg_index,
                                                           const Vector3d &foot_pos_legbase,
                                                           Vector3d &joint_state) const {
  // Initialize exact bool
  bool is_exact = true;
  // int leg_index = get_LegIndex(leg);
  // Calculate offsets
  Vector3d legbase_offset = legbase_offsets_[leg_index];
  double l0 = hip_offsets_[leg_index](1);
  std::cout << "hip offset  : " << l0 << std::endl;
  // Extract coordinates and declare joint variables
  double x = foot_pos_legbase[0];
  double y = foot_pos_legbase[1];
  double z = foot_pos_legbase[2];
  double q0;
  double q1;
  double q2;
  std::cout << "foot pos : " << foot_pos_legbase << std::endl;
  // Start IK, check foot pos is at least l0 away from leg base, clamp otherwise
  double temp = l0 / sqrt(z * z + y * y);
  if (std::abs(temp) > 1) {
    std::cerr << "Foot too close, choosing closest alternative\n ";
    is_exact = false;
    temp = std::max(std::min(temp, 1.0), -1.0);
  }
  std::cout << "temp : " << temp << std::endl;
  // Compute both solutions of q0, use hip-above-knee if z<0 (preferred)
  // Store the inverted solution in case hip limits are exceeded
  if (z > 0) {
    q0 = -acos(temp) + atan2(z, y);
  } else {
    q0 = acos(temp) + atan2(z, y);
  }

  // Make sure abad is within joint limits, clamp otherwise
  if (q0 > joint_max_[leg_index][0] || q0 < joint_min_[leg_index][0]) {
    q0 = std::max(std::min(q0, joint_max_[leg_index][0]), joint_min_[leg_index][0]);
    is_exact = false;
    std::cerr << "Abad limits exceeded, clamping to" << q0 << "\n";
  }

  // // Rotate to ab-ad fixed frame
  double z_body_frame = z;
  z = -sin(q0) * y + cos(q0) * z_body_frame;

  // Check reachibility for hip
  double acos_eps = 1.0;
  double temp2 = (l1_ * l1_ + x * x + z * z - l2_ * l2_) / (2 * l1_ * sqrt(x * x + z * z));
  std::cout << "temp2 : " << temp2 << std::endl;
  if (abs(temp2) > acos_eps) {
    std::cerr << "Foot location too far for hip, choosing closest alternative \n ";
    is_exact = false;
    temp2 = std::max(std::min(temp2, acos_eps), -acos_eps);
  }
  // Check reachibility for knee
  double temp3 = (l1_ * l1_ + l2_ * l2_ - x * x - z * z) / (2 * l1_ * l2_);

  if (temp3 > acos_eps || temp3 < -acos_eps) {
    std::cerr << "Foot location too far for knee, choosing closest alternative \n ";
    is_exact = false;
    temp3 = std::max(std::min(temp3, acos_eps), -acos_eps);
  }

  // Compute joint angles
  q1 = 0.5 * M_PI + atan2(x, -z) - acos(temp2);
  std::cout << " Joint angle q1 : " << q1 << std::endl;
  // Make sure hip is within joint limits
  if (q1 > joint_max_[leg_index][1] || q1 < joint_min_[leg_index][1]) {
    q1 = std::max(std::min(q1, joint_max_[leg_index][1]), joint_min_[leg_index][1]);
    is_exact = false;
    std::cerr << "Hip limits exceeded, clamping to" << q1 << " \n";
  }

  // Compute knee val to get closest toe position in the plane
  Vector2d knee_pos, toe_pos, toe_offset;
  knee_pos << -l1_ * sin(q1), -l1_ * cos(q1);
  toe_pos << x, z;
  toe_offset = toe_pos - knee_pos;
  q2 = atan2(-toe_offset(1), toe_offset(0)) + q1;
  std::cout << " Toe offset: " << toe_offset << std::endl;
  std::cout << " Joint angle q2 : " << q2 << std::endl;

  // ROHIT Update
  // double z_hat = (z - l0 * sin(q0)) / cos(q0);
  // q2 = std::acos((x * x + z_hat * z_hat - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_));
  // std::cout << (x * x + z_hat * z_hat - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_) << std::endl;
  // q1 = 0;
  // Make sure knee is within joint limits
  if (q2 > joint_max_[leg_index][2] || q2 < joint_min_[leg_index][2]) {
    q2 = std::max(std::min(q2, joint_max_[leg_index][2]), joint_min_[leg_index][2]);
    is_exact = false;
    std::cerr << "Knee limit exceeded, clamping to" << q2 << " \n";
  }

  // q1 is undefined if q2=0, resolve this
  if (q2 == 0) {
    q1 = 0;
    std::cerr << "Hip value undefined (in singularity), setting to" << q1 << "\n";
    is_exact = false;
  }

  if (z_body_frame - l0 * sin(q0) > 0) {
    std::cerr << "IK solution is in hip-inverted region !Beware !\n ";
    is_exact = false;
  }

  joint_state = {q0, q1, q2};
  std::cout << " Joint angles : \n " << joint_state << std::endl;
  // return is_exact;
}
*/
void QuadModelSymbolic::calcLegInverseKinematicsInLegFrame(int leg_index,
                                                           const Vector3d &foot_pos_legbase,
                                                           const Vector3d &joint_state_init_guess,
                                                           Vector3d &joint_state) const {
  // initializations
  Matrix4d T_leg_foot;
  Matrix3d Jac;
  Vector3d joint_state_prior_est, foot_pos_current;
  joint_state_prior_est = joint_state_init_guess;
  joint_state = Vector3d::Zero();
  calcFwdKinLegBase(leg_index, joint_state_prior_est, T_leg_foot, foot_pos_current);
  Vector3d error = (foot_pos_legbase - foot_pos_current);
  int max_iter = 1000, iter = 0;

  while (error.norm() > 1e-8 && iter < max_iter) {
    calcJacobianLegBase(leg_index, joint_state_prior_est, Jac);
    joint_state = joint_state_prior_est + Jac.completeOrthogonalDecomposition().pseudoInverse() * error;
    calcFwdKinLegBase(leg_index, joint_state, T_leg_foot, foot_pos_current);
    error = (foot_pos_legbase - foot_pos_current);
    joint_state_prior_est = joint_state;
    iter++;
  }
  if (iter == max_iter) {
    std::cerr << "Inv Kin: Max iteration reached."
              << "\n";
    // std::cerr << "Last update joint angles : " << joint_state << "\n";
  }
  // if (error.norm() < 1e-6) {
  //   std::cout << "Solution found for Inv Kin. with iter " << iter << std::endl;
  // } else {
  //   std::cout << "Solution NOT found." << std::endl;
  // }
  if (joint_state(0) > joint_max_[leg_index][0] || joint_state(0) < joint_min_[leg_index][0]) {
    joint_state(0) = std::max(std::min(joint_state(0), joint_max_[leg_index][0]), joint_min_[leg_index][0]);
    std::cerr << "Abad limits exceeded, clamping to" << joint_state(0) << "\n";
  }
  if (joint_state(1) > joint_max_[leg_index][1] || joint_state(1) < joint_min_[leg_index][1]) {
    joint_state(1) = std::max(std::min(joint_state(1), joint_max_[leg_index][1]), joint_min_[leg_index][1]);
    std::cerr << "Hip limits exceeded, clamping to" << joint_state(1) << " \n";
  }
  if (joint_state(2) > joint_max_[leg_index][2] || joint_state(2) < joint_min_[leg_index][2]) {
    joint_state(2) = std::max(std::min(joint_state(2), joint_max_[leg_index][2]), joint_min_[leg_index][2]);
    std::cerr << "Knee limit exceeded, clamping to" << joint_state(2) << " \n";
  }
}
void QuadModelSymbolic::CalcBaseHeight(const std::array<bool, 4> &feet_contacts,
                                       const std::array<const Eigen::Vector3d, 4> &joint_states,
                                       const Eigen::Quaterniond &body_orientation,
                                       double &base_height) const {
  double sum_z = 0;
  int num_contact_feet = 0;
  Eigen::Vector3d foot_pos_body;
  for (int i = 0; i < 4; i++) {
    if (feet_contacts[i]) {
      QuadModelSymbolic::calcBodyToFootFKBodyFrame(i, joint_states[i], body_orientation, foot_pos_body);
      sum_z += -foot_pos_body(2);
      num_contact_feet += 1;
    }
  }
  base_height = sum_z / num_contact_feet;
}

Matrix4d QuadModelSymbolic::createTransformationMatrix(const Vector3d &trans, const Quaterniond &quat) {
  auto t = Translation3d(trans) * Affine3d(quat);
  return t.matrix();
}

Matrix4d QuadModelSymbolic::createTransformationMatrix(const Vector3d &trans, const AngleAxisd &rot) {
  Transform<double, 3, Affine> t;
  t = Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

void QuadModelSymbolic::transformBodyToWorld(const Vector3d &body_pos,
                                             const Quaterniond &body_quat,
                                             const Matrix4d &transform_body,
                                             Matrix4d &transform_world) {
  // Compute transform from world to body frame
  Matrix4d T_world_body = createTransformationMatrix(body_pos, body_quat);

  // Get the desired transform in the world frame
  transform_world = T_world_body * transform_body;
}
void QuadModelSymbolic::transformWorldToBody(const Vector3d &body_pos,
                                             const Quaterniond &body_quat,
                                             const Matrix4d &transform_world,
                                             Matrix4d &transform_body) {
  // Compute transform from world to body frame
  Matrix4d T_world_body = createTransformationMatrix(body_pos, body_quat);

  // Compute the desired transform in the body frame
  transform_body = T_world_body.inverse() * transform_world;
}

void QuadModelSymbolic::calcWorldToLegbaseFKWorldFrame(int leg_index,
                                                       const Vector3d &body_pos,
                                                       const Quaterniond &body_quat,
                                                       Matrix4d &T_world_legbase,
                                                       Vector3d &leg_base_pos_world) const {
  Matrix4d T_world_body = createTransformationMatrix(body_pos, body_quat);
  // int leg_index = get_LegIndex(leg);
  // Compute transform for leg base relative to the world frame
  T_world_legbase = T_world_body * T_body_legbases_[leg_index];
  leg_base_pos_world = T_world_legbase.block<3, 1>(0, 3);
}

void QuadModelSymbolic::calcBodyToFootFKBodyFrame(int leg_index,
                                                  const Vector3d &joint_state,
                                                  const Quaterniond &body_quat,
                                                  Vector3d &foot_pos_body) const {
  Matrix4d T_legBase_foot = Matrix4d::Zero();
  Matrix4d T_body_foot = Matrix4d::Zero();
  Vector3d p_legBase_foot = Vector3d::Zero();
  // int leg_index = get_LegIndex(leg);
  calcFwdKinLegBase(leg_index, joint_state, T_legBase_foot, p_legBase_foot);
  // Only consider body orientation
  // std::cout << T_legBase_foot << std::endl;
  T_body_foot = createTransformationMatrix(Vector3d(0, 0, 0), body_quat) * T_body_legbases_[leg_index] * T_legBase_foot;
  foot_pos_body = T_body_foot.block<3, 1>(0, 3);
}

Eigen::Vector3d QuadModelSymbolic::GetFootPositionInBodyFrame(unsigned int foot_idx,
                                                              const Eigen::Vector3d &joint_positions) const {
  Eigen::Vector3d pos;
  calcBodyToFootFKBodyFrame(foot_idx, joint_positions, Eigen::Quaterniond(1, 0, 0, 0), pos);
  return pos;
}

void QuadModelSymbolic::calcWorldToFootFKWorldFrame(int leg_index,
                                                    const Vector3d &body_pos,
                                                    const Quaterniond &body_quat,
                                                    const Vector3d &joint_state,
                                                    Vector3d &foot_pos_world) const {
  Matrix4d T_legBase_foot = Matrix4d::Zero();
  Vector3d p_legBase_foot = Vector3d::Zero();
  Matrix4d T_world_legbase = Matrix4d::Zero();
  Matrix4d T_world_foot = Matrix4d::Zero();
  Vector3d leg_base_pos_world = Vector3d::Zero();
  calcWorldToLegbaseFKWorldFrame(leg_index, body_pos, body_quat, T_world_legbase, leg_base_pos_world);
  calcFwdKinLegBase(leg_index, joint_state, T_legBase_foot, p_legBase_foot);
  T_world_foot = T_world_legbase * T_legBase_foot;
  foot_pos_world = T_world_foot.block<3, 1>(0, 3);
}

void QuadModelSymbolic::calcWorldToFootIKWorldFrame(int leg_index,
                                                    const Vector3d &body_pos,
                                                    const Quaterniond &body_quat,
                                                    const Vector3d &foot_pos_world,
                                                    const Vector3d &joint_state_initial_guess,
                                                    Vector3d &joint_state) const {
  Matrix4d T_world_legbase = Matrix4d::Zero();
  Vector3d leg_base_pos_world = Vector3d::Zero();
  Matrix4d T_legBase_foot = Matrix4d::Zero();
  Vector3d foot_pos_legbase = Vector3d::Zero();
  calcWorldToLegbaseFKWorldFrame(leg_index, body_pos, body_quat, T_world_legbase, leg_base_pos_world);
  Matrix4d T_world_foot = Matrix4d::Zero();
  T_world_foot = createTransformationMatrix(foot_pos_world, Quaterniond(1, 0, 0, 0));
  T_legBase_foot = T_world_legbase.inverse() * T_world_foot;
  foot_pos_legbase = T_legBase_foot.block<3, 1>(0, 3);
  calcLegInverseKinematicsInLegFrame(leg_index, foot_pos_legbase, joint_state_initial_guess, joint_state);
}

void QuadModelSymbolic::calcBodyToFootIKBodyFrame(int leg_index,
                                                  const Quaterniond &body_quat,
                                                  const Vector3d &foot_pos_body,
                                                  const Vector3d &joint_state_initial_guess,
                                                  Vector3d &joint_state) const {
  Matrix4d T_body_legbase = Matrix4d::Zero();
  Vector3d leg_base_pos_body = Vector3d::Zero();
  Matrix4d T_legBase_foot = Matrix4d::Zero();
  Vector3d foot_pos_legbase = Vector3d::Zero();
  calcWorldToLegbaseFKWorldFrame(leg_index, Vector3d(0, 0, 0), body_quat, T_body_legbase, leg_base_pos_body);
  Matrix4d T_body_foot = createTransformationMatrix(foot_pos_body, Quaterniond(1, 0, 0, 0));
  T_legBase_foot = T_body_legbase.inverse() * T_body_foot;
  foot_pos_legbase = T_legBase_foot.block<3, 1>(0, 3);
  calcLegInverseKinematicsInLegFrame(leg_index, foot_pos_legbase, joint_state_initial_guess, joint_state);
}

void QuadModelSymbolic::calcFwdKinLegBase(int leg_index,
                                          const Vector3d &joint_pos,
                                          Matrix4d &T_legBaseToFoot,
                                          Vector3d &foot_pos_legbase) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  T_legBaseToFoot.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q1);
  double t6 = std::sin(q2);
  double t7 = std::sin(q3);
  double t8 = q2 + q3;
  double t9 = std::cos(t8);
  double t10 = std::sin(t8);
  T_legBaseToFoot(0, 0) = t9;
  T_legBaseToFoot(0, 2) = t10;
  T_legBaseToFoot(0, 3) = -l1_ * t6 - l2_ * t10;
  T_legBaseToFoot(1, 0) = t5 * t10;
  T_legBaseToFoot(1, 1) = t2;
  T_legBaseToFoot(1, 2) = -t5 * t9;
  T_legBaseToFoot(1, 3) = sideSign * l0_ * t2 + l1_ * t3 * t5 + l2_ * t3 * t4 * t5 - l2_ * t5 * t6 * t7;
  T_legBaseToFoot(2, 0) = -t2 * t10;
  T_legBaseToFoot(2, 1) = t5;
  T_legBaseToFoot(2, 2) = t2 * t9;
  T_legBaseToFoot(2, 3) = sideSign * l0_ * t5 - l1_ * t2 * t3 - l2_ * t2 * t3 * t4 + l2_ * t2 * t6 * t7;
  T_legBaseToFoot(3, 3) = 1.0;

  foot_pos_legbase = T_legBaseToFoot.block(0, 3, 3, 1);
}

void QuadModelSymbolic::calcFwdKinLegBody(int leg_indx,
                                          const Vector3d &joint_pos,
                                          Matrix4d &T_BodyToFoot,
                                          Vector3d &foot_pos_body) const {
  Matrix4d T_LegBaseToFoot;
  Vector3d foot_pos_legbase;
  calcFwdKinLegBase(leg_indx, joint_pos, T_LegBaseToFoot, foot_pos_legbase);
  T_BodyToFoot = Eigen::Affine3d(T_body_legbases_[leg_indx]) * T_LegBaseToFoot;
  foot_pos_body = Eigen::Affine3d(T_body_legbases_[leg_indx]) * foot_pos_legbase;
}

void QuadModelSymbolic::calcJacobianLegBase(int leg_index, Vector3d joint_pos, Matrix3d &Jac_legBaseToFoot) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  Jac_legBaseToFoot.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q1);
  double t6 = std::sin(q2);
  double t7 = std::sin(q3);
  double t8 = q2 + q3;
  double t9 = std::cos(t8);
  double t10 = l1_ * t6;
  double t11 = std::sin(t8);
  double t12 = l2_ * t9;
  double t13 = l2_ * t11;
  double t14 = -t12;
  double t15 = t10 + t13;
  Jac_legBaseToFoot(0, 1) = t14 - l1_ * t3;
  Jac_legBaseToFoot(0, 2) = t14;
  Jac_legBaseToFoot(1, 0) = -1.0 * sideSign * l0_ * t5 + l1_ * t2 * t3 + l2_ * t2 * t3 * t4 - l2_ * t2 * t6 * t7;
  Jac_legBaseToFoot(1, 1) = -t5 * t15;
  Jac_legBaseToFoot(1, 2) = -t5 * t13;
  Jac_legBaseToFoot(2, 0) = sideSign * l0_ * t2 + l1_ * t3 * t5 + l2_ * t3 * t4 * t5 - l2_ * t5 * t6 * t7;
  Jac_legBaseToFoot(2, 1) = t2 * t15;
  Jac_legBaseToFoot(2, 2) = t2 * t13;
}

void QuadModelSymbolic::calcJacobianDotLegBase(int leg_index,
                                               Vector3d joint_pos,
                                               Vector3d joint_vel,
                                               Matrix3d &JacDot_legBaseToFoot) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  JacDot_legBaseToFoot.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double dq1 = joint_vel[0], dq2 = joint_vel[1], dq3 = joint_vel[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q1);
  double t6 = std::sin(q2);
  double t7 = std::sin(q3);
  double t8 = dq2 + dq3;
  double t9 = q2 + q3;
  double t10 = dq1 * t2;
  double t11 = std::cos(t9);
  double t12 = dq1 * t5;
  double t13 = std::sin(t9);
  double t14 = t11 * t11;
  double t15 = t13 * t13;
  double t16 = -t12;
  double t17 = dq1 * t14;
  double t18 = dq1 * t15;
  double t19 = t17 + t18;

  JacDot_legBaseToFoot(0, 1) = dq2 * l1_ * t6 + dq2 * l2_ * t13 + dq3 * l2_ * t13;
  JacDot_legBaseToFoot(0, 2) = l2_ * t8 * t13;
  JacDot_legBaseToFoot(1, 0) = -1.0 * sideSign * l0_ * t10 + l1_ * t3 * t16 - dq2 * l1_ * t2 * t6 + l2_ * t3 * t4 * t16
                               + l2_ * t6 * t7 * t12 - dq2 * l2_ * t2 * t3 * t7 - dq2 * l2_ * t2 * t4 * t6
                               - dq3 * l2_ * t2 * t3 * t7 - dq3 * l2_ * t2 * t4 * t6;
  JacDot_legBaseToFoot(1, 1) = -l1_ * t6 * t10 - dq2 * l1_ * t3 * t5 - l2_ * t3 * t7 * t10 - l2_ * t4 * t6 * t10
                               - dq2 * l2_ * t3 * t4 * t5 - dq3 * l2_ * t3 * t4 * t5 + dq2 * l2_ * t5 * t6 * t7
                               + dq3 * l2_ * t5 * t6 * t7;
  JacDot_legBaseToFoot(1, 2) = -l2_ * (t5 * t8 * t11 + t2 * t13 * t19);
  JacDot_legBaseToFoot(2, 0) = sideSign * l0_ * t16 + l1_ * t3 * t10 - dq2 * l1_ * t5 * t6 + l2_ * t3 * t4 * t10
                               - l2_ * t6 * t7 * t10 - dq2 * l2_ * t3 * t5 * t7 - dq2 * l2_ * t4 * t5 * t6
                               - dq3 * l2_ * t3 * t5 * t7 - dq3 * l2_ * t4 * t5 * t6;
  JacDot_legBaseToFoot(2, 1) = l1_ * t6 * t16 + dq2 * l1_ * t2 * t3 + l2_ * t3 * t7 * t16 + l2_ * t4 * t6 * t16
                               + dq2 * l2_ * t2 * t3 * t4 + dq3 * l2_ * t2 * t3 * t4 - dq2 * l2_ * t2 * t6 * t7
                               - dq3 * l2_ * t2 * t6 * t7;
  JacDot_legBaseToFoot(2, 2) = l2_ * (t2 * t8 * t11 - t5 * t13 * t19);
}

void QuadModelSymbolic::calcGeneralizedMassInetiaMatrix(int leg_index,
                                                        const Vector3d &joint_pos,
                                                        Matrix3d &MassMatrix_legBase) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  if (sideSign == 1) {
    calcLeftLegGeneralizedMassInetiaMatrix(joint_pos, MassMatrix_legBase);
  } else if (sideSign == -1) {
    calcRightLegGeneralizedMassInetiaMatrix(joint_pos, MassMatrix_legBase);
  } else {
    std::cerr << "Invalid leg ID" << std::endl;
  }
}
void QuadModelSymbolic::calcLeftLegGeneralizedMassInetiaMatrix(Vector3d joint_pos, Matrix3d &MassMatrix_legBase) const {
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  MassMatrix_legBase.setZero();
  double t2 = std::cos(q2);
  double t3 = std::cos(q3);
  double t4 = std::sin(q2);
  double t5 = std::sin(q3);
  double t6 = q2 + q3;
  double t7 = l0_ * l0_;
  double t8 = l1_ * l1_;
  double t9 = q2 * 2.0;
  double t10 = q3 * 2.0;
  double t11 = std::cos(t9);
  double t12 = I2xy * t2;
  double t13 = I2yz * t4;
  double t14 = std::cos(t6);
  double t15 = std::sin(t6);
  double t16 = q2 + t6;
  double t19 = c2x * l0_ * m2 * t2;
  double t20 = c3z * l1_ * m3 * t3;
  double t21 = c3x * l1_ * m3 * t5;
  double t22 = c3y * l1_ * m3 * t4;
  double t23 = c2z * l0_ * m2 * t4;
  double t24 = l0_ * l1_ * m3 * t4;
  double t25 = t6 * 2.0;
  double t17 = I3xy * t14;
  double t18 = I3yz * t15;
  double t26 = c3x * l0_ * m3 * t14;
  double t27 = c3z * l0_ * m3 * t15;
  double t28 = std::cos(t25);
  double t29 = -t19;
  double t30 = -t20;
  double t31 = -t23;
  double t32 = -t26;
  double t33 = -t27;
  double t34 = I3yy + t21 + t30;
  double t35 = t17 + t18 + t32 + t33;
  double t36 = t12 + t13 + t22 + t24 + t29 + t31 + t35;
  MassMatrix_legBase(0, 0) = I1xx + I2xx / 2.0 + I3xx / 2.0 + I2zz / 2.0 + I3zz / 2.0 + t21 + t30 + (I2xx * t11) / 2.0
                             + (I3xx * t28) / 2.0 - (I2zz * t11) / 2.0 - (I3zz * t28) / 2.0 + m2 * t7 + m3 * t7
                             + (m3 * t8) / 2.0 + I2xz * sin(t9) + I3xz * sin(t25) + c2y * l0_ * m2 * 2.0
                             + c3y * l0_ * m3 * 2.0 + (m3 * t8 * t11) / 2.0 - c3z * l1_ * m3 * cos(t16)
                             + c3x * l1_ * m3 * sin(t16);
  MassMatrix_legBase(0, 1) = t36;
  MassMatrix_legBase(0, 2) = t35;
  MassMatrix_legBase(1, 0) = t36;
  MassMatrix_legBase(1, 1) = I2yy + I3yy - t20 * 2.0 + t21 * 2.0 + m3 * t8;
  MassMatrix_legBase(1, 2) = t34;
  MassMatrix_legBase(2, 0) = t35;
  MassMatrix_legBase(2, 1) = t34;
  MassMatrix_legBase(2, 2) = I3yy;
}
void QuadModelSymbolic::calcRightLegGeneralizedMassInetiaMatrix(Vector3d joint_pos,
                                                                Matrix3d &MassMatrix_legBase) const {
  int sideSign = -1;
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  MassMatrix_legBase.setZero();
  double t2 = std::cos(q2);
  double t3 = std::cos(q3);
  double t4 = std::sin(q2);
  double t5 = std::sin(q3);
  double t6 = q2 + q3;
  double t7 = l0_ * l0_;
  double t8 = l1_ * l1_;
  double t9 = q2 * 2.0;
  double t10 = q3 * 2.0;
  double t11 = std::cos(t9);
  double t12 = I2xy * t2;
  double t13 = I2yz * t4;
  double t14 = std::cos(t6);
  double t15 = std::sin(t6);
  double t16 = q2 + t6;
  double t19 = c2x * l0_ * m2 * t2;
  double t20 = c3z * l1_ * m3 * t3;
  double t21 = c3x * l1_ * m3 * t5;
  double t22 = c3y * l1_ * m3 * t4;
  double t23 = c2z * l0_ * m2 * t4;
  double t24 = l0_ * l1_ * m3 * t4;
  double t25 = t6 * 2.0;
  double t17 = I3xy * t14;
  double t18 = I3yz * t15;
  double t26 = c3x * l0_ * m3 * t14;
  double t27 = c3z * l0_ * m3 * t15;
  double t28 = std::cos(t25);
  double t29 = -t20;
  double t30 = -t24;
  double t31 = I3yy + t21 + t29;
  double t32 = t17 + t18 + t26 + t27;
  double t33 = t12 + t13 + t19 + t22 + t23 + t30 + t32;
  MassMatrix_legBase(0, 0) = I1xx + I2xx / 2.0 + I3xx / 2.0 + I2zz / 2.0 + I3zz / 2.0 + t21 + t29 + (I2xx * t11) / 2.0
                             + (I3xx * t28) / 2.0 - (I2zz * t11) / 2.0 - (I3zz * t28) / 2.0 + m2 * t7 + m3 * t7
                             + (m3 * t8) / 2.0 + I2xz * sin(t9) + I3xz * sin(t25) - sideSign * c2y * l0_ * m2 * 2.0
                             - c3y * l0_ * m3 * 2.0 + (m3 * t8 * t11) / 2.0 - c3z * l1_ * m3 * cos(t16)
                             + c3x * l1_ * m3 * sin(t16);
  // MassMatrix_legBase(0, 0) = ;
  MassMatrix_legBase(0, 1) = t33;
  MassMatrix_legBase(0, 2) = t32;
  MassMatrix_legBase(1, 0) = t33;
  MassMatrix_legBase(1, 1) = I2yy + I3yy - t20 * 2.0 + t21 * 2.0 + m3 * t8;
  MassMatrix_legBase(1, 2) = t31;
  MassMatrix_legBase(2, 0) = t32;
  MassMatrix_legBase(2, 1) = t31;
  MassMatrix_legBase(2, 2) = I3yy;
}
void QuadModelSymbolic::calcGeneralizedCoriolisCentrifugalMatrix(int leg_index,
                                                                 Vector3d joint_pos,
                                                                 Vector3d joint_vel,
                                                                 Matrix3d &CoriolisMatrix_legBase) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  if (sideSign == 1) {
    calcLeftLegGeneralizedCoriolisCentrifugalMatrix(joint_pos, joint_vel, CoriolisMatrix_legBase);
  } else if (sideSign == -1) {
    calcRightLegGeneralizedCoriolisCentrifugalMatrix(joint_pos, joint_vel, CoriolisMatrix_legBase);
  } else {
    std::cerr << "Invalid leg ID" << std::endl;
  }
}

void QuadModelSymbolic::calcLeftLegGeneralizedCoriolisCentrifugalMatrix(Vector3d joint_pos,
                                                                        Vector3d joint_vel,
                                                                        Matrix3d &CoriolisMatrix_legBase) const {
  CoriolisMatrix_legBase.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double dq1 = joint_vel[0], dq2 = joint_vel[1], dq3 = joint_vel[2];
  double t2 = std::cos(q2);
  double t3 = std::cos(q3);
  double t4 = std::sin(q2);
  double t5 = std::sin(q3);
  double t6 = q2 + q3;
  double t7 = l1_ * l1_;
  double t8 = q2 * 2.0;
  double t9 = q3 * 2.0;
  double t10 = std::cos(t8);
  double t11 = std::sin(t8);
  double t12 = std::cos(t6);
  double t13 = std::sin(t6);
  double t14 = q2 + t6;
  double t15 = I2yz * dq2 * t2;
  double t16 = I2xy * dq2 * t4;
  double t19 = t6 * 2.0;
  double t26 = c3x * dq2 * l1_ * m3 * t3;
  double t27 = c3x * dq3 * l1_ * m3 * t3;
  double t28 = c3y * dq1 * l1_ * m3 * t2;
  double t29 = c3y * dq2 * l1_ * m3 * t2;
  double t30 = c2z * dq2 * l0_ * m2 * t2;
  double t31 = dq1 * l0_ * l1_ * m3 * t2;
  double t32 = dq2 * l0_ * l1_ * m3 * t2;
  double t33 = c2x * dq2 * l0_ * m2 * t4;
  double t34 = c3z * dq2 * l1_ * m3 * t5;
  double t35 = c3z * dq3 * l1_ * m3 * t5;
  double t17 = std::cos(t14);
  double t18 = std::sin(t14);
  double t20 = I3yz * dq1 * t12;
  double t21 = I3yz * dq2 * t12;
  double t22 = I3yz * dq3 * t12;
  double t23 = I3xy * dq1 * t13;
  double t24 = I3xy * dq2 * t13;
  double t25 = I3xy * dq3 * t13;
  double t36 = -t16;
  double t37 = std::cos(t19);
  double t38 = std::sin(t19);
  double t39 = c3z * dq1 * l0_ * m3 * t12;
  double t40 = c3z * dq2 * l0_ * m3 * t12;
  double t41 = c3z * dq3 * l0_ * m3 * t12;
  double t43 = c3x * dq1 * l0_ * m3 * t13;
  double t44 = c3x * dq2 * l0_ * m3 * t13;
  double t45 = c3x * dq3 * l0_ * m3 * t13;
  double t48 = -t26;
  double t49 = -t28;
  double t50 = -t30;
  double t51 = -t31;
  double t52 = -t34;
  double t42 = -t20;
  double t46 = -t24;
  double t47 = -t25;
  double t53 = I3xz * dq1 * t37;
  double t54 = -t40;
  double t55 = -t41;
  double t56 = -t43;
  double t58 = (I3xx * dq1 * t38) / 2.0;
  double t59 = (I3zz * dq1 * t38) / 2.0;
  double t57 = -t53;
  double t60 = -t59;
  CoriolisMatrix_legBase(0, 0) = t27 + t35 - I2xx * dq2 * t11 + I2xz * dq2 * t10 * 2.0 - I3xx * dq2 * t38
                                 - I3xx * dq3 * t38 + I3xz * dq2 * t37 * 2.0 + I3xz * dq3 * t37 * 2.0 + I2zz * dq2 * t11
                                 + I3zz * dq2 * t38 + I3zz * dq3 * t38 - dq2 * m3 * t7 * t11
                                 + c3x * dq2 * l1_ * m3 * t17 * 2.0 + c3x * dq3 * l1_ * m3 * t17
                                 + c3z * dq2 * l1_ * m3 * t18 * 2.0 + c3z * dq3 * l1_ * m3 * t18;
  CoriolisMatrix_legBase(0, 1) = t15 + t21 + t22 + t29 + t32 + t33 + t36 + t44 + t45 + t46 + t47 + t50 + t54 + t55;
  CoriolisMatrix_legBase(0, 2) = -(dq2 + dq3) * (I3xy * t13 - I3yz * t12 - c3x * l0_ * m3 * t13 + c3z * l0_ * m3 * t12);
  CoriolisMatrix_legBase(1, 0) = t15 + t21 + t22 + t29 + t32 + t33 + t36 + t44 + t45 + t46 + t47 + t50 + t54 + t55 + t57
                                 + t58 + t60 + (I2xx * dq1 * t11) / 2.0 - I2xz * dq1 * t10 - (I2zz * dq1 * t11) / 2.0
                                 + (dq1 * m3 * t7 * t11) / 2.0 - c3x * dq1 * l1_ * m3 * t17
                                 - c3z * dq1 * l1_ * m3 * t18;
  CoriolisMatrix_legBase(1, 1) = t23 + t27 * 2.0 + t35 * 2.0 + t39 + t42 + t49 + t51 + t56 + I2xy * dq1 * t4
                                 - I2yz * dq1 * t2 - c2x * dq1 * l0_ * m2 * t4 + c2z * dq1 * l0_ * m2 * t2;
  CoriolisMatrix_legBase(1, 2) = t23 + t27 + t35 + t39 + t42 + t56;
  CoriolisMatrix_legBase(2, 0) = t21 + t22 + t29 + t32 + t44 + t45 + t46 + t47 + t54 + t55 + t57 + t58 + t60
                                 - (c3x * dq1 * l1_ * m3 * t3) / 2.0 - (c3x * dq1 * l1_ * m3 * t17) / 2.0
                                 - (c3z * dq1 * l1_ * m3 * t5) / 2.0 - (c3z * dq1 * l1_ * m3 * t18) / 2.0;
  CoriolisMatrix_legBase(2, 1) = t23 + t27 + t35 + t39 + t42 + t48 + t49 + t51 + t52 + t56;
  CoriolisMatrix_legBase(2, 2) = t23 + t39 + t42 + t48 + t52 + t56;
}

void QuadModelSymbolic::calcRightLegGeneralizedCoriolisCentrifugalMatrix(Vector3d joint_pos,
                                                                         Vector3d joint_vel,
                                                                         Matrix3d &CoriolisMatrix_legBase) const {
  int sideSign = -1;
  CoriolisMatrix_legBase.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double dq1 = joint_vel[0], dq2 = joint_vel[1], dq3 = joint_vel[2];
  double t2 = std::cos(q2);
  double t3 = std::cos(q3);
  double t4 = std::sin(q2);
  double t5 = std::sin(q3);
  double t6 = q2 + q3;
  double t7 = l1_ * l1_;
  double t8 = q2 * 2.0;
  double t9 = q3 * 2.0;
  double t10 = std::cos(t8);
  double t11 = std::sin(t8);
  double t12 = std::cos(t6);
  double t13 = std::sin(t6);
  double t14 = q2 + t6;
  double t15 = I2yz * dq2 * t2;
  double t16 = I2xy * dq2 * t4;
  double t19 = t6 * 2.0;
  double t26 = c3x * dq2 * l1_ * m3 * t3;
  double t27 = c3x * dq3 * l1_ * m3 * t3;
  double t28 = c3y * dq1 * l1_ * m3 * t2;
  double t29 = c3y * dq2 * l1_ * m3 * t2;
  double t30 = c2z * dq2 * l0_ * m2 * t2;
  double t31 = dq1 * l0_ * l1_ * m3 * t2;
  double t32 = dq2 * l0_ * l1_ * m3 * t2;
  double t33 = c2x * dq2 * l0_ * m2 * t4;
  double t34 = c3z * dq2 * l1_ * m3 * t5;
  double t35 = c3z * dq3 * l1_ * m3 * t5;
  double t17 = std::cos(t14);
  double t18 = std::sin(t14);
  double t20 = I3yz * dq1 * t12;
  double t21 = I3yz * dq2 * t12;
  double t22 = I3yz * dq3 * t12;
  double t23 = I3xy * dq1 * t13;
  double t24 = I3xy * dq2 * t13;
  double t25 = I3xy * dq3 * t13;
  double t36 = -t16;
  double t37 = std::cos(t19);
  double t38 = std::sin(t19);
  double t39 = c3z * dq1 * l0_ * m3 * t12;
  double t40 = c3z * dq2 * l0_ * m3 * t12;
  double t41 = c3z * dq3 * l0_ * m3 * t12;
  double t43 = c3x * dq1 * l0_ * m3 * t13;
  double t44 = c3x * dq2 * l0_ * m3 * t13;
  double t45 = c3x * dq3 * l0_ * m3 * t13;
  double t48 = -t26;
  double t49 = -t28;
  double t50 = -t32;
  double t51 = -t33;
  double t52 = -t34;
  double t42 = -t20;
  double t46 = -t24;
  double t47 = -t25;
  double t53 = I3xz * dq1 * t37;
  double t54 = -t39;
  double t55 = -t44;
  double t56 = -t45;
  double t58 = (I3xx * dq1 * t38) / 2.0;
  double t59 = (I3zz * dq1 * t38) / 2.0;
  double t57 = -t53;
  double t60 = -t59;
  CoriolisMatrix_legBase(0, 0) = t27 + t35 - I2xx * dq2 * t11 + I2xz * dq2 * t10 * 2.0 - I3xx * dq2 * t38
                                 - I3xx * dq3 * t38 + I3xz * dq2 * t37 * 2.0 + I3xz * dq3 * t37 * 2.0 + I2zz * dq2 * t11
                                 + I3zz * dq2 * t38 + I3zz * dq3 * t38 - dq2 * m3 * t7 * t11
                                 + c3x * dq2 * l1_ * m3 * t17 * 2.0 + c3x * dq3 * l1_ * m3 * t17
                                 + c3z * dq2 * l1_ * m3 * t18 * 2.0 + c3z * dq3 * l1_ * m3 * t18;
  CoriolisMatrix_legBase(0, 1) = t15 + t21 + t22 + t29 + t30 + t36 + t40 + t41 + t46 + t47 + t50 + t51 + t55 + t56;
  CoriolisMatrix_legBase(0, 2) = -(dq2 + dq3) * (I3xy * t13 - I3yz * t12 + c3x * l0_ * m3 * t13 - c3z * l0_ * m3 * t12);
  CoriolisMatrix_legBase(1, 0) = t15 + t21 + t22 + t29 + t30 + t36 + t40 + t41 + t46 + t47 + t50 + t51 + t55 + t56 + t57
                                 + t58 + t60 + (I2xx * dq1 * t11) / 2.0 - I2xz * dq1 * t10 - (I2zz * dq1 * t11) / 2.0
                                 + (dq1 * m3 * t7 * t11) / 2.0 - c3x * dq1 * l1_ * m3 * t17
                                 - c3z * dq1 * l1_ * m3 * t18;
  CoriolisMatrix_legBase(1, 1) = t23 + t27 * 2.0 + t31 + t35 * 2.0 + t42 + t43 + t49 + t54 + I2xy * dq1 * t4
                                 - I2yz * dq1 * t2 + c2x * dq1 * l0_ * m2 * t4 - c2z * dq1 * l0_ * m2 * t2;
  CoriolisMatrix_legBase(1, 2) = t23 + t27 + t35 + t42 + t43 + t54;
  CoriolisMatrix_legBase(2, 0) = t21 + t22 + t29 + t40 + t41 + t46 + t47 + t50 + t55 + t56 + t57 + t58 + t60
                                 - (c3x * dq1 * l1_ * m3 * t3) / 2.0 - (c3x * dq1 * l1_ * m3 * t17) / 2.0
                                 - (c3z * dq1 * l1_ * m3 * t5) / 2.0 - (c3z * dq1 * l1_ * m3 * t18) / 2.0;
  CoriolisMatrix_legBase(2, 1) = t23 + t27 + t31 + t35 + t42 + t43 + t48 + t49 + t52 + t54;
  CoriolisMatrix_legBase(2, 2) = t23 + t42 + t43 + t48 + t52 + t54;
}

void QuadModelSymbolic::calcGravityVector(int leg_index, Vector3d joint_pos, Vector3d &GravityVec_legBase) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  GravityVec_legBase.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q1);
  double t6 = std::sin(q2);
  double t7 = std::sin(q3);
  double t8 = q2 + q3;
  double t9 = std::cos(t8);
  double t10 = std::sin(t8);
  GravityVec_legBase(0) = -g
                          * (sideSign * c1y * m1 * t2 + sideSign * c2y * m2 * t2 + c3y * m3 * t2
                             - sideSign * c1z * m1 * t5 + sideSign * l0_ * m2 * t2 + sideSign * l0_ * m3 * t2
                             + c2x * m2 * t5 * t6 - c2z * m2 * t3 * t5 + l1_ * m3 * t3 * t5 + c3x * m3 * t3 * t5 * t7
                             + c3x * m3 * t4 * t5 * t6 - c3z * m3 * t3 * t4 * t5 + c3z * m3 * t5 * t6 * t7);

  // GravityVec_legBase(0) =
  //     -g
  //     * (-c1y * m1 * t2 - c2y * m2 * t2 + c3y * m3 * t2 + c1z * m1 * t5 - l0_ * m2 * t2 - l0_ * m3 * t2
  //        + c2x * m2 * t5 * t6 - c2z * m2 * t3 * t5 + l1_ * m3 * t3 * t5 + c3x * m3 * t3 * t5 * t7
  //        + c3x * m3 * t4 * t5 * t6 - c3z * m3 * t3 * t4 * t5 + c3z * m3 * t5 * t6 * t7);
  GravityVec_legBase(1) = g * t2 * (c2x * m2 * t3 + c3x * m3 * t9 + c2z * m2 * t6 + c3z * m3 * t10 - l1_ * m3 * t6);
  GravityVec_legBase(2) = g * m3 * t2 * (c3x * t9 + c3z * t10);
}
Translation3d QuadModelSymbolic::GetBodyToBellyBottom(unsigned int leg) const {
  return Eigen::Translation3d(belly_bottom_in_base_frame_ + legbase_offsets_[leg]);
}

void QuadModelSymbolic::computeInverseDynamics(
    int leg_index, Vector3d joint_pos, Vector3d joint_vel, Vector3d joint_acc, Vector3d &torqueVec_legBase) const {
  // int leg_index = get_LegIndex(leg);
  int sideSign = get_sideSignLeg(leg_index);
  if (sideSign == 1) {
    computeLeftLegInverseDynamics(joint_pos, joint_vel, joint_acc, torqueVec_legBase);
  } else if (sideSign == -1) {
    computeRightLegInverseDynamics(joint_pos, joint_vel, joint_acc, torqueVec_legBase);
  } else {
    std::cerr << "Invalid leg ID" << std::endl;
  }
}
void QuadModelSymbolic::computeLeftLegInverseDynamics(Vector3d joint_pos,
                                                      Vector3d joint_vel,
                                                      Vector3d joint_acc,
                                                      Vector3d &torqueVec_legBase) const {
  torqueVec_legBase.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double dq1 = joint_vel[0], dq2 = joint_vel[1], dq3 = joint_vel[2];
  double ddq1 = joint_acc[0], ddq2 = joint_acc[1], ddq3 = joint_acc[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q2);
  double t6 = std::sin(q3);
  double t7 = I3yy * ddq2;
  double t8 = I3yy * ddq3;
  double t9 = q1 + q2;
  double t10 = q2 + q3;
  double t11 = dq1 * dq1;
  double t12 = dq2 * dq2;
  double t13 = dq3 * dq3;
  double t14 = l0_ * l0_;
  double t15 = l1_ * l1_;
  double t16 = q2 * 2.0;
  double t17 = q3 * 2.0;
  double t25 = -q1;
  double t26 = -q2;
  double t18 = std::cos(t16);
  double t19 = std::sin(t16);
  double t20 = std::cos(t9);
  double t21 = std::cos(t10);
  double t22 = q3 + t9;
  double t23 = std::sin(t9);
  double t24 = std::sin(t10);
  double t29 = q2 + t10;
  double t32 = q1 + t26;
  double t33 = t10 * 2.0;
  double t35 = t10 + t25;
  double t27 = std::cos(t22);
  double t28 = std::sin(t22);
  double t30 = std::cos(t29);
  double t31 = std::sin(t29);
  double t34 = std::cos(t32);
  double t36 = std::sin(t32);
  double t37 = I3xy * ddq1 * t21;
  double t38 = I3yz * ddq1 * t24;
  double t39 = std::cos(t33);
  double t40 = std::sin(t33);
  double t41 = std::cos(t35);
  double t42 = std::sin(t35);
  double t43 = c3x * ddq1 * l0_ * m3 * t21;
  double t44 = c3z * ddq1 * l0_ * m3 * t24;
  double t45 = (c2x * g * m2 * t20) / 2.0;
  double t46 = (c2z * g * m2 * t23) / 2.0;
  double t47 = (g * l1_ * m3 * t23) / 2.0;
  double t48 = -t43;
  double t49 = -t44;
  double t50 = -t47;
  double t51 = (c3x * g * m3 * t27) / 2.0;
  double t52 = (c3z * g * m3 * t28) / 2.0;
  double t53 = I3xz * t11 * t39;
  double t54 = (c2x * g * m2 * t34) / 2.0;
  double t55 = (c2z * g * m2 * t36) / 2.0;
  double t56 = (g * l1_ * m3 * t36) / 2.0;
  double t57 = (c3x * g * m3 * t41) / 2.0;
  double t58 = (c3z * g * m3 * t42) / 2.0;
  double t60 = (I3xx * t11 * t40) / 2.0;
  double t61 = (I3zz * t11 * t40) / 2.0;
  double t59 = -t53;
  double t62 = -t61;
  torqueVec_legBase(0) =
      t45 + t46 + t50 + t51 + t52 - t54 + t55 - t56 - t57 - t58 + I1xx * ddq1 + (I2xx * ddq1) / 2.0
      + (I3xx * ddq1) / 2.0 + (I2zz * ddq1) / 2.0 + (I3zz * ddq1) / 2.0 + I2xy * ddq2 * t3 + (I2xx * ddq1 * t18) / 2.0
      + I2xz * ddq1 * t19 + I3xy * ddq2 * t21 + I3xy * ddq3 * t21 + (I3xx * ddq1 * t39) / 2.0 + I3xz * ddq1 * t40
      + I2yz * ddq2 * t5 + I3yz * ddq2 * t24 + I3yz * ddq3 * t24 - (I2zz * ddq1 * t18) / 2.0 - (I3zz * ddq1 * t39) / 2.0
      - I2xy * t5 * t12 - I3xy * t12 * t24 - I3xy * t13 * t24 + I2yz * t3 * t12 + I3yz * t12 * t21 + I3yz * t13 * t21
      + ddq1 * m2 * t14 + ddq1 * m3 * t14 + (ddq1 * m3 * t15) / 2.0 - c1y * g * m1 * t2 - c2y * g * m2 * t2
      - c3y * g * m3 * t2 - g * l0_ * m2 * t2 - g * l0_ * m3 * t2 + (ddq1 * m3 * t15 * t18) / 2.0
      + c1z * g * m1 * sin(q1) - I2xx * dq1 * dq2 * t19 + I2xz * dq1 * dq2 * t18 * 2.0 - I3xy * dq2 * dq3 * t24 * 2.0
      - I3xx * dq1 * dq2 * t40 - I3xx * dq1 * dq3 * t40 + I3xz * dq1 * dq2 * t39 * 2.0 + I3xz * dq1 * dq3 * t39 * 2.0
      + I3yz * dq2 * dq3 * t21 * 2.0 + I2zz * dq1 * dq2 * t19 + I3zz * dq1 * dq2 * t40 + I3zz * dq1 * dq3 * t40
      + c2y * ddq1 * l0_ * m2 * 2.0 + c3y * ddq1 * l0_ * m3 * 2.0 - c2x * ddq2 * l0_ * m2 * t3
      + c3x * ddq1 * l1_ * m3 * t6 - c3x * ddq2 * l0_ * m3 * t21 - c3x * ddq3 * l0_ * m3 * t21
      + c3x * ddq1 * l1_ * m3 * t31 + c3y * ddq2 * l1_ * m3 * t5 - c2z * ddq2 * l0_ * m2 * t5
      - c3z * ddq1 * l1_ * m3 * t4 - c3z * ddq2 * l0_ * m3 * t24 - c3z * ddq3 * l0_ * m3 * t24
      - c3z * ddq1 * l1_ * m3 * t30 + ddq2 * l0_ * l1_ * m3 * t5 - dq1 * dq2 * m3 * t15 * t19
      + c2x * l0_ * m2 * t5 * t12 + c3x * l0_ * m3 * t12 * t24 + c3x * l0_ * m3 * t13 * t24 + c3y * l1_ * m3 * t3 * t12
      - c2z * l0_ * m2 * t3 * t12 - c3z * l0_ * m3 * t12 * t21 - c3z * l0_ * m3 * t13 * t21 + l0_ * l1_ * m3 * t3 * t12
      + c3x * dq1 * dq3 * l1_ * m3 * t4 + c3x * dq2 * dq3 * l0_ * m3 * t24 * 2.0
      + c3x * dq1 * dq2 * l1_ * m3 * t30 * 2.0 + c3x * dq1 * dq3 * l1_ * m3 * t30 + c3z * dq1 * dq3 * l1_ * m3 * t6
      - c3z * dq2 * dq3 * l0_ * m3 * t21 * 2.0 + c3z * dq1 * dq2 * l1_ * m3 * t31 * 2.0
      + c3z * dq1 * dq3 * l1_ * m3 * t31;
  torqueVec_legBase(1) =
      t7 + t8 + t37 + t38 + t45 + t46 + t48 + t49 + t50 + t51 + t52 + t54 - t55 + t56 + t57 + t58 + t59 + t60 + t62
      + I2yy * ddq2 + I2xy * ddq1 * t3 + I2yz * ddq1 * t5 + (I2xx * t11 * t19) / 2.0 - I2xz * t11 * t18
      - (I2zz * t11 * t19) / 2.0 + ddq2 * m3 * t15 + (m3 * t11 * t15 * t19) / 2.0 - c2x * ddq1 * l0_ * m2 * t3
      + c3x * ddq2 * l1_ * m3 * t6 * 2.0 + c3x * ddq3 * l1_ * m3 * t6 + c3y * ddq1 * l1_ * m3 * t5
      - c2z * ddq1 * l0_ * m2 * t5 - c3z * ddq2 * l1_ * m3 * t4 * 2.0 - c3z * ddq3 * l1_ * m3 * t4
      + ddq1 * l0_ * l1_ * m3 * t5 + c3x * l1_ * m3 * t4 * t13 - c3x * l1_ * m3 * t11 * t30 + c3z * l1_ * m3 * t6 * t13
      - c3z * l1_ * m3 * t11 * t31 + c3x * dq2 * dq3 * l1_ * m3 * t4 * 2.0 + c3z * dq2 * dq3 * l1_ * m3 * t6 * 2.0;
  torqueVec_legBase(2) = t7 + t8 + t37 + t38 + t48 + t49 + t51 + t52 + t57 + t58 + t59 + t60 + t62
                         + c3x * ddq2 * l1_ * m3 * t6 - c3z * ddq2 * l1_ * m3 * t4 - (c3x * l1_ * m3 * t4 * t11) / 2.0
                         - c3x * l1_ * m3 * t4 * t12 - (c3x * l1_ * m3 * t11 * t30) / 2.0
                         - (c3z * l1_ * m3 * t6 * t11) / 2.0 - c3z * l1_ * m3 * t6 * t12
                         - (c3z * l1_ * m3 * t11 * t31) / 2.0;
}
void QuadModelSymbolic::computeRightLegInverseDynamics(Vector3d joint_pos,
                                                       Vector3d joint_vel,
                                                       Vector3d joint_acc,
                                                       Vector3d &torqueVec_legBase) const {
  int sideSign = -1;
  torqueVec_legBase.setZero();
  double q1 = joint_pos[0], q2 = joint_pos[1], q3 = joint_pos[2];
  double dq1 = joint_vel[0], dq2 = joint_vel[1], dq3 = joint_vel[2];
  double ddq1 = joint_acc[0], ddq2 = joint_acc[1], ddq3 = joint_acc[2];
  double t2 = std::cos(q1);
  double t3 = std::cos(q2);
  double t4 = std::cos(q3);
  double t5 = std::sin(q2);
  double t6 = std::sin(q3);
  double t7 = I3yy * ddq2;
  double t8 = I3yy * ddq3;
  double t9 = q1 + q2;
  double t10 = q2 + q3;
  double t11 = dq1 * dq1;
  double t12 = dq2 * dq2;
  double t13 = dq3 * dq3;
  double t14 = l0_ * l0_;
  double t15 = l1_ * l1_;
  double t16 = q2 * 2.0;
  double t17 = q3 * 2.0;
  double t25 = -q1;
  double t26 = -q2;
  double t18 = std::cos(t16);
  double t19 = std::sin(t16);
  double t20 = std::cos(t9);
  double t21 = std::cos(t10);
  double t22 = q3 + t9;
  double t23 = sin(t9);
  double t24 = sin(t10);
  double t29 = q2 + t10;
  double t32 = q1 + t26;
  double t33 = t10 * 2.0;
  double t35 = t10 + t25;
  double t27 = std::cos(t22);
  double t28 = std::sin(t22);
  double t30 = std::cos(t29);
  double t31 = std::sin(t29);
  double t34 = std::cos(t32);
  double t36 = std::sin(t32);
  double t37 = I3xy * ddq1 * t21;
  double t38 = I3yz * ddq1 * t24;
  double t39 = std::cos(t33);
  double t40 = std::sin(t33);
  double t41 = std::cos(t35);
  double t42 = std::sin(t35);
  double t43 = c3x * ddq1 * l0_ * m3 * t21;
  double t44 = c3z * ddq1 * l0_ * m3 * t24;
  double t45 = (c2x * g * m2 * t20) / 2.0;
  double t46 = (c2z * g * m2 * t23) / 2.0;
  double t47 = (g * l1_ * m3 * t23) / 2.0;
  double t48 = -t47;
  double t49 = (c3x * g * m3 * t27) / 2.0;
  double t50 = (c3z * g * m3 * t28) / 2.0;
  double t51 = I3xz * t11 * t39;
  double t52 = (c2x * g * m2 * t34) / 2.0;
  double t53 = (c2z * g * m2 * t36) / 2.0;
  double t54 = (g * l1_ * m3 * t36) / 2.0;
  double t55 = (c3x * g * m3 * t41) / 2.0;
  double t56 = (c3z * g * m3 * t42) / 2.0;
  double t58 = (I3xx * t11 * t40) / 2.0;
  double t59 = (I3zz * t11 * t40) / 2.0;
  double t57 = -t51;
  double t60 = -t59;
  torqueVec_legBase(0) =
      t45 + t46 + t48 + t49 + t50 - t52 + t53 - t54 - t55 - t56 + I1xx * ddq1 + (I2xx * ddq1) / 2.0
      + (I3xx * ddq1) / 2.0 + (I2zz * ddq1) / 2.0 + (I3zz * ddq1) / 2.0 + I2xy * ddq2 * t3 + (I2xx * ddq1 * t18) / 2.0
      + I2xz * ddq1 * t19 + I3xy * ddq2 * t21 + I3xy * ddq3 * t21 + (I3xx * ddq1 * t39) / 2.0 + I3xz * ddq1 * t40
      + I2yz * ddq2 * t5 + I3yz * ddq2 * t24 + I3yz * ddq3 * t24 - (I2zz * ddq1 * t18) / 2.0 - (I3zz * ddq1 * t39) / 2.0
      - I2xy * t5 * t12 - I3xy * t12 * t24 - I3xy * t13 * t24 + I2yz * t3 * t12 + I3yz * t12 * t21 + I3yz * t13 * t21
      + ddq1 * m2 * t14 + ddq1 * m3 * t14 + (ddq1 * m3 * t15) / 2.0 - sideSign * c1y * g * m1 * t2
      - sideSign * c2y * g * m2 * t2 - c3y * g * m3 * t2 + g * l0_ * m2 * t2 + g * l0_ * m3 * t2
      + (ddq1 * m3 * t15 * t18) / 2.0 + sideSign * c1z * g * m1 * sin(q1) - I2xx * dq1 * dq2 * t19
      + I2xz * dq1 * dq2 * t18 * 2.0 - I3xy * dq2 * dq3 * t24 * 2.0 - I3xx * dq1 * dq2 * t40 - I3xx * dq1 * dq3 * t40
      + I3xz * dq1 * dq2 * t39 * 2.0 + I3xz * dq1 * dq3 * t39 * 2.0 + I3yz * dq2 * dq3 * t21 * 2.0
      + I2zz * dq1 * dq2 * t19 + I3zz * dq1 * dq2 * t40 + I3zz * dq1 * dq3 * t40
      - sideSign * c2y * ddq1 * l0_ * m2 * 2.0 - c3y * ddq1 * l0_ * m3 * 2.0 + c2x * ddq2 * l0_ * m2 * t3
      + c3x * ddq1 * l1_ * m3 * t6 + c3x * ddq2 * l0_ * m3 * t21 + c3x * ddq3 * l0_ * m3 * t21
      + c3x * ddq1 * l1_ * m3 * t31 + c3y * ddq2 * l1_ * m3 * t5 + c2z * ddq2 * l0_ * m2 * t5
      - c3z * ddq1 * l1_ * m3 * t4 + c3z * ddq2 * l0_ * m3 * t24 + c3z * ddq3 * l0_ * m3 * t24
      - c3z * ddq1 * l1_ * m3 * t30 - ddq2 * l0_ * l1_ * m3 * t5 - dq1 * dq2 * m3 * t15 * t19
      - c2x * l0_ * m2 * t5 * t12 - c3x * l0_ * m3 * t12 * t24 - c3x * l0_ * m3 * t13 * t24 + c3y * l1_ * m3 * t3 * t12
      + c2z * l0_ * m2 * t3 * t12 + c3z * l0_ * m3 * t12 * t21 + c3z * l0_ * m3 * t13 * t21 - l0_ * l1_ * m3 * t3 * t12
      + c3x * dq1 * dq3 * l1_ * m3 * t4 - c3x * dq2 * dq3 * l0_ * m3 * t24 * 2.0
      + c3x * dq1 * dq2 * l1_ * m3 * t30 * 2.0 + c3x * dq1 * dq3 * l1_ * m3 * t30 + c3z * dq1 * dq3 * l1_ * m3 * t6
      + c3z * dq2 * dq3 * l0_ * m3 * t21 * 2.0 + c3z * dq1 * dq2 * l1_ * m3 * t31 * 2.0
      + c3z * dq1 * dq3 * l1_ * m3 * t31;
  torqueVec_legBase(1) =
      t7 + t8 + t37 + t38 + t43 + t44 + t45 + t46 + t48 + t49 + t50 + t52 - t53 + t54 + t55 + t56 + t57 + t58 + t60
      + I2yy * ddq2 + I2xy * ddq1 * t3 + I2yz * ddq1 * t5 + (I2xx * t11 * t19) / 2.0 - I2xz * t11 * t18
      - (I2zz * t11 * t19) / 2.0 + ddq2 * m3 * t15 + (m3 * t11 * t15 * t19) / 2.0 + c2x * ddq1 * l0_ * m2 * t3
      + c3x * ddq2 * l1_ * m3 * t6 * 2.0 + c3x * ddq3 * l1_ * m3 * t6 + c3y * ddq1 * l1_ * m3 * t5
      + c2z * ddq1 * l0_ * m2 * t5 - c3z * ddq2 * l1_ * m3 * t4 * 2.0 - c3z * ddq3 * l1_ * m3 * t4
      - ddq1 * l0_ * l1_ * m3 * t5 + c3x * l1_ * m3 * t4 * t13 - c3x * l1_ * m3 * t11 * t30 + c3z * l1_ * m3 * t6 * t13
      - c3z * l1_ * m3 * t11 * t31 + c3x * dq2 * dq3 * l1_ * m3 * t4 * 2.0 + c3z * dq2 * dq3 * l1_ * m3 * t6 * 2.0;
  torqueVec_legBase(2) = t7 + t8 + t37 + t38 + t43 + t44 + t49 + t50 + t55 + t56 + t57 + t58 + t60
                         + c3x * ddq2 * l1_ * m3 * t6 - c3z * ddq2 * l1_ * m3 * t4 - (c3x * l1_ * m3 * t4 * t11) / 2.0
                         - c3x * l1_ * m3 * t4 * t12 - (c3x * l1_ * m3 * t11 * t30) / 2.0
                         - (c3z * l1_ * m3 * t6 * t11) / 2.0 - c3z * l1_ * m3 * t6 * t12
                         - (c3z * l1_ * m3 * t11 * t31) / 2.0;
}

Eigen::Vector3d QuadModelSymbolic::GetFootPositionInWorld(unsigned int foot_idx, const StateInterface &state) const {
  Eigen::Vector3d pos;
  calcWorldToFootFKWorldFrame(foot_idx,
                              state.GetPositionInWorld(),
                              state.GetOrientationInWorld(),
                              Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[foot_idx].data()),
                              pos);
  return pos;
}

Eigen::Vector3d QuadModelSymbolic::GetFootPositionInWorld(unsigned int foot_idx,
                                                          const Eigen::Vector3d &body_pos,
                                                          const Eigen::Quaterniond &body_orientation,
                                                          const Eigen::Vector3d &joint_positions) const {
  Eigen::Vector3d pos;
  calcWorldToFootFKWorldFrame(foot_idx, body_pos, body_orientation, joint_positions, pos);
  return pos;
}

Eigen::Matrix3d QuadModelSymbolic::GetInertia() const { return getBaseInertia(); }
double QuadModelSymbolic::GetInertia(const int row, const int column) const { return base_inertia_(row, column); }
double QuadModelSymbolic::GetMass() const { return getBaseMass(); }
double QuadModelSymbolic::GetG() const { return g; }
Translation3d QuadModelSymbolic::GetBodyToIMU() const { return Eigen::Translation3d(imu_offset_); }

Translation3d QuadModelSymbolic::GetBodyToCOM() const { return Eigen::Translation3d(com_offset_); }

bool QuadModelSymbolic::IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> &joint_states,
                                    const std::array<double, ModelInterface::N_LEGS> &distance_threshold) const {
  Eigen::Vector3d foot_pos_body;
  for (unsigned int i = 0; i < ModelInterface::N_LEGS; i++) {
    QuadModelSymbolic::calcBodyToFootFKBodyFrame(i, joint_states[i], Eigen::Quaterniond::Identity(), foot_pos_body);
    if ((foot_pos_body(2) - distance_threshold[i]) < belly_bottom_in_base_frame_(2)) {
      return false;
    }
  }
  return true;
}

Translation3d QuadModelSymbolic::GetBodyToBellyBottom() const {
  return Eigen::Translation3d(belly_bottom_in_base_frame_);
}

double QuadModelSymbolic::ComputeKineticEnergy(int leg_index,
                                               const Eigen::Vector3d &joint_pos,
                                               const Eigen::Vector3d &joint_vel) const {
  Matrix3d Mass_matrix;
  calcGeneralizedMassInetiaMatrix(leg_index, joint_pos, Mass_matrix);
  return 0.5 * joint_vel.transpose() * Mass_matrix * joint_vel;
}

double QuadModelSymbolic::ComputeEnergyDerivative(int leg_index,
                                                  const Eigen::Vector3d &joint_vel,
                                                  const Eigen::Vector3d &tau) const {
  (void)leg_index;
  return joint_vel.transpose() * tau;
}

void QuadModelSymbolic::SetInertia(const Eigen::Matrix3d &inertia_tensor) { base_inertia_ = inertia_tensor; }
void QuadModelSymbolic::SetInertia(const int row, const int column, const double value) {
  base_inertia_(row, column) = value;
}
void QuadModelSymbolic::SetMass(double mass) { base_mass_ = mass; }
void QuadModelSymbolic::SetCOM(const Eigen::Vector3d &com) {
  // TODO: implement
  com_offset_ = com;
}

void QuadModelSymbolic::SetCOM(const int idx, const double val) { com_offset_(idx) = val; }

QuadModelSymbolic &QuadModelSymbolic::operator=(const ModelInterface &other) {
  return QuadModelSymbolic::operator=(dynamic_cast<const QuadModelSymbolic &>(other));
}

#ifdef ROBOT_MODEL
QuadModelSymbolic::QuadModelSymbolic()
    :
#if ROBOT_MODEL == GO2
      QuadModelSymbolic(UNITREE_QUAD)
#elif ROBOT_MODEL == ULAB
      QuadModelSymbolic(DFKI_QUAD)
#else
#error WRONG ROBOT_MODEL specified
#endif
{
}
#endif

#include "quad_model.hpp"

#include <Eigen/src/Core/GlobalFunctions.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>

// using namespace drake;
using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

// this is also there in qpmpc TODO: move to commons
double map_angle(double value) {
  double mapped_value = fmod(value, (2 * M_PI));
  if (mapped_value > M_PI) {
    mapped_value -= 2 * M_PI;
  }
  // mapped_value -= M_PI;
  return mapped_value;
}

QuadModel::QuadModel() {}

QuadModel::QuadModel(const QuadModel& other)
    : J_(other.J_),
      M_(other.M_),
      Cv_(other.Cv_),
      g_(other.g_),
      eeposf_(other.eeposf_),
      T_wb_(other.T_wb_),
      legindices_(other.legindices_),
      footframes_(other.footframes_),
      shoulderframes_(other.shoulderframes_),
      baselinkframe_(other.baselinkframe_),
      q_(other.q_),
      qd_(other.qd_),
      tau_(other.tau_),
      isrightleg_(other.isrightleg_),
      l1_(other.l1_),
      l2_(other.l2_),
      l3_(other.l3_),
      T_SB_(other.T_SB_),
      base_inertia_(other.base_inertia_),
      base_mass_(other.base_mass_),
      T_bimu_(other.T_bimu_),
      imu_offset_(other.imu_offset_),
      time_(other.time_),
      urdf_path_(other.urdf_path_) {
  // TODO: what to do with the pointers and drake related stuff?
  std::tie(quad_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder_, 0.0);
  auto parser = Parser(quad_, scene_graph_);
  parser.package_map().PopulateFromRosPackagePath();
  auto quad_model_instance_index_ = parser.AddModels(urdf_path_)[0];
  quad_->Finalize();
  this->update();
}

void QuadModel::loadModel(std::string urdf_path) {
  urdf_path_ = urdf_path;
  // TODO: derive all of these from a config. create a struct to store these
  // in a cleaner way probably drake has a function for some of this
  legindices_["fl"] = 6;
  legindices_["fr"] = 9;
  legindices_["bl"] = 12;
  legindices_["br"] = 15;

  // joint name -> pos in qd. for index in q, add one (due to the quaternion))
  legindices_["fl_abad"] = 6;
  legindices_["fl_shoulder"] = 7;
  legindices_["fl_knee"] = 8;
  legindices_["fr_abad"] = 9;
  legindices_["fr_shoulder"] = 10;
  legindices_["fr_knee"] = 11;
  legindices_["bl_abad"] = 12;
  legindices_["bl_shoulder"] = 13;
  legindices_["bl_knee"] = 14;
  legindices_["br_abad"] = 15;
  legindices_["br_shoulder"] = 16;
  legindices_["br_knee"] = 17;

  footframes_["fr"] = "fr_foot_link";
  footframes_["fl"] = "fl_foot_link";
  footframes_["br"] = "br_foot_link";
  footframes_["bl"] = "bl_foot_link";

  shoulderframes_["fr"] = "fr_HAA_link";
  shoulderframes_["fl"] = "fl_HAA_link";
  shoulderframes_["br"] = "br_HAA_link";
  shoulderframes_["bl"] = "bl_HAA_link";

  // True for right
  isrightleg_["fr"] = true;
  isrightleg_["fl"] = false;
  isrightleg_["br"] = true;
  isrightleg_["bl"] = false;

  // load urdf and set up multibody
  std::tie(quad_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder_, 0.0);
  auto parser = Parser(quad_, scene_graph_);
  parser.package_map().PopulateFromRosPackagePath();
  auto quad_model_instance_index_ = parser.AddModels(urdf_path)[0];

  quad_->Finalize();

  // Set to initial zero configuration and determine relevant lengths for leg
  // IK
  Eigen::Matrix<double, 19, 1> q0 = Eigen::Matrix<double, 19, 1>::Zero();
  q0[0] = 1.0;  // quatenion w
  Eigen::Matrix<double, 18, 1> qd0 = Eigen::Matrix<double, 18, 1>::Zero();
  Eigen::Matrix<double, 12, 1> tau0 = Eigen::Matrix<double, 12, 1>::Zero();
  this->setState(q0, qd0, tau0);
  this->update();

  // get l1, l2, l3 from quad in zero position
  auto T =
      quad_->CalcRelativeTransform(*ctxt_, quad_->GetFrameByName("fl_HAA_link"), quad_->GetFrameByName("fl_KFE_link"));
  l1_ = T.translation()[1];  // y - component of HAA-HFE transform
  T = quad_->CalcRelativeTransform(*ctxt_, quad_->GetFrameByName("fl_HAA_link"), quad_->GetFrameByName("fl_KFE_link"));
  l2_ = -T.translation()[2];  // z - component of HFE-KFE transform
  T = quad_->CalcRelativeTransform(*ctxt_, quad_->GetFrameByName("fl_KFE_link"), quad_->GetFrameByName("fl_contact"));
  l3_ = -T.translation()[2];  // z - component of HFE-KFE transform + length of
                              // foot

  // calc imu offset
  T_bimu_ = quad_->CalcRelativeTransform(*ctxt_, quad_->GetFrameByName("base_link"), quad_->GetFrameByName("link_imu"));

  imu_offset_ = T_bimu_.translation();

  // get mass and inertia of the base link
  const drake::multibody::Body<double>& base_body = quad_->GetBodyByName("base_link");
  std::vector<drake::multibody::BodyIndex> body_idx;
  body_idx.push_back(base_body.index());
  const drake::multibody::SpatialInertia<double> I =
      quad_->CalcSpatialInertia(*ctxt_, quad_->GetFrameByName("base_link"), body_idx);
  base_inertia_ = I.CalcRotationalInertia().ShiftToCenterOfMassInPlace(I.get_mass(), I.get_com()).CopyToFullMatrix3();
  base_mass_ = I.get_mass();

  // Furthermore, get the transform from base link to shoulder.
  for (const std::string& leg : {"fl", "fr", "bl", "br"}) {
    T_SB_[leg] = quad_->CalcRelativeTransform(
        *ctxt_, quad_->GetFrameByName(shoulderframes_[leg]), quad_->GetFrameByName("base_link"));
  }
}

// get indices in generalized coordinates vector can only be called after
// calling the load model function
int QuadModel::getPosIdxfromJointName(std::string joint_name) { return legindices_[joint_name] + 1; }

int QuadModel::getVelIdxfromJointName(std::string joint_name) { return legindices_[joint_name]; }

int QuadModel::getTauIdxfromJointName(std::string joint_name) { return legindices_[joint_name] - 6; }

// TODO: remove all of the below messages and eventually only use
// setFromQuadStateMsg
void QuadModel::setState(const Eigen::Matrix<double, 19, 1>& q,
                         const Eigen::Matrix<double, 18, 1>& qd,
                         const Eigen::Matrix<double, 12, 1>& tau) {
  q_ = q;
  qd_ = qd;
  tau_ = tau;
}

Eigen::Vector3d QuadModel::get_body_position() const { return q_.segment(4, 3); }

Eigen::Quaterniond QuadModel::get_body_orientation() const { return Eigen::Quaterniond(q_(0), q_(1), q_(2), q_(3)); }

Eigen::Quaterniond QuadModel::get_body_yaw_quaternion() const {
  return yaw_quaternion_from_quaternion(get_body_orientation());
}

double QuadModel::get_body_yaw_angle() const { return yaw_from_quaternion(get_body_orientation()); }

Eigen::Vector3d QuadModel::get_body_velocity() const { return qd_.segment(3, 3); }

Eigen::Vector3d QuadModel::get_body_twist() const { return qd_.segment(0, 3); }

// set state from QuadState message.
void QuadModel::setFromQuadStateMsg(const interfaces::msg::QuadState& msg) {
  time_ = msg.header.stamp;
  auto o = msg.pose.pose.orientation;
  auto p = msg.pose.pose.position;
  q_.segment(0, 4) << o.w, o.x, o.y, o.z;
  q_.segment(4, 3) << p.x, p.y, p.z;
  T_wb_.set_rotation(Eigen::Quaterniond(o.w, o.x, o.y, o.z));
  T_wb_.set_translation(Eigen::Vector3d(p.x, p.y, p.z));

  // base link twist:
  auto omega = msg.twist.twist.angular;
  auto v = msg.twist.twist.linear;
  qd_.segment(0, 3) << omega.x, omega.y, omega.z;
  qd_.segment(3, 3) << v.x, v.y, v.z;

  for (unsigned int i = 0; i < msg.joint_state.position.size(); i++) {
    int q_idx = i + 7;
    int qd_idx = i + 6;
    int tau_idx = i;
    q_[q_idx] = msg.joint_state.position[i];
    qd_[qd_idx] = msg.joint_state.velocity[i];
    tau_[tau_idx] = msg.joint_state.effort[i];
  }

  this->update();
}

void QuadModel::setFromJointStateMsg(const interfaces::msg::JointState& msg) {
  for (unsigned int i = 0; i < msg.position.size(); i++) {
    int q_idx = i + 7;
    int qd_idx = i + 6;
    int tau_idx = i;
    q_[q_idx] = msg.position[i];
    qd_[qd_idx] = msg.velocity[i];
    tau_[tau_idx] = msg.effort[i];
  }

  this->update();
}

void QuadModel::update() {
  // create context and update state
  ctxt_ = quad_->CreateDefaultContext();
  quad_->SetPositions(ctxt_.get(), q_);
  quad_->SetVelocities(ctxt_.get(), qd_);

  // update dynamic quantities
  quad_->CalcMassMatrix(*ctxt_, &M_);
  quad_->CalcBiasTerm(*ctxt_, &Cv_);
  g_ = quad_->CalcGravityGeneralizedForces(*ctxt_);
}

void QuadModel::getLegJacobian(const std::string leg, Eigen::Matrix3d& J) {
  // start and end idices
  int startidx = legindices_[leg];

  Eigen::Matrix<double, 3, 19> Jtemp;
  quad_->CalcJacobianTranslationalVelocity(
      *ctxt_,
      drake::multibody::JacobianWrtVariable::kQDot,
      quad_->GetFrameByName(leg + "_contact"),  // frame to which foot is attached to
      Eigen::Vector3d().setZero(),              // this is the position of the foot in the foot frame
      quad_->GetFrameByName("base_link_com"),   // base frame
      quad_->GetFrameByName("base_link_com"),   // the frame in which the input
                                                // velocity/force is expressed.
      &Jtemp);                                  // this returns the "big" J. in the following, we are only
                                                // going to use 3x3 submatrices of this one
  J = Jtemp.block(0,
                  startidx + 1,
                  3,
                  3);  // J_ is 3x19, +1, because this uses quaternions
                       // instead of euler angles?
}

void QuadModel::getLegDynamicQuantities(std::string leg,
                                        Eigen::Matrix<double, 3, 3>& Mj,
                                        Eigen::Matrix<double, 3, 3>& Mt,
                                        Eigen::Vector3d& Jdqd,
                                        Eigen::Vector3d& Cv,
                                        Eigen::Vector3d& g) {
  // start and end idices
  int startidx = legindices_[leg];

  // choose the right blocks
  Eigen::Matrix3d J;
  this->getLegJacobian(leg, J);
  Mj = M_.block(startidx, startidx, 3, 3);  // M_ is 18x18
  Cv = Cv_.block(startidx, 0, 3, 1);        // Cv_ and g are 18x1
  g = g_.block(startidx, 0, 3, 1);

  // task space inertia matrix
  auto Jinv = J.inverse();
  Mt = Jinv.transpose() * Mj * Jinv;  // modern robotics p 318

  // jacobiandot bias terms.
  Jdqd = quad_->CalcBiasTranslationalAcceleration(
      *ctxt_,
      drake::multibody::JacobianWrtVariable::kV,
      quad_->GetFrameByName(leg + "_contact"),  // frame to which foot is attached to
      Eigen::Vector3d().setZero(),              // this is the position of the foot in the foot frame
      quad_->GetFrameByName("base_link_com"),   // base frame
      quad_->GetFrameByName("base_link_com"));  // the frame in which the input
                                                // velocity/force is expressed
}

// ---- Differential Kinematics ----
// TODO: check if this can be removed
void QuadModel::calcLegDiffKinematics(std::string leg,
                                      const Eigen::Vector3d& f_ee_goal,
                                      const Eigen::Vector3d& v_ee_goal,
                                      Eigen::Vector3d& tau_goal,
                                      Eigen::Vector3d& qd_goal,
                                      Eigen::Vector3d& f_ee,
                                      Eigen::Vector3d& v_ee) {
  Eigen::Matrix3d J_leg, J_leg_inv;
  Eigen::Vector3d tau_leg, qd_leg;

  // get relevant segments from state vector
  tau_leg = tau_.block(this->getTauIdxfromJointName(leg), 0, 3, 1);
  qd_leg = qd_.block(this->getVelIdxfromJointName(leg), 0, 3, 1);

  // leg Jacobian
  this->getLegJacobian(leg, J_leg);
  J_leg_inv = J_leg.inverse();  // TODO: make sure this is invertible

  // cartesian to joint mappings
  tau_goal = J_leg.transpose() * f_ee_goal;  // f must be in body frame
  qd_goal = J_leg_inv * v_ee_goal;

  // joint to cartesian mappings
  f_ee = J_leg_inv * tau_leg;
  v_ee = J_leg * qd_leg;
}

// map desired endeffector force and vel to joint qunatities
void QuadModel::calcLegDiffKinematics(std::string leg,
                                      const Eigen::Vector3d& f_ee_goal,
                                      const Eigen::Vector3d& v_ee_goal,
                                      Eigen::Vector3d& tau_goal,
                                      Eigen::Vector3d& qd_goal) {
  // leg jacobian
  Eigen::Matrix3d J_leg, J_leg_inv;
  this->getLegJacobian(leg, J_leg);
  J_leg_inv = J_leg.inverse();  // TODO: make sure this is invertible

  // cartesian to joint mappings
  tau_goal = J_leg.transpose() * f_ee_goal;  // f must be in body frame
  qd_goal = J_leg_inv * v_ee_goal;
}

// get endeffector force and velocity
void QuadModel::calcFootForceVelocity(std::string leg, Eigen::Vector3d& f_ee, Eigen::Vector3d& v_ee) {
  Eigen::Matrix3d J_leg, J_leg_trans_inv;
  Eigen::Vector3d tau_leg, qd_leg;

  // get relevant segments from state vector
  tau_leg = tau_.block(this->getTauIdxfromJointName(leg), 0, 3, 1);
  qd_leg = qd_.block(this->getVelIdxfromJointName(leg), 0, 3, 1);

  // leg Jacobian
  this->getLegJacobian(leg, J_leg);
  J_leg_trans_inv = J_leg.transpose().inverse();  // TODO: make sure this is invertible

  // joint to cartesian mappings
  f_ee = J_leg_trans_inv * tau_leg;
  v_ee = J_leg * qd_leg;
}

// ---- Forward Kinematics  TODO: check if its safe to change base_link to
// base_link_com----- get foot position in body coordinates for a given leg
void QuadModel::calcLegForwardKinematics(std::string leg, Eigen::Vector3d& p) {
  auto& framea = quad_->GetFrameByName("base_link_com");
  auto& frameb = quad_->GetFrameByName(leg + "_contact");
  p = quad_->CalcRelativeTransform(*ctxt_, framea, frameb).translation();
}

// needed by mpc
void QuadModel::calcBaseToFootVectInWorld(std::string leg, Eigen::Vector3d& p) {
  auto& framea = quad_->GetFrameByName("base_link_com");
  auto& frameb = quad_->GetFrameByName(leg + "_contact");
  auto rel_tra = quad_->CalcRelativeTransform(*ctxt_, framea, frameb);
  p = T_wb_.rotation() * rel_tra.translation();
}

QuadModel& QuadModel::operator=(const QuadModel& other) {
  J_ = other.J_;
  M_ = other.M_;
  Cv_ = other.Cv_;
  g_ = other.g_;
  eeposf_ = other.eeposf_;
  T_wb_ = other.T_wb_;
  // Skip drake realted things
  legindices_ = other.legindices_;
  footframes_ = other.footframes_;
  shoulderframes_ = other.shoulderframes_;
  baselinkframe_ = other.baselinkframe_;

  q_ = other.q_;
  qd_ = other.qd_;
  tau_ = other.tau_;
  isrightleg_ = other.isrightleg_;
  l1_ = other.l1_;
  l2_ = other.l2_;
  l3_ = other.l3_;
  T_SB_ = other.T_SB_;

  base_inertia_ = other.base_inertia_;
  base_mass_ = other.base_mass_;
  T_bimu_ = other.T_bimu_;
  imu_offset_ = other.imu_offset_;
  time_ = other.time_;

  return *this;
}

// ---- Inverse Kinematics ----
// shoulder IK helper function.
void schulterTransform(
    const double& theta, const double& l1, const bool& right, const Eigen::Vector3d& p_ee, Eigen::Vector3d& p_ee_rot) {
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

// calculate inverse dynamics for a given leg
void QuadModel::calcLegInverseKinematicsInLegFrame(const bool right,
                                                   const Eigen::Vector3d& p_ee,
                                                   Eigen::Vector3d& theta) {
  double x, y, z, l1, l2, l3, gamma, theta1, theta2, theta3, A, alpha1, alpha2, alpha3, xhat, yhat, zhat, beta1, beta2,
      beta3, B;

  // unpack ee position
  x = p_ee[0];
  y = p_ee[1];
  z = p_ee[2];

  gamma = M_PI_2;  // angle of the shoulder
  l1 = l1_;
  l2 = l2_;
  l3 = l3_;

  // calculate tehta1
  A = std::sqrt(x * x + z * z);
  alpha1 = std::atan2(z, y);
  alpha2 = std::asin((l1 / A) * std::sin(gamma));
  alpha3 = M_PI - gamma - alpha2;

  if (right) {
    theta1 = alpha1 - alpha3;
  } else {
    theta1 = alpha1 + alpha3;
  }

  // Rotate coordinate system before calculating theta2 and theta3
  Eigen::Vector3d p_ee_rot;
  schulterTransform(theta1,
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
  beta2 = std::acos((B * B + l2 * l2 - l3 * l3) / (2 * B * l2));
  beta3 = std::acos((l2 * l2 + l3 * l3 - B * B) / (2 * l2 * l3));

  theta2 = beta1 - beta2 + M_PI_2;
  theta3 = M_PI - beta3;

  theta << theta1, theta2, theta3;
}

// IK in body frame
void QuadModel::calcLegInverseKinematics(const std::string leg, const Eigen::Vector3d& p_ee_B, Eigen::Vector3d& theta) {
  // transform from shoulder to body
  Eigen::Vector3d p_ee_S = T_SB_[leg] * p_ee_B;

  // get inverse Kinematics in shoulder frame
  this->calcLegInverseKinematicsInLegFrame(isrightleg_[leg], p_ee_S, theta);

  // fix zero pos and set directions to face in the right direction (TODO:
  // take from conf)
  if (isrightleg_[leg]) {
    theta[0] += M_PI;
  }

  theta[1] = -theta[1];
  theta[2] = -theta[2];

  for (int i = 0; i < 3; i++) {
    theta[i] = map_angle(theta[i]);
  }
}

void QuadModel::getM(Eigen::Matrix<double, 18, 18>& M) { M = M_; }

void QuadModel::getCv(Eigen::Matrix<double, 18, 1>& Cv) { Cv = Cv_; }

void QuadModel::getState(Eigen::Matrix<double, 19, 1>& q, Eigen::Matrix<double, 18, 1>& qd) {
  q = q_;
  qd = qd_;
}

double QuadModel::getBaseMass() { return base_mass_; }

void QuadModel::getBaseInertia(Eigen::Matrix3d& I) { I = base_inertia_; }
Eigen::Matrix3d QuadModel::getBaseInertia() { return base_inertia_; };

Eigen::VectorXd QuadModel::getg() { return g_; }

void QuadModel::getR(Eigen::Matrix3d& R) { R = T_wb_.rotation().matrix(); }

QuadModel::~QuadModel() {}

const rclcpp::Time& QuadModel::getTime() const { return time_; }

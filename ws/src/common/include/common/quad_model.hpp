#pragma once

#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>
#include <map>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "interfaces/msg/quad_state.hpp"
#include "quaternion_operations.hpp"
#include "rclcpp/rclcpp.hpp"

// using namespace drake;
using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

// Quadruped Model
class QuadModel {
 private:
  // Quantities of entire guad
  Eigen::Matrix<double, 3, 19> J_;
  Eigen::Matrix<double, 18, 18> M_;
  Eigen::Matrix<double, 18, 1> Cv_;
  Eigen::Matrix<double, 18, 1> g_;
  Eigen::Matrix<double, 3, 1> eeposf_;
  drake::math::RigidTransform<double> T_wb_;

  // Drake related
  MultibodyPlant<double>* quad_;
  SceneGraph<double>* scene_graph_;
  DiagramBuilder<double> builder_;

  // indexing related TODO make these parameters
  std::unordered_map<std::string, int> legindices_;
  std::unordered_map<std::string, std::string> footframes_;
  std::unordered_map<std::string, std::string> shoulderframes_;
  std::string baselinkframe_;

  // state
  Eigen::Matrix<double, 19, 1> q_;
  Eigen::Matrix<double, 18, 1> qd_;
  Eigen::Matrix<double, 12, 1> tau_;
  std::unique_ptr<drake::systems::Context<double>> ctxt_;

  // IK parameters
  std::unordered_map<std::string, bool> isrightleg_;
  double l1_, l2_, l3_;
  std::map<std::string, drake::math::RigidTransform<double>> T_SB_;

  // mass inertial properties
  Eigen::Matrix3d base_inertia_;
  double base_mass_;

  // imu related
  drake::math::RigidTransform<double> T_bimu_;
  Eigen::Vector3d imu_offset_;

  // time when this model is valid
  rclcpp::Time time_;

  std::string urdf_path_;

 public:
  QuadModel();
  ~QuadModel();

  /**
   * The copy constructor makes sure that the object is copied pointer and thread safe
   * @param other the instance to copy from
   */
  QuadModel(const QuadModel& other);

  QuadModel& operator=(const QuadModel& other);

  // load model
  void loadModel(std::string urdf_path);

  // update function (use setter functions, then call update, to update the
  // drake model and get jacobians etc)
  void update();

  // whole body Quantities
  void getState(Eigen::Matrix<double, 19, 1>& q, Eigen::Matrix<double, 18, 1>& qd);

  // States
  Eigen::Vector3d get_body_position() const;
  Eigen::Quaterniond get_body_orientation() const;
  Eigen::Quaterniond get_body_yaw_quaternion() const;
  double get_body_yaw_angle() const;
  Eigen::Vector3d get_body_velocity() const;
  Eigen::Vector3d get_body_twist() const;

  void getM(Eigen::Matrix<double, 18, 18>& M);   // entire system Mass matrix
  void getCv(Eigen::Matrix<double, 18, 1>& Cv);  // entire system Bias terms
  Eigen::VectorXd getg();
  void getR(Eigen::Matrix3d& R);  // rotation matrix from world to body

  // mass inertial properties
  void getBaseInertia(Eigen::Matrix3d& I);
  Eigen::Matrix3d getBaseInertia();
  double getBaseMass();

  // --- Leg Dynamics ---
  void getLegDynamicQuantities(std::string leg,
                               Eigen::Matrix<double, 3, 3>& Mj,
                               Eigen::Matrix<double, 3, 3>& Mt,
                               Eigen::Vector3d& Jdqd,
                               Eigen::Vector3d& Cv,
                               Eigen::Vector3d& g);  // leg dynamic quantities

  // --- Leg Kinematics ---
  void getLegJacobian(const std::string leg, Eigen::Matrix3d& J);
  void getLegJacobian(const std::string leg, const std::string frame, Eigen::Matrix3d& J);

  // FK
  void getTransformStamped(std::string child_frame,
                           Eigen::Vector3d& p,
                           Eigen::Quaternion<double>& q);  // transform wrt body
  // FK in body frame
  void calcLegForwardKinematics(std::string leg, Eigen::Vector3d& p);
  // FK in world frame
  void calcBaseToFootVectInWorld(std::string leg, Eigen::Vector3d& p);

  // IK
  void calcLegInverseKinematicsInLegFrame(const bool right, const Eigen::Vector3d& p_ee, Eigen::Vector3d& theta);
  void calcLegInverseKinematics(std::string leg, const Eigen::Vector3d& p_ee, Eigen::Vector3d& theta);

  // Differential Kinematics & Torque to EE Force mapping
  // the quadrupeds state is set e.g. via SetFromQuadStateMsg, so we do not
  // need to pass q, dotq and tau manually.
  // TODO: eventually remove this one ?
  void calcLegDiffKinematics(std::string leg,
                             const Eigen::Vector3d& f_ee_goal,
                             const Eigen::Vector3d& v_ee_goal,
                             Eigen::Vector3d& tau_goal,
                             Eigen::Vector3d& qd_goal,
                             Eigen::Vector3d& f_ee,
                             Eigen::Vector3d& v_ee);

  // joint torques and angular velocities from ee force and velocity
  void calcLegDiffKinematics(std::string leg,
                             const Eigen::Vector3d& f_ee_goal,
                             const Eigen::Vector3d& v_ee_goal,
                             Eigen::Vector3d& tau_goal,
                             Eigen::Vector3d& qd_goal);

  // just get endeffector force and velocity
  void calcFootForceVelocity(std::string leg, Eigen::Vector3d& f_ee, Eigen::Vector3d& v_ee);

  // setters TODO: right now its easy to confuse joint states q and
  // generalized coordinates q.
  void setFromQuadStateMsg(const interfaces::msg::QuadState& msg);
  void setFromJointStateMsg(const interfaces::msg::JointState& msg);
  void setState(const Eigen::Matrix<double, 19, 1>& q,
                const Eigen::Matrix<double, 18, 1>& qd,
                const Eigen::Matrix<double, 12, 1>& tau);
  void setPose(Eigen::Quaternion<double>& q, Eigen::Vector3d& p);
  void setTwist(Eigen::Vector3d& lin, Eigen::Vector3d& rot);
  void setJointStates(Eigen::Matrix<double, 12, 1>& q, Eigen::Matrix<double, 12, 1>& qd);

  // imu related
  Eigen::Vector3d getImuOffset() { return imu_offset_; }

  drake::math::RigidTransform<double> getImuTransform() { return T_bimu_; }

  // helper functions for indexing
  int getPosIdxfromJointName(std::string joint_name);
  int getVelIdxfromJointName(std::string joint_name);
  int getTauIdxfromJointName(std::string joint_name);

  const rclcpp::Time& getTime() const;
};

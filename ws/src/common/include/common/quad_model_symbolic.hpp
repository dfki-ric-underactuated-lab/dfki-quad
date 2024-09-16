#pragma once

#include <eigen3/Eigen/Dense>
#include <map>
#include <string>

#include "model_interface.hpp"
#include "quad_state.hpp"
#include "quaternion_operations.hpp"
#include "state_interface.hpp"

#define XSTR(x) STR(x)
#define STR(x) #x
#define GO2 1
#define ULAB 2

using namespace Eigen;  // Matrix3d, Vector3d, Matrix 4d, MatrixXd, VectorXd

struct QuadLegJointVar {
  /// Leg: Abad, Hip, Knee
  int leg_index = -1;
  Vector3d joint_pos_ = Vector3d::Zero();
  Vector3d joint_vel_ = Vector3d::Zero();
  Vector3d joint_acc_ = Vector3d::Zero();
  Vector3d joint_tau_ = Vector3d::Zero();
};

// Quadruped Model
class QuadModelSymbolic : public ModelInterface {
 private:
  /// Number of feet
  static constexpr int num_feet_ = 4;

  /// Vector of all joint names as per quad state message
  std::vector<std::string> joint_names_;

  /// Body dimensions
  double bodyLength_, bodyWidth_, bodyHeight_;

  /// Garvity in negative z direction.
  static constexpr double g = 9.81;  // Direction taken care in the expressions

  /// Leg quantities mass-inertia properties
  /// Abadd
  double m1, c1x, c1y, c1z, I1xx, I1yy, I1zz, I1xy, I1xz, I1yz;
  /// Hip
  double m2, c2x, c2y, c2z, I2xx, I2yy, I2zz, I2xy, I2xz, I2yz;
  /// Knee
  double m3, c3x, c3y, c3z, I3xx, I3yy, I3zz, I3xy, I3xz, I3yz;

  /// l0 -> Abad link length, l1 -> Upper link length, and l2 -> lower link length
  double l0_, l1_, l2_;

  /// Abad offset from legbase
  Vector3d abad_offset_;

  /// Knee offset from hip
  Vector3d knee_offset_;

  /// Foot offset from knee
  Vector3d foot_offset_;

  /// Vector of hip offsets
  std::vector<Vector3d> hip_offsets_;

  /// Vector of legbase offsets
  std::vector<Vector3d> legbase_offsets_;

  /// Vector of legbase offsets
  std::vector<Matrix4d> T_body_legbases_;

  /// Epsilon offset for joint bounds
  static constexpr double joint_eps = 0.1;

  /// Vector of the joint lower limits
  std::vector<std::vector<double>> joint_min_;

  /// Vector of the joint upper limits
  std::vector<std::vector<double>> joint_max_;

  /// Abad max joint torque
  static constexpr double abad_tau_max_ = 16.0;

  /// Hip max joint torque
  static constexpr double hip_tau_max_ = 16.0;

  /// Knee max joint torque
  static constexpr double knee_tau_max_ = 30.0;

  /// Vector of max torques
  VectorXd tau_max_ = (VectorXd(12) << abad_tau_max_,
                       hip_tau_max_,
                       knee_tau_max_,
                       abad_tau_max_,
                       hip_tau_max_,
                       knee_tau_max_,
                       abad_tau_max_,
                       hip_tau_max_,
                       knee_tau_max_,
                       abad_tau_max_,
                       hip_tau_max_,
                       knee_tau_max_)
                          .finished();

  /// Abad max joint velocity
  static constexpr double abad_vel_max_ = 43.63;

  /// Hip max joint velocity
  static constexpr double hip_vel_max_ = 43.63;

  /// Knee max joint velocity
  static constexpr double knee_vel_max_ = 20.0;

  /// Vector of max velocities
  VectorXd vel_max_ = (VectorXd(12) << abad_vel_max_,
                       hip_vel_max_,
                       knee_vel_max_,
                       abad_vel_max_,
                       hip_vel_max_,
                       knee_vel_max_,
                       abad_vel_max_,
                       hip_vel_max_,
                       knee_vel_max_,
                       abad_vel_max_,
                       hip_vel_max_,
                       knee_vel_max_)
                          .finished();

  VectorXd mm_slope_ = tau_max_.cwiseQuotient(vel_max_);

  /// Mass inertial properties for single rigid body dynamics
  Matrix3d base_inertia_;
  double base_mass_;

  Vector3d imu_offset_;
  Vector3d belly_bottom_in_base_frame_;
  Vector3d com_offset_;

  Matrix4d T_base_imu_, T_world_base_;

 public:
  enum RobotModel { DFKI_QUAD = 0, UNITREE_QUAD = 1 };
  QuadModelSymbolic(RobotModel model);

#ifdef ROBOT_MODEL
  QuadModelSymbolic();
#endif

  // IMU offset from world
  Vector3d getIMUOffset() const { return imu_offset_; }
  Matrix4d getImuTransform() const { return T_base_imu_; }
  // mass inertial properties
  Matrix3d getBaseInertia() const { return base_inertia_; }
  double getBaseMass() const { return base_mass_; }
  std::array<double, 3> getLegLinkLengths() const { return {l0_, l1_, l2_}; }

  void SetInertia(const Eigen::Matrix3d& inertia_tensor) override;
  void SetInertia(const int row, const int columns, const double value) override;
  void SetMass(double mass) override;
  void SetCOM(const Eigen::Vector3d& com) override;
  void SetCOM(const int idx, const double val) override;

  // -ve sign for right legs. In the order: fl,fr,bl,br
  static int get_sideSignLeg(int leg) {
    std::vector<int> sideSigns = {1, -1, 1, -1};
    return sideSigns[leg];
  }

  static Vector3d withLegSigns(const Vector3d& v, int legID);

  /**
   * @brief Transform a transformation matrix from the body frame to the world
   * frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] transform_body Specified transform in the body frame
   * @param[out] transform_world Specified transform in the world frame
   */
  static void transformBodyToWorld(const Vector3d& body_pos,
                                   const Quaterniond& body_quat,
                                   const Matrix4d& transform_body,
                                   Matrix4d& transform_world);

  /**
   * @brief Transform a transformation matrix from the world frame to the body
   * frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] transform_world Specified transform in the world frame
   * @param[out] transform_body Specified transform in the body frame
   */
  static void transformWorldToBody(const Vector3d& body_pos,
                                   const Quaterniond& body_quat,
                                   const Matrix4d& transform_world,
                                   Matrix4d& transform_body);
  /**
   * @brief Create an Eigen Matrix4d containing a homogeneous transform
   * from a specified translation and a roll, pitch, and yaw vector
   * @param[in] trans Translation from input frame to output frame
   * @param[in] rpy Rotation from input frame to output frame as roll, pitch,
   * yaw
   * @return Homogenous transformation matrix
   */
  static Matrix4d createTransformationMatrix(const Vector3d& trans, const Quaterniond& quat);

  /**
   * @brief Create an Eigen Matrix4d containing a homogeneous transform
   * from a specified translation and an AngleAxis object
   * @param[in] trans Translation from input frame to output frame
   * @param[in] rot Rotation from input frame to output frame as AngleAxis
   * @return Homogenous transformation matrix
   */
  static Matrix4d createTransformationMatrix(const Vector3d& trans, const AngleAxisd& rot);

  /* Quantities of leg of the quadruped */
  void calcLegInverseKinematicsInLegFrame(int leg_index,
                                          const Eigen::Vector3d& foot_pos_legbase,
                                          const Eigen::Vector3d& joint_state_init_guess,
                                          Eigen::Vector3d& joint_state) const;
  void calcLegInverseKinematicsInBody(int leg_index,
                                      const Eigen::Vector3d& p_ee_B,
                                      const Eigen::Vector3d& joint_state_init_guess,
                                      Eigen::Vector3d& joint_state) const;

  void calcFwdKinLegBase(int leg_index,
                         const Vector3d& joint_pos,
                         Matrix4d& T_legBaseToFoot,
                         Vector3d& foot_pos_legbase) const;

  void calcFwdKinLegBody(int leg_indx,
                         const Vector3d& joint_pos,
                         Matrix4d& T_BodyToFoot,
                         Vector3d& foot_pos_body) const;

  void calcJacobianLegBase(int leg_index, Vector3d joint_pos, Matrix3d& Jac_legBaseToFoot) const;

  void calcJacobianDotLegBase(int leg_index,
                              Vector3d joint_pos,
                              Vector3d joint_vel,
                              Matrix3d& JacDot_legBaseToFoot) const;

  void calcGravityVector(int leg_index, Vector3d joint_pos, Vector3d& GravityVec_legBase) const;

  void calcGeneralizedMassInetiaMatrix(int leg_index, const Vector3d& joint_pos, Matrix3d& MassMatrix_legBase) const;
  void calcLeftLegGeneralizedMassInetiaMatrix(Vector3d joint_pos, Matrix3d& MassMatrix_legBase) const;
  void calcRightLegGeneralizedMassInetiaMatrix(Vector3d joint_pos, Matrix3d& MassMatrix_legBase) const;

  void calcGeneralizedCoriolisCentrifugalMatrix(int leg_index,
                                                Vector3d joint_pos,
                                                Vector3d joint_vel,
                                                Matrix3d& CoriolisMatrix_legBase) const;
  void calcLeftLegGeneralizedCoriolisCentrifugalMatrix(Vector3d joint_pos,
                                                       Vector3d joint_vel,
                                                       Matrix3d& CoriolisMatrix_legBase) const;
  void calcRightLegGeneralizedCoriolisCentrifugalMatrix(Vector3d joint_pos,
                                                        Vector3d joint_vel,
                                                        Matrix3d& CoriolisMatrix_legBase) const;

  void computeInverseDynamics(
      int leg_index, Vector3d joint_pos, Vector3d joint_vel, Vector3d joint_acc, Vector3d& torqueVec_legBase) const;
  void computeLeftLegInverseDynamics(Vector3d joint_pos,
                                     Vector3d joint_vel,
                                     Vector3d joint_acc,
                                     Vector3d& torqueVec_legBase) const;
  void computeRightLegInverseDynamics(Vector3d joint_pos,
                                      Vector3d joint_vel,
                                      Vector3d joint_acc,
                                      Vector3d& torqueVec_legBase) const;

  void calcWorldToLegbaseFKWorldFrame(int leg_index,
                                      const Vector3d& body_pos,
                                      const Quaterniond& body_quat,
                                      Matrix4d& T_world_legbase,
                                      Vector3d& leg_base_pos_world) const;

  void calcBodyToFootFKBodyFrame(int leg_index,
                                 const Vector3d& joint_state,
                                 const Quaterniond& body_quat,
                                 Vector3d& foot_pos_body) const;

  void calcWorldToFootFKWorldFrame(int leg_index,
                                   const Vector3d& body_pos,
                                   const Quaterniond& body_quat,
                                   const Vector3d& joint_state,
                                   Vector3d& foot_pos_world) const;

  void calcWorldToFootIKWorldFrame(int leg_index,
                                   const Vector3d& body_pos,
                                   const Quaterniond& body_quat,
                                   const Vector3d& foot_pos_world,
                                   const Vector3d& joint_state_initial_guess,
                                   Vector3d& joint_state) const;

  void calcBodyToFootIKBodyFrame(int leg_index,
                                 const Quaterniond& body_quat,
                                 const Vector3d& foot_pos_body,
                                 const Vector3d& joint_state_initial_guess,
                                 Vector3d& joint_state) const;

  void calcFootForceVelocityBodyFrame(int leg_index, const StateInterface& state, Vector3d& f_ee, Vector3d& v_ee) const;

  // void CalcFootForceVelocityInBodyFrame(
  //     int leg_index,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_positions,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_velocities,
  //     const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_torques,
  //     Eigen::Ref<Eigen::Vector3d> f_ee,
  //     Eigen::Ref<Eigen::Vector3d> v_ee) const override;

  void CalcFootForceVelocityInBodyFrame(
      int leg_index,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_positions,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_velocities,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_accelerations,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_torques,
      Eigen::Ref<Eigen::Vector3d> f_ee,
      Eigen::Ref<Eigen::Vector3d> v_ee) const override;

  void calcLegDiffKinematicsBodyFrame(int leg_index,
                                      const StateInterface& state,
                                      Vector3d& f_ee_goal,
                                      Vector3d& v_ee_goal,
                                      Vector3d& tau_goal,
                                      Vector3d& qd_goal) const;

  virtual void CalcBaseHeight(const std::array<bool, 4>& feet_contacts,
                              const std::array<const Eigen::Vector3d, 4>& joint_states,
                              const Eigen::Quaterniond& body_orientation,
                              double& base_height) const override;

  virtual Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx,
                                                 const Eigen::Vector3d& body_pos,
                                                 const Eigen::Quaterniond& body_orientation,
                                                 const Eigen::Vector3d& joint_positions) const override;

  virtual Eigen::Vector3d GetFootPositionInWorld(unsigned int foot_idx, const StateInterface& state) const override;

  virtual Eigen::Vector3d GetFootPositionInBodyFrame(unsigned int foot_idx,
                                                     const Eigen::Vector3d& joint_positions) const override;

  virtual Eigen::Matrix3d GetInertia() const override;
  virtual double GetInertia(int row, int column) const override;
  virtual double GetMass() const override;
  virtual double GetG() const override;
  static inline double map_angle(const double& value) {
    double mapped_value = fmod(value, (2 * M_PI));
    if (mapped_value > M_PI) {
      mapped_value -= 2 * M_PI;
    }
    return mapped_value;
  }
  Translation3d GetBodyToIMU() const override;
  Eigen::Translation3d GetBodyToBellyBottom() const override;

  virtual bool IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>& joint_states,
                           const std::array<double, ModelInterface::N_LEGS>& distance_threshold) const override;
  Translation3d GetBodyToBellyBottom(unsigned int leg) const override;

  double ComputeKineticEnergy(int leg_index,
                              const Eigen::Vector3d& joint_pos,
                              const Eigen::Vector3d& joint_vel) const override;
  double ComputeEnergyDerivative(int leg_index,
                                 const Eigen::Vector3d& joint_vel,
                                 const Eigen::Vector3d& tau) const override;
  Translation3d GetBodyToCOM() const override;

  QuadModelSymbolic& operator=(const ModelInterface& other) override;
};

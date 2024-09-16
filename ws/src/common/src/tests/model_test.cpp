#include <drake/geometry/geometry_frame.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <map>
#include <ostream>
#include <quad_model.hpp>
#include <quad_model_symbolic.hpp>
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

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

const std::string URDF_PATH = "./src/common/model/urdf/quad.urdf";

// void Gravity_vector(Eigen::Matrix<double, 3, 1>& gravity_vector, double q1, double q2, double q3) {
//   double sub0_14_0 = std::cos(q1);
//   double sub0_14_1 = std::sin(q2);
//   double sub0_14_2 = std::cos(q2);
//   double sub0_14_3 = q2 + q3;
//   double sub0_14_4 = 0.025086377858665368 * std::sin(sub0_14_3);
//   gravity_vector << 0.50869676571933908 * sub0_14_0
//                         + (-0.0022965054265986252 * sub0_14_1 + 0.26111583862911103 * sub0_14_2
//                            + 0.025086377858665368 * std::cos(sub0_14_3))
//                               * std::sin(q1),
//       sub0_14_0 * (0.26111583862911097 * sub0_14_1 + 0.0022965054265986252 * sub0_14_2 + sub0_14_4),
//       sub0_14_0 * sub0_14_4;
// }

// Eigen::Matrix<double, 3, 1> Gravity_vector(double q1, double q2, double q3) {
//   Eigen::Matrix<double, 3, 1> gravity_vector;
//   Gravity_vector(gravity_vector, q1, q2, q3);
//   return gravity_vector;
// }

// void Gravity_vector(Eigen::Matrix<double, 3, 1>& gravity_vector, const Eigen::Matrix<double, 3, 1>& q) {
//   Gravity_vector(gravity_vector, q[0], q[1], q[2]);
// }

// Eigen::Matrix<double, 3, 1> Gravity_vector(const Eigen::Matrix<double, 3, 1>& q) {
//   return Gravity_vector(q[0], q[1], q[2]);
// }

// void generalized_mass_inertia_matrix(Eigen::Matrix<double, 3, 3>& M, double q2, double q3) {
//   double sub0_12_0 = std::sin(q2);
//   double sub0_12_1 = std::cos(q2);
//   double sub0_12_2 = q2 + q3;
//   double sub0_12_3 = std::sin(sub0_12_2);
//   double sub0_12_4 = std::cos(sub0_12_2);
//   double sub0_12_5 = 0.00022708158550963149 * sub0_12_3;
//   double sub0_12_6 = 0.0020116722276856559 * sub0_12_0 + sub0_12_5;
//   double sub0_12_7 = std::cos(q3);
//   double sub0_12_8 = 0.00038358375930680992 * sub0_12_7;
//   double sub0_12_9 = sub0_12_8 + 0.00023310172625214202;
//   M << 0.0031314194551247398 * std::pow(sub0_12_0, 2.0) + 0.0037775204959576 * std::pow(sub0_12_1, 2.0)
//            + 0.14999999999999999 * sub0_12_1 * (0.014818664672012309 * sub0_12_1 + 0.0025572250620453994 * sub0_12_4)
//            + 0.00077901127434181828 * std::pow(sub0_12_3, 2.0) + 0.00077901127434181828 * std::pow(sub0_12_4, 2.0)
//            + sub0_12_4 * (0.00038358375930680992 * sub0_12_1 + 0.00021687107073270701 * sub0_12_4)
//            + 0.00037984234138564,
//       sub0_12_6, sub0_12_5, sub0_12_6,
//       -0.14999999999999999 * sub0_12_7 * (-0.014818664672012309 * sub0_12_7 - 0.0025572250620453994) + sub0_12_8
//           + 0.0022227997008018464 * std::pow(std::sin(q3), 2.0) + 0.0016799192446349022,
//       sub0_12_9, sub0_12_5, sub0_12_9, 0.00023310172625214199;
// }

// void generalized_mass_inertia_matrix(Eigen::Matrix<double, 3, 3>& M, const Eigen::Matrix<double, 3, 1>& q) {
//   generalized_mass_inertia_matrix(M, q[1], q[2]);
// }

// void Hybrid_jacobian_matrix_ee(Eigen::Matrix<double, 6, 3>& hybrid_jacobian_matrix_ee,
//                                double q1,
//                                double q2,
//                                double q3) {
//   double sub0_11_0 = std::cos(q1);
//   double sub0_11_1 = std::sin(q1);
//   double sub0_11_2 = 0.14999999999999999 * std::cos(q2);
//   double sub0_11_3 = q2 + q3;
//   double sub0_11_4 = 0.13 * std::cos(sub0_11_3);
//   double sub0_11_5 = -sub0_11_4;
//   double sub0_11_6 = sub0_11_2 + sub0_11_4;
//   double sub0_11_7 = 0.14999999999999999 * std::sin(q2);
//   double sub0_11_8 = 0.13 * std::sin(sub0_11_3);
//   hybrid_jacobian_matrix_ee << 1.0, 0.0, 0.0, 0.0, sub0_11_0, sub0_11_0, 0.0, sub0_11_1, sub0_11_1, 0.0,
//       -sub0_11_2 + sub0_11_5, sub0_11_5, sub0_11_0 * sub0_11_6 - 0.088800000000000004 * sub0_11_1,
//       sub0_11_1 * (-sub0_11_7 - sub0_11_8), -sub0_11_1 * sub0_11_8,
//       0.088800000000000004 * sub0_11_0 + sub0_11_1 * sub0_11_6, sub0_11_0 * (sub0_11_7 + sub0_11_8),
//       sub0_11_0 * sub0_11_8;
// }

// Eigen::Matrix<double, 6, 3> Hybrid_jacobian_matrix_ee(double q1, double q2, double q3) {
//   Eigen::Matrix<double, 6, 3> hybrid_jacobian_matrix_ee;
//   Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix_ee, q1, q2, q3);
//   return hybrid_jacobian_matrix_ee;
// }

// void Hybrid_jacobian_matrix_ee(Eigen::Matrix<double, 6, 3>& hybrid_jacobian_matrix_ee,
//                                const Eigen::Matrix<double, 3, 1>& q) {
//   Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix_ee, q[0], q[1], q[2]);
// }

// Eigen::Matrix<double, 6, 3> Hybrid_jacobian_matrix_ee(const Eigen::Matrix<double, 3, 1>& q) {
//   return Hybrid_jacobian_matrix_ee(q[0], q[1], q[2]);
// }

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;
  // drake initializations
  DiagramBuilder<double> builder;
  MultibodyPlant<double>* quad;
  SceneGraph<double>* scene_graph;
  // add quad (multibodyplant) to scene_graph
  std::tie(quad, scene_graph) = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // initialize URDF parser
  auto parser = Parser(quad, scene_graph);
  parser.package_map().PopulateFromRosPackagePath();
  // load from URDF
  auto quadmodel_instance_index = parser.AddModels(URDF_PATH)[0];
  quad->WeldFrames(quad->world_frame(), quad->GetFrameByName("base_link"));
  quad->Finalize();
  std::unique_ptr<drake::systems::Context<double>> context = quad->CreateDefaultContext();

  // drake quad dynamic quantities initialization
  Eigen::Matrix<double, 3, 12> J;   // jacobian
  Eigen::Matrix<double, 12, 12> M;  // mass matrix
  Eigen::Vector<double, 12> Cv;     // coriolis vector
  Eigen::Vector<double, 12> G;      // gravity forces
  Eigen::Vector3d eeposf;
  drake::math::RigidTransform<double> T_wb;  // transformation world to body

  // set zero configuration
  Eigen::Vector<double, 12> q_drake = Eigen::Vector<double, 12>::Zero();
  // q0(0) = 1.0;  // q starts with quaternion for orientation -> q0(0) == quaternion w
  Eigen::Vector<double, 12> qd_drake = Eigen::Vector<double, 12>::Zero();
  Eigen::Vector<double, 12> qdd_drake = Eigen::Vector<double, 12>::Zero();
  Eigen::Vector<double, 12> tau0 = Eigen::Vector<double, 12>::Zero();
  drake::multibody::MultibodyForces external_forces_drake(*quad);
  external_forces_drake.SetZero();
  for (uint i = 0; i < 12; i++) {
    q_drake(i) = 0.3;
    qd_drake(i) = 0.1;
    qdd_drake(i) = 0.05;
  }
  quad->SetPositions(context.get(), q_drake);
  quad->SetVelocities(context.get(), qd_drake);
  quad->CalcMassMatrix(*context, &M);
  quad->CalcBiasTerm(*context, &Cv);
  G = quad->CalcGravityGeneralizedForces(*context);
  quad->CalcBiasTerm(*context, &Cv);
  quad->CalcMassMatrix(*context, &M);
  // create quadmodel symbolic
  QuadModelSymbolic quad_sym;
  Eigen::Vector3d q_sym, qd_sym, qdd_sym, tau_sym;
  q_sym << 0.3, 0.3, 0.3;
  qd_sym << 0.1, 0.1, 0.1;
  qdd_sym << 0.05, 0.05, 0.05;
  tau_sym << 0.2, 0.2, 0.2;
  // ----------Start Tests-----------

  // Check Consistency of Parameters
  // ----------link lengths----------
  std::cout << "---Geometric Quantities---" << std::endl;
  auto T =
      quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_HAA_link"), quad->GetFrameByName("fl_KFE_link"));
  auto leg_lengths_sym = quad_sym.getLegLinkLengths();

  std::cout << "\nl0:" << std::endl;
  std::cout << "Urdf:\t" << T.translation()[1] << std::endl;
  std::cout << "Sym:\t" << leg_lengths_sym[0] << std::endl;
  T = quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_HFE_link"), quad->GetFrameByName("fl_KFE_link"));
  std::cout << "\nl1:" << std::endl;
  std::cout << "Urdf:\t" << -T.translation()[2] << std::endl;
  std::cout << "Sym:\t" << leg_lengths_sym[1] << std::endl;

  T = quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_KFE_link"), quad->GetFrameByName("fl_contact"));

  std::cout << "\nl2:" << std::endl;
  std::cout << "Urdf:\t" << -T.translation()[2] << std::endl;
  std::cout << "Sym:\t" << leg_lengths_sym[2] << std::endl;

  // ----------IMU offset----------
  drake::math::RigidTransform T_bimu =
      quad->CalcRelativeTransform(*context, quad->GetFrameByName("base_link"), quad->GetFrameByName("link_imu"));
  Eigen::Vector3d imu_offset = T_bimu.translation();

  std::cout << "\n---IMU offset:---" << std::endl;
  std::cout << "Urdf:\n" << imu_offset << std::endl;
  std::cout << "Sym:\n" << quad_sym.GetBodyToIMU().vector().array() << std::endl;

  // ----------inertia of base + total mass----------

  std::cout << "\n---Total mass:---" << std::endl;
  std::cout << "Urdf:\t" << quad->CalcTotalMass(*context) << std::endl;
  std::cout << "Sym:\t" << quad_sym.getBaseMass() << std::endl;

  std::vector<drake::multibody::BodyIndex> body_indices = quad->GetBodyIndices(quadmodel_instance_index);
  const drake::multibody::SpatialInertia<double> I =
      quad->CalcSpatialInertia(*context, quad->GetFrameByName("base_link"), body_indices);
  auto base_inertia =
      I.CalcRotationalInertia().ShiftToCenterOfMassInPlace(I.get_mass(), I.get_com()).CopyToFullMatrix3();

  std::cout << "\n---Total inerta---" << std::endl;
  std::cout << "Urdf:\n" << base_inertia << std::endl;
  std::cout << "Sym:\n" << quad_sym.GetInertia() << std::endl;

  // ----------dynamic quantities and kinematics----------
  std::array<std::string, 4> leg_names = {"fl", "fr", "bl", "br"};
  for (int i = 0; i < 4; i++) {
    std::cout << "\n ===== Leg " << i << " =====" << std::endl;
    // -------forward kinematics-------
    std::cout << "\n---Forward Kinematics---" << std::endl;
    T = quad->CalcRelativeTransform(
        *context, quad->GetFrameByName("base_link"), quad->GetFrameByName(leg_names[i] + "_contact"));
    Eigen::Vector3d foot_pos_body;
    foot_pos_body.setZero();
    quad_sym.calcBodyToFootFKBodyFrame(i, q_sym, Eigen::Quaterniond(1, 0, 0, 0), foot_pos_body);
    std::cout << "Fkin of Leg in body frame:" << i << std::endl;
    std::cout << "Drake:\n" << T << std::endl;
    std::cout << "Sym:\n" << foot_pos_body << std::endl;

    // -----inverse kinematics-----
    // ---legframe---
    Eigen::Vector3d foot_pos_leg;
    Eigen::Matrix4d T_foot_leg;
    foot_pos_leg.setZero();
    quad_sym.calcFwdKinLegBase(i, q_sym, T_foot_leg, foot_pos_leg);
    Eigen::Vector3d q_inv_kin_leg;
    // quad_sym.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(0, 0, 0), q_inv_kin);
    std::cout << "\n---Inverse Kinematics---" << std::endl;
    std::cout << "Goal Joint State:\n" << q_sym << std::endl;
    quad_sym.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(-0.06, -0.18, 0.17), q_inv_kin_leg);
    std::cout << "Joint State from Inv Kin leg " << i << " in legbase frame:\n" << q_inv_kin_leg << std::endl;
    std::cout << "Remember the initial guess of the joint angles impacts the result!" << std::endl;
    // ---body frame---
    // foot_pos_body is already calculated in forward kinematics
    Eigen::Vector3d q_inv_kin_body;
    // quad_sym.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(0, 0, 0), q_inv_kin);
    quad_sym.calcLegInverseKinematicsInBody(i, foot_pos_body, Eigen::Vector3d(0, 0, 0), q_inv_kin_body);
    std::cout << "Joint State from Inv Kin leg " << i << " in body frame:\n" << q_inv_kin_body << std::endl;
    std::cout << "Remember the initial guess of the joint angles impacts the result!" << std::endl;
    // -----inverse dynamics-----
    std::cout << "\n---Inverse Dynamics---" << std::endl;
    Eigen::Vector<double, 12> tau_invdyn_drake = quad->CalcInverseDynamics(*context, qdd_drake, external_forces_drake)
                                                 + quad->CalcGravityGeneralizedForces(*context);
    std::cout << "Torque Vector from Inverse Dynamics" << std::endl;
    std::cout << "Drake:\n" << tau_invdyn_drake.segment<3>(i * 3) << std::endl;
    Eigen::Vector3d tau_invdyn_sym;
    quad_sym.computeInverseDynamics(i, q_sym, qd_sym, qdd_sym, tau_invdyn_sym);
    std::cout << "Sym:\n" << tau_invdyn_sym << std::endl;
    //  -----leg jacobians-----
    std::cout << "\n---Leg Jacobians---" << std::endl;
    Eigen::Matrix<double, 3, 12> J_temp;
    quad->CalcJacobianTranslationalVelocity(*context,
                                            drake::multibody::JacobianWrtVariable::kQDot,
                                            quad->GetFrameByName(leg_names[i] + "_contact"),
                                            Eigen::Vector3d(0, 0, 0),
                                            quad->GetFrameByName("base_link"),
                                            quad->GetFrameByName("base_link"),
                                            &J_temp);
    // std::cout << "temp:\n" << J_temp << std::endl;
    Eigen::Matrix3d J_drake;
    J_drake << J_temp.block(0, 3 * i, 3, 3);
    Eigen::Matrix3d J_sym;
    // Eigen::Matrix<double, 6, 3> hybrid_jacobian_matrix;
    quad_sym.calcJacobianLegBase(i, q_sym, J_sym);
    // Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix, Eigen::Vector3d::Zero());

    std::cout << "\nBase to Foot Jacobian of Leg:" << i << std::endl;
    // std::cout << "temp:\n" << J_temp << std::endl;
    std::cout << "Drake:\n" << J_drake << std::endl;
    std::cout << "Sym:\n" << J_sym << std::endl;
    // std::cout << "Skiddy Sym:\n" << hybrid_jacobian_matrix << std::endl;

    // foot force and velocities
    Eigen::Vector3d fee_drake;
    auto J_drake_trans_inv = J_drake.transpose().completeOrthogonalDecomposition().pseudoInverse();
    fee_drake = J_drake_trans_inv * (tau_sym - tau_invdyn_drake.segment<3>(i * 3));
    auto vee_drake = quad->EvalBodySpatialVelocityInWorld(*context, quad->GetBodyByName(leg_names[i] + "_contact"));
    Eigen::Vector3d fee_sym, vee_sym;
    quad_sym.CalcFootForceVelocityInBodyFrame(i, q_sym, qd_sym, qdd_sym, tau_sym, fee_sym, vee_sym);
    std::cout << "\n-----Foot Forces-----" << std::endl;
    std::cout << "\n Foot " << i << " force:" << std::endl;
    std::cout << "Drake:\n" << fee_drake << std::endl;
    std::cout << "Sym:\n" << fee_sym << std::endl;
    std::cout << "\n-----Foot Velocities-----" << std::endl;
    std::cout << "\n Foot " << i << " velocity:" << std::endl;
    std::cout << "Drake:\n" << vee_drake.translational() << std::endl;
    std::cout << "Sym:\n" << vee_sym << std::endl;
    std::cout << "\n---Dynamic Quantities---" << std::endl;
    Eigen::Matrix3d M_drake = M.block(i * 3, i * 3, 3, 3);
    Eigen::Matrix3d M_sym;
    quad_sym.calcGeneralizedMassInetiaMatrix(i, q_sym, M_sym);
    Eigen::Vector3d Cv_drake = Cv.block(i * 3, 0, 3, 1);
    Eigen::Vector3d G_drake = G.block(i * 3, 0, 3, 1);
    Eigen::Vector3d G_sym;
    Eigen::Matrix3d Cv_sym;
    quad_sym.calcGeneralizedCoriolisCentrifugalMatrix(i, q_sym, qd_sym, Cv_sym);
    quad_sym.calcGravityVector(i, q_sym, G_sym);
    std::cout << "\nMass Matrix Leg " << i << " Drake:\n" << M << std::endl;
    std::cout << "\nMass Matrix Leg " << i << " Sym:\n" << M_sym << std::endl;

    std::cout << "\nCoriolis Term Leg " << i << " Drake:\n" << Cv_drake << std::endl;
    std::cout << "\nCoriolis Term Leg " << i << " Sym:\n" << Cv_sym * qd_sym << std::endl;

    std::cout << "\ngravity Term Leg " << i << " Drake:\n" << G_drake << std::endl;
    std::cout << "\ngravity Term Leg " << i << " Sym:\n" << G_sym << std::endl;
  }
  std::cout << "\n!!!BEWARE: THE DRAKE REFERENCE VALUES ARE ONLY VALID FOR THE ULAB QUAD!!!\n"
            << "there is no option to load the go2 model yet." << std::endl;
}

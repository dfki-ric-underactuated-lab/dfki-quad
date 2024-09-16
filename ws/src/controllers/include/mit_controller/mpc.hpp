#pragma once

#include <acados_c/ocp_qp_interface.h>

#include <Eigen/Dense>

#include "common/eigen_util.hpp"
#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "mit_controller_params.hpp"
#include "mpc_interface.hpp"

class MPC : public MPCInterface {
 public:
  static constexpr int STATE_SIZE = 13;
  static constexpr int NUM_FEET = N_LEGS;
  static constexpr int INPUT_SIZE = 3 * NUM_FEET;
  static constexpr int PREDICTION_HORIZON = MPC_PREDICTION_HORIZON;
  static constexpr double POS_UNBOUND = 99999999;
  static constexpr double NEG_UNBOUND = -POS_UNBOUND;
  static constexpr double DeltaT = MPC_DT;
  static constexpr int NUM_POLY_CONST_PER_FOOT = 4;
  static constexpr double GRAVITY_CONSTANT = 9.8067;
  static constexpr double SMALL_NUMBER = 0.00001;

 private:
  std::unique_ptr<ModelInterface> quad_model_;
  bool model_update_;
  std::unique_ptr<StateInterface> quad_state_;
  GaitSequence gait_sequence_;

  // params
  double fmin_;
  double fmax_;
  double mu_;

  // This typedefs automatically enforce the correct storage order to be used as raw pointers for acados
  // (see: https://discourse.acados.org/t/storage-order-of-c-interface/1379/3)
  typedef Eigen::Matrix<double, STATE_SIZE, STATE_SIZE, Eigen::ColMajor> AMatrixT;
  typedef Eigen::Matrix<double, STATE_SIZE, INPUT_SIZE, Eigen::ColMajor> BMatrixT;
  typedef Eigen::Matrix<double, NUM_FEET * NUM_POLY_CONST_PER_FOOT, INPUT_SIZE, Eigen::ColMajor> CMatrixT;
  typedef Eigen::Matrix<double, NUM_FEET * NUM_POLY_CONST_PER_FOOT, 1, Eigen::ColMajor> CBoundT;
  typedef Eigen::Matrix<double, STATE_SIZE, STATE_SIZE, Eigen::ColMajor> QMatrixT;
  typedef Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE, Eigen::ColMajor> RMatrixT;
  typedef Eigen::Matrix<double, STATE_SIZE, 1, Eigen::ColMajor> stateVecT;
  typedef Eigen::Matrix<double, INPUT_SIZE, 1, Eigen::ColMajor> inputVecT;

  // time varying A
  std::array<AMatrixT, PREDICTION_HORIZON> As_;
  // time varying B
  std::array<BMatrixT, PREDICTION_HORIZON> Bs_;
  // fixed zero D_:
  Eigen::Matrix<double, NUM_FEET * NUM_POLY_CONST_PER_FOOT, STATE_SIZE, Eigen::ColMajor> ZeroC_;
  // fixed C;
  CMatrixT D_;
  // fixed Jbu
  RMatrixT Jbu;
  // fixed cost matrices Q and R,
  QMatrixT Q_;
  RMatrixT R_;
  // time variny q vector which contains the target state
  std::array<stateVecT, PREDICTION_HORIZON + 1> qs_;
  // fixed upper and lower bounds of C
  CBoundT Cub_;
  CBoundT Clb_;
  // current state
  stateVecT x0_;
  // input forces bounds
  std::array<inputVecT, PREDICTION_HORIZON> ulbs_;
  std::array<inputVecT, PREDICTION_HORIZON> uubs_;
  // some constants
  stateVecT stateOnes_;
  inputVecT inputOnes_;
  AMatrixT stateIdentity_;
  RMatrixT inputIdentity_;

  /**
   * Contains the solver planned to be used
   */
  ocp_qp_solver_plan_t solver_plan_;
  ocp_qp_dims *ocp_dims_;

  ocp_qp_xcond_solver_config *solver_config_;
  ocp_qp_xcond_solver_dims *solver_dims_;
  void *solver_opts_;
  ocp_qp_solver *qp_solver_;

  ocp_qp_in *qp_in_;
  ocp_qp_out *qp_out_;

  // TODO: try to make them constexpr
  static void GetA(double psi, double dt, bool update_linearization, Eigen::Ref<AMatrixT> A);
  static void GetB(double psi,
                   double m,
                   const Eigen::Ref<const Eigen::Matrix3d> &inertia,
                   const std::array<const Eigen::Ref<const Eigen::Vector3d>, NUM_FEET> &r,
                   const std::array<bool, NUM_FEET> &contacts,
                   double dt,
                   bool update_linearization,
                   Eigen::Ref<BMatrixT> B);
  static void GetR(double psi, double multiply_with, bool update_linearization, Eigen::Ref<Eigen::Matrix3d> R);
  static void GetIWorld(double psi,
                        const Eigen::Ref<const Eigen::Matrix3d> &inertia,
                        Eigen::Ref<Eigen::Matrix3d> IWorld);
  static void GetC(double mu, Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, INPUT_SIZE>> C);
  static void GetCbounds(double fmin,
                         double fmax,
                         Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_lb,
                         Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_ub);
  static void GetUBounds(const std::array<bool, NUM_FEET> &contact,
                         double fmin,
                         double fmax,
                         Eigen::Ref<inputVecT> lb,
                         Eigen::Ref<inputVecT> ub);

 public:
  MPC(double alpha,
      const Eigen::Matrix<double, STATE_SIZE - 1, 1> &state_weights,
      double mu,
      double fmin,
      double fmax,
      std::unique_ptr<StateInterface> quad_state,
      std::unique_ptr<ModelInterface> quad_model,
      ocp_qp_solver_t solver,
      int condensing_N,
      std::string hpipm_mode,
      int warm_start);

  void UpdateState(const StateInterface &quad_state) override;
  void UpdateModel(const ModelInterface &quad_model) override;

  void UpdateGaitSequence(const GaitSequence &gait_sequence) override;

  void GetWrenchSequence(WrenchSequence &wrench_sequence,
                         MPCPrediction &state_prediction,
                         SolverInformation &solver_information) override;

  void SetStateWeights(const Eigen::Matrix<double, STATE_SIZE - 1, 1> &weights);
  void SetInputWeights(double alpha);
  void SetFmax(double fmax);
  void SetMu(double mu);

  ~MPC() override;
};
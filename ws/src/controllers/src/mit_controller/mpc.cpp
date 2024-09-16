#include "mit_controller/mpc.hpp"

#include <iostream>

#include "acados/ocp_qp/ocp_qp_full_condensing.h"
#include "acados/utils/print.h"
#include "acados_c/dense_qp_interface.h"
#include "common/eigen_util.hpp"
#include "common/quaternion_operations.hpp"

void MPC::GetA(double psi, double dt, bool update_linearization, Eigen::Ref<AMatrixT> A) {
  if (!update_linearization) {
    A.setIdentity();
    A.block<3, 3>(3, 9).diagonal().setConstant(dt);
    A(11, 12) = -dt;
  }
  MPC::GetR(psi, dt, update_linearization, A.block<3, 3>(0, 6));
}

void MPC::GetB(double psi,
               double m,
               const Eigen::Ref<const Eigen::Matrix3d> &inertia,
               const std::array<const Eigen::Ref<const Eigen::Vector3d>, NUM_FEET> &r,
               const std::array<bool, NUM_FEET> &contacts,
               double dt,
               bool update_linearization,
               Eigen::Ref<BMatrixT> B) {
  if (!update_linearization) {
    B.setZero();
    for (unsigned int foot_indx = 0; foot_indx < NUM_FEET; foot_indx++) {
      B.block<3, 3>(9, foot_indx * 3).diagonal().setConstant(dt / m);
    }
  }
  Eigen::Matrix3d I_rot;
  MPC::GetIWorld(psi, inertia, I_rot);
  Eigen::Matrix3d I_rot_inv = I_rot.inverse();
  for (unsigned int foot_indx = 0; foot_indx < NUM_FEET; foot_indx++) {
    if (contacts[foot_indx]) {
      B.block<3, 3>(6, foot_indx * 3) = I_rot_inv * skew(r[foot_indx]) * dt;
    } else {
      B.block<3, 3>(6, foot_indx * 3).setZero();
    }
  }
}

void MPC::GetR(double psi, double multiply_with, bool update_linearization, Eigen::Ref<Eigen::Matrix3d> R) {
  if (!update_linearization) {
    R(0, 2) = 0;
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = multiply_with;
  }
  R(0, 0) = cos(psi) * multiply_with;
  R(0, 1) = sin(psi) * multiply_with;
  R(1, 0) = -sin(psi) * multiply_with;
  R(1, 1) = cos(psi) * multiply_with;
}

void MPC::GetIWorld(double psi, const Eigen::Ref<const Eigen::Matrix3d> &inertia, Eigen::Ref<Eigen::Matrix3d> IWorld) {
  static Eigen::Matrix3d R;
  MPC::GetR(psi, 1.0, false, R);
  IWorld.noalias() = R * inertia * R.transpose();
}

void MPC::GetC(double mu, Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, INPUT_SIZE>> C) {
  for (unsigned int foot = 0; foot < NUM_FEET; foot++) {
    C.block<NUM_POLY_CONST_PER_FOOT, 3>(NUM_POLY_CONST_PER_FOOT * foot, 3 * foot) << 1, 0, -mu, 1, 0, mu, 0, 1, -mu, 0,
        1, mu;
  }
}

void MPC::GetCbounds(double fmin,
                     double fmax,
                     Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_lb,
                     Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_ub) {
  (void)fmin;
  (void)fmax;
  for (unsigned int foot = 0; foot < NUM_FEET; foot++) {
    auto block_lb = c_lb.block<NUM_POLY_CONST_PER_FOOT, 1>(foot * NUM_POLY_CONST_PER_FOOT, 0);
    auto block_ub = c_ub.block<NUM_POLY_CONST_PER_FOOT, 1>(foot * NUM_POLY_CONST_PER_FOOT, 0);
    block_lb(0, 0) = NEG_UNBOUND;
    block_lb(1, 0) = 0;
    block_lb(2, 0) = NEG_UNBOUND;
    block_lb(3, 0) = 0;

    block_ub(0, 0) = 0;
    block_ub(1, 0) = POS_UNBOUND;
    block_ub(2, 0) = 0;
    block_ub(3, 0) = POS_UNBOUND;
  }
}

void MPC::GetUBounds(const std::array<bool, NUM_FEET> &contact,
                     double fmin,
                     double fmax,
                     Eigen::Ref<inputVecT> lb,
                     Eigen::Ref<inputVecT> ub) {
  // TODO: maybe dont use this redundant bb constraints
  for (unsigned int feet_idx = 0; feet_idx < NUM_FEET; feet_idx++) {
    auto lb_block = lb.block<3, 1>(feet_idx * 3, 0);
    auto ub_block = ub.block<3, 1>(feet_idx * 3, 0);
    if (contact[feet_idx]) {
      lb_block(0) = -fmax;
      lb_block(1) = -fmax;
      lb_block(2) = fmin;
      ub_block(0) = fmax;
      ub_block(1) = fmax;
      ub_block(2) = fmax;
    } else {
      lb_block.setZero();
      ub_block.setZero();
    }
  }
}

MPC::MPC(double alpha,
         const Eigen::Matrix<double, STATE_SIZE - 1, 1> &state_weights,
         double mu,
         double fmin,
         double fmax,
         std::unique_ptr<StateInterface> quad_state,
         std::unique_ptr<ModelInterface> quad_model,
         ocp_qp_solver_t solver,
         int condensing_N,
         std::string hpipm_mode,
         int warm_start)
    : quad_model_(std::move(quad_model)),
      model_update_(false),
      quad_state_(std::move(quad_state)),
      fmin_(fmin),
      fmax_(fmax),
      mu_(mu),
      stateOnes_(stateVecT::Ones()),
      inputOnes_(inputVecT::Ones()),
      stateIdentity_(AMatrixT::Identity()),
      inputIdentity_(RMatrixT::Identity()) {
  // Initial creation of matrices with placeholder for yaw and foot positions
  Eigen::Vector3d zero_pos;
  zero_pos.setZero();
  ZeroC_.setZero();
  // Time varying -> Very inefficent for now, but thats fine as this is the constructor call
  for (unsigned int k = 0; k < PREDICTION_HORIZON; k++) {
    MPC::GetA(0, DeltaT, false, As_[k]);
    MPC::GetB(0,
              quad_model_->GetMass(),
              quad_model_->GetInertia(),
              {zero_pos, zero_pos, zero_pos, zero_pos},
              {false, false, false, false},
              DeltaT,
              false,
              Bs_[k]);
    // This cheks are important to later pass the pointer to the acados calls
    assert(As_[k].innerStride() == 1);
    assert(As_[k].outerStride() == STATE_SIZE);
    assert(Bs_[k].innerStride() == 1);
    assert(Bs_[k].outerStride() == STATE_SIZE);
  }
  // Constants
  MPC::GetC(mu_, D_);
  MPC::GetCbounds(fmin_, fmax_, Clb_, Cub_);
  Eigen::Matrix<double, STATE_SIZE, 1> state_weights_with_g;
  state_weights_with_g.setConstant(0);
  state_weights_with_g.block<STATE_SIZE - 1, 1>(0, 0) = state_weights;
  Q_ = state_weights_with_g.asDiagonal();
  R_ = RMatrixT::Identity() * alpha;

  assert(R_.innerStride() == 1);
  assert(R_.outerStride() == INPUT_SIZE);
  assert(Q_.innerStride() == 1);
  assert(Q_.outerStride() == STATE_SIZE);
  assert(D_.innerStride() == 1);
  assert(D_.outerStride() == NUM_POLY_CONST_PER_FOOT * NUM_FEET);
  // Solver planned to use
  solver_plan_.qp_solver = solver;
  // Create dimensions of QP
  ocp_dims_ = ocp_qp_dims_create(PREDICTION_HORIZON);
  solver_config_ = ocp_qp_xcond_solver_config_create(solver_plan_);
  int nx = STATE_SIZE;  // This is necessary in order to not lose const qualifiers
  int nu = INPUT_SIZE;  // TODO: do they have to be in object?
  int ncb = NUM_POLY_CONST_PER_FOOT * NUM_FEET;
  int nbu = NUM_FEET * 3;  // Limit all forces
  int zero = 0;
  // inital value constraint for x
  ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nx);
  for (int k = 0; k <= PREDICTION_HORIZON; k++) {
    // State size
    ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
    // Input size
    ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
    // Input constraints
    ocp_qp_dims_set(solver_config_, ocp_dims_, k, "ng", &ncb);
    ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
  }

  // last stage has no controls
  ocp_qp_dims_set(solver_config_, ocp_dims_, PREDICTION_HORIZON, "nu", &zero);
  ocp_qp_dims_set(solver_config_, ocp_dims_, PREDICTION_HORIZON, "ng", &zero);
  ocp_qp_dims_set(solver_config_, ocp_dims_, PREDICTION_HORIZON, "nbu", &zero);
  // create qp in based on the dimensions
  qp_in_ = ocp_qp_in_create(ocp_dims_);
  // set the values that won't be changed
  for (int k = 0; k < PREDICTION_HORIZON; k++) {
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("D"), D_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("C"), ZeroC_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("R"), R_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Q"), Q_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lg"), Clb_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ug"), Cub_.data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Jbu"), inputIdentity_.data());
  }
  ocp_qp_in_set(solver_config_, qp_in_, PREDICTION_HORIZON, const_cast<char *>("Q"), Q_.data());
  ocp_qp_in_set(solver_config_, qp_in_, 0, const_cast<char *>("Jbx"), stateIdentity_.data());

  // create solver
  solver_dims_ = ocp_qp_xcond_solver_dims_create_from_ocp_qp_dims(solver_config_, ocp_dims_);
  solver_opts_ = ocp_qp_xcond_solver_opts_create(solver_config_, solver_dims_);

  if (solver < FULL_CONDENSING_HPIPM) {  // this is the first solver after partials
    assert(condensing_N > 0);
    assert(condensing_N <= MPC_PREDICTION_HORIZON);
    std::cout << "Set condensing to " << condensing_N << std::endl;
    ocp_qp_xcond_solver_opts_set(
        solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "cond_N", &condensing_N);
  }
  if (solver == PARTIAL_CONDENSING_HPIPM || solver == FULL_CONDENSING_HPIPM) {
    assert(hpipm_mode == "SPEED_ABS" or hpipm_mode == "SPEED" or hpipm_mode == "BALANCE" or hpipm_mode == "ROBUST");
    std::cout << "Set hpipm mode to " << hpipm_mode.c_str() << std::endl;
    // char hpipm_mode[] = "SPEED_ABS";
    ocp_qp_xcond_solver_opts_set(solver_config_,
                                 reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_),
                                 "hpipm_mode",
                                 (void *)hpipm_mode.c_str());
  }

  ocp_qp_xcond_solver_opts_set(
      solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "warm_start", &warm_start);

  qp_solver_ = ocp_qp_create(solver_config_, solver_dims_, solver_opts_);

  // create problem out
  qp_out_ = ocp_qp_out_create(ocp_dims_);

  // int print_level = 1;
  // ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts*>(solver_opts_),
  // "qp_print_level", &print_level);

  // ocp_qp_in xcond_in;
  // solver_config_->xcond->memory_get(
  // Print Dimensons

  std::cout << " ---> Original problem dimensions: " << std::endl;
  print_ocp_qp_dims(ocp_dims_);
  std::cout << " <--- " << std::endl;

  if (solver < FULL_CONDENSING_HPIPM
      && condensing_N < MPC_PREDICTION_HORIZON) {  // Only for partial condensed solvers, and if condensed size
                                                   // different from original size:
    std::cout << " ---> Partial condensed problem dimensions: " << std::endl;
    print_ocp_qp_dims(((ocp_qp_in *)((ocp_qp_xcond_solver_memory *)qp_solver_->mem)->xcond_qp_in)->dim);
    std::cout << " <--- " << std::endl;
  } else if (solver >= FULL_CONDENSING_HPIPM) {  // For fully condensed solvers
    auto dense_dim = ((ocp_qp_full_condensing_memory *)qp_solver_->mem)->fcond_qp_in->dim;
    std::cout << " ---> Full condensed problem dimensions: " << std::endl;
    printf("k\tnv\t\tnb\tne\t\tng\tns\n");
    printf("0\t%d\t\t%d\t%d\t\t%d\t%d\t\n", dense_dim->nv, dense_dim->nb, dense_dim->ne, dense_dim->ng, dense_dim->ns);
    std::cout << " <--- " << std::endl;
  }

  //     solver_config_->xcond, ((ocp_qp_xcond_solver_memory *)qp_solver_->mem)->xcond_memory, "xcond_qp_in",
  //     &xcond_in);
}

MPC::~MPC() {
  ocp_qp_xcond_solver_dims_free(solver_dims_);
  ocp_qp_dims_free(ocp_dims_);
  ocp_qp_xcond_solver_config_free(solver_config_);
  ocp_qp_xcond_solver_opts_free(reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_));
  ocp_qp_in_free(qp_in_);
  ocp_qp_out_free(qp_out_);
  ocp_qp_solver_destroy(qp_solver_);
}

void MPC::UpdateState(const StateInterface &quad_state) { *quad_state_ = quad_state; };

void MPC::UpdateModel(const ModelInterface &quad_model) {
  *quad_model_ = quad_model;
  // The B matrix depends on the quad_model, but will anyway be changed with every control step.
  model_update_ = true;
}

void MPC::UpdateGaitSequence(const GaitSequence &gait_sequence) { gait_sequence_ = gait_sequence; }

void MPC::SetStateWeights(const Eigen::Matrix<double, STATE_SIZE - 1, 1> &weights) {
  Q_.diagonal().block<STATE_SIZE - 1, 1>(0, 0) = weights;
  for (int k = 0; k <= PREDICTION_HORIZON; k++) {
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Q"), Q_.data());
  }
  std::cout << "Updated state weights to: " << weights.transpose() << std::endl;
}

void MPC::SetInputWeights(double alpha) {
  R_ = RMatrixT::Identity() * alpha;
  for (int k = 0; k < PREDICTION_HORIZON; k++) {
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("R"), R_.data());
  }
  std::cout << "Updated input weights to: " << alpha << std::endl;
}

void MPC::SetFmax(double fmax) {
  fmax_ = fmax;
  for (int k = 0; k < PREDICTION_HORIZON; k++) {
    MPC::GetUBounds(gait_sequence_.contact_sequence[k], fmin_, fmax_, ulbs_[k], uubs_[k]);
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbu"), ulbs_[k].data());
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubu"), uubs_[k].data());
  }
  std::cout << "Updated fmax to: " << fmax_ << std::endl;
}

void MPC::SetMu(double mu) {
  mu_ = mu;
  MPC::GetC(mu_, D_);
  for (int k = 0; k < PREDICTION_HORIZON; k++) {
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("D"), D_.data());
  }
  std::cout << "Updated mu to: " << mu_ << std::endl;
}

void MPC::GetWrenchSequence(WrenchSequence &wrench_sequence,
                            MPCPrediction &state_prediction,
                            SolverInformation &solver_information) {
  double psi_avg = yaw_from_quaternion(
      quad_state_->GetOrientationInWorld());  // TODO: not really clear from the paper what is going on here
  double yaw_before = psi_avg;
  Eigen::Vector3d base_link_to_com = quad_model_->GetBodyToCOM().translation();
  for (int k = 0; k <= PREDICTION_HORIZON; k++) {
    double desired_yaw = yaw_from_quaternion(gait_sequence_.desired_reference_trajectory_orientation[k]);
    double yaw_difference = (desired_yaw - yaw_before);
    if (yaw_difference < -M_PI) {
      desired_yaw += 2 * M_PI;
    } else if (yaw_difference > M_PI) {
      desired_yaw -= 2 * M_PI;
    }
    yaw_before = desired_yaw;
    if (k < PREDICTION_HORIZON) {  // Input related stuff is only up to PREDICITION HORIZON - 1
      std::array<const Eigen::Ref<const Eigen::Vector3d>, 4> rs{
          gait_sequence_.foot_position_sequence[k][0]
              - (gait_sequence_.desired_reference_trajectory_position[k] + base_link_to_com),
          gait_sequence_.foot_position_sequence[k][1]
              - (gait_sequence_.desired_reference_trajectory_position[k] + base_link_to_com),
          gait_sequence_.foot_position_sequence[k][2]
              - (gait_sequence_.desired_reference_trajectory_position[k] + base_link_to_com),
          gait_sequence_.foot_position_sequence[k][3]
              - (gait_sequence_.desired_reference_trajectory_position[k] + base_link_to_com)};
      MPC::GetA(psi_avg, DeltaT, true, As_[k]);
      MPC::GetB(desired_yaw,
                quad_model_->GetMass(),
                quad_model_->GetInertia(),
                rs,
                gait_sequence_.contact_sequence[k],
                DeltaT,
                !model_update_,
                Bs_[k]);
      MPC::GetUBounds(gait_sequence_.contact_sequence[k], fmin_, fmax_, ulbs_[k], uubs_[k]);
      // TODO: clarify if this is nececarry or could be done once in constructor
      ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("A"), As_[k].data());
      ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("B"), Bs_[k].data());
      ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbu"), ulbs_[k].data());
      ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubu"), uubs_[k].data());
    }  // State realted stuff is over full horizon
    // Target state and put into q vector
    Eigen::Matrix<double, STATE_SIZE, 1> target_state;
    target_state.block<3, 1>(0, 0) =
        quaternion_to_euler(gait_sequence_.desired_reference_trajectory_orientation[k]);  // TODO: right conversion?
    target_state(2) = desired_yaw;
    target_state.block<3, 1>(3, 0) = gait_sequence_.desired_reference_trajectory_position[k] + base_link_to_com;
    target_state.block<3, 1>(6, 0) = gait_sequence_.reference_trajectory_twist[k];
    target_state.block<3, 1>(9, 0) = gait_sequence_.reference_trajectory_velocity[k];
    target_state(12, 0) = 0;
    qs_[k] = -(Q_ * target_state);
    ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("q"), qs_[k].data());

#ifdef DEBUG_PRINTS
    std::cout << "Set target x(" << k << ") to: \t[" << target_state(0) << ", " << target_state(1) << ", "
              << target_state(2) << ", " << target_state(3) << ", " << target_state(4) << ", " << target_state(5)
              << ", " << target_state(6) << ", " << target_state(7) << ", " << target_state(8) << ", "
              << target_state(9) << ", " << target_state(10) << ", " << target_state(11) << ", " << target_state(12)
              << "]" << std::endl;
#endif
  }
  // Set current state:
  x0_.block<3, 1>(0, 0) = quaternion_to_euler(quad_state_->GetOrientationInWorld());
  x0_.block<3, 1>(3, 0) = quad_state_->GetPositionInWorld() + base_link_to_com;
  x0_.block<3, 1>(6, 0) = quad_state_->GetAngularVelInWorld();
  x0_.block<3, 1>(9, 0) = quad_state_->GetLinearVelInWorld();
  x0_(12) = GRAVITY_CONSTANT;
  ocp_qp_in_set(solver_config_, qp_in_, 0, const_cast<char *>("lbx"), x0_.data());
  ocp_qp_in_set(solver_config_, qp_in_, 0, const_cast<char *>("ubx"), x0_.data());

#ifdef DEBUG_PRINTS
  std::cout << "Set x0 to: \t[" << x0_(0) << ", " << x0_(1) << ", " << x0_(2) << ", " << x0_(3) << ", " << x0_(4)
            << ", " << x0_(5) << ", " << x0_(6) << ", " << x0_(7) << ", " << x0_(8) << ", " << x0_(9) << ", " << x0_(10)
            << ", " << x0_(11) << ", " << x0_(12) << "]" << std::endl;
#endif

  // print_ocp_qp_in(qp_in_);
  // print_ocp_qp_dims(ocp_dims_);

  // Solving
  int acados_return = ocp_qp_solve(qp_solver_, qp_in_, qp_out_);

#ifdef DEBUG_PRINTS
  if (acados_return == ACADOS_SUCCESS) {
    std::cout << "ACADOS returned ACADOS_SUCCESS" << std::endl;
    // print_ocp_qp_out(qp_out_);
  } else if (acados_return == ACADOS_NAN_DETECTED) {
    std::cout << "ACADOS returned ACADOS_NAN_DETECTED" << std::endl;
  } else if (acados_return == ACADOS_MAXITER) {
    std::cout << "ACADOS returned ACADOS_MAXITER" << std::endl;
  } else if (acados_return == ACADOS_MINSTEP) {
    std::cout << "ACADOS returned ACADOS_MINSTEP" << std::endl;
  } else if (acados_return == ACADOS_QP_FAILURE) {
    std::cout << "ACADOS returned ACADOS_QP_FAILURE" << std::endl;
  } else if (acados_return == ACADOS_READY) {
    std::cout << "ACADOS returned ACADOS_READY" << std::endl;
  } else {
    std::cout << "ACADOS returned " << acados_return << std::endl;
  }

  ///* compute inf norm of residuals */
  // double res[4];
  // ocp_qp_inf_norm_residuals(ocp_dims_, qp_in_, qp_out_, res);
  // printf("\ninf norm res: stat %e, dyn %e, ineq %e, comp %e\n\n", res[0], res[1], res[2], res[3]);
  print_qp_info(reinterpret_cast<qp_info *>(qp_out_->misc));
#endif

  solver_information.return_code = acados_return;
  solver_information.total_solver_time = reinterpret_cast<qp_info *>(qp_out_->misc)->total_time;
  solver_information.acados_solve_QP_time = reinterpret_cast<qp_info *>(qp_out_->misc)->solve_QP_time;
  solver_information.acados_condensing_time = reinterpret_cast<qp_info *>(qp_out_->misc)->condensing_time;
  solver_information.acados_interface_time = reinterpret_cast<qp_info *>(qp_out_->misc)->interface_time;
  solver_information.acados_num_iter = reinterpret_cast<qp_info *>(qp_out_->misc)->num_iter;
  solver_information.acados_t_computed = reinterpret_cast<qp_info *>(qp_out_->misc)->t_computed;

  solver_information.success = (acados_return == ACADOS_SUCCESS);

  if (acados_return == ACADOS_SUCCESS) {
    // Some solver (including OSQP) might not check for NaN's, to not overwrite the last solution lets test the first
    // force for NaNs;
    if (Eigen::Map<Eigen::Matrix<double, STATE_SIZE + INPUT_SIZE, 1>>(qp_out_->ux->pa).hasNaN()) {
      std::cout << "NaN detected in solution after solver success" << std::endl;
      solver_information.return_code = -7777;
      solver_information.success = false;
      return;  // Immediately return to not destory data
    }
    // Get data
    for (int k = 0; k < PREDICTION_HORIZON; k++) {
      d_ocp_qp_sol_get_u(k, qp_out_, reinterpret_cast<double *>(wrench_sequence.forces[k].data()));
    }
    auto &state_prediction_raw = state_prediction.raw_data;
    for (int k = 0; k <= PREDICTION_HORIZON; k++) {
      d_ocp_qp_sol_get_x(k, qp_out_, state_prediction_raw[k].data());
      state_prediction.orientation[k] = euler_to_quaternion<double>(state_prediction_raw[k].block<3, 1>(0, 0));
      state_prediction.position[k] = state_prediction_raw[k].block<3, 1>(3, 0) - base_link_to_com;
      state_prediction.angular_velocity[k] = state_prediction_raw[k].block<3, 1>(6, 0);
      state_prediction.linear_velocity[k] = state_prediction_raw[k].block<3, 1>(9, 0);
    }
  }
  model_update_ = false;
}

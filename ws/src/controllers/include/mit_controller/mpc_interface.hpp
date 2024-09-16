#pragma once

#include <common/model_interface.hpp>

#include "gait_sequence.hpp"
#include "mpc_prediction.hpp"
#include "potato_sim/potato_model.hpp"
#include "wrench_sequence.hpp"

struct SolverInformation {
  bool success;
  int return_code;
  double total_solver_time;

  double acados_solve_QP_time;
  double acados_condensing_time;
  double acados_interface_time;
  int acados_num_iter;
  int acados_t_computed;
};

class MPCInterface {
 protected:
  MPCInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  virtual void UpdateState(const StateInterface &quad_state) = 0;
  virtual void UpdateModel(const ModelInterface &quad_model) = 0;
  virtual void UpdateGaitSequence(const GaitSequence &gait_sequence) = 0;
  virtual void GetWrenchSequence(WrenchSequence &wrench_sequence,
                                 MPCPrediction &state_prediction,
                                 SolverInformation &solver_information) = 0;
  virtual ~MPCInterface() = default;
};
#pragma once

#include <memory>  // for the smart pointers

#include "common/filters.hpp"
#include "model_adaptation_interface.hpp"

class LeastSquaresModelAdaptation : public ModelAdaptationInterface {
 public:
  LeastSquaresModelAdaptation(std::unique_ptr<ModelInterface> quad_model,
                              std::unique_ptr<StateInterface> initial_state,
                              int buffer_size,
                              Eigen::DiagonalMatrix<double, 10> Gamma,
                              double lambda);
  void UpdateState(const StateInterface& state) override;
  bool DoModelAdaptation(ModelInterface& model) override;
  void RegressorMatrix(const Eigen::Vector3d& orientation,
                       const Eigen::Vector3d& linear_acceleration,
                       const Eigen::Vector3d& angular_acceleration,
                       const std::array<Eigen::Vector3d, 4>& contact_forces,
                       Eigen::Matrix<double, 6, 10>& y) const;
  void bVec(const std::array<Eigen::Vector3d, 4>& contact_forces,
            const std::array<Eigen::Vector3d, 4>& foot_positions,
            Eigen::Vector<double, 6>& b) const;

 private:
  std::unique_ptr<StateInterface> state_;
  Eigen::Vector<double, 10> phi_;
  Eigen::Matrix<double, 10, 10> P_;
  int buffer_size_;
  MovingAverage<Eigen::Vector<double, 10>> output_filter_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> contact_force_buffer_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> foot_position_buffer_;
  Eigen::Vector3d orientation_buffer_;
  Eigen::Vector3d linear_acc_buffer_;
  Eigen::Vector3d angular_acc_buffer_;
  int msg_count_ = 0;
  Eigen::DiagonalMatrix<double, 10> Gamma_;
  double lambda_;
  const double g_ = 9.81;  // gravity TODO: make model parameter
};
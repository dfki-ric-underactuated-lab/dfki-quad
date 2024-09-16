// CODE Taken from https://github.com/RobotLocomotion/drake/blob/master/systems/controllers/pid_controller.h
// and adapted by Franek

#include "pd_changeable_controller.hpp"

#include <iostream>
#include <string>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
PdChangableController<T>::PdChangableController(int size)
    : PdChangableController(MatrixX<double>::Identity(2 * size, 2 * size)) {}

template <typename T>
PdChangableController<T>::PdChangableController(const MatrixX<double>& state_projection)
    : PdChangableController(
          state_projection,
          MatrixX<double>::Identity(int(state_projection.rows() / 2.0), int(state_projection.rows() / 2.0))) {}

template <typename T>
PdChangableController<T>::PdChangableController(const MatrixX<double>& state_projection,
                                                const MatrixX<double>& output_projection)
    : LeafSystem<T>(SystemTypeTag<PdChangableController>{}),
      num_controlled_q_(int(state_projection.rows() / 2.0)),
      num_full_state_(state_projection.cols()),
      state_projection_(state_projection),
      output_projection_(output_projection) {
  if (state_projection_.rows() != 2 * num_controlled_q_) {
    throw std::logic_error("State projection row dimension mismatch, expecting " + std::to_string(2 * num_controlled_q_)
                           + ", is " + std::to_string(state_projection_.rows()));
  }

  output_index_control_ =
      this->DeclareVectorOutputPort("control", output_projection_.rows(), &PdChangableController<T>::CalcControl)
          .get_index();

  input_index_state_ = this->DeclareVectorInputPort("estimated_state", num_full_state_).get_index();

  input_index_desired_state_ =
      this->DeclareInputPort("desired_state", kVectorValued, 2 * num_controlled_q_).get_index();

  input_index_kp_ = this->DeclareInputPort("kp", kVectorValued, num_controlled_q_).get_index();
  input_index_kd_ = this->DeclareInputPort("kd", kVectorValued, num_controlled_q_).get_index();
}

template <typename T>
template <typename U>
PdChangableController<T>::PdChangableController(const PdChangableController<U>& other)
    : PdChangableController(other.state_projection_, other.output_projection_) {}

template <typename T>
void PdChangableController<T>::CalcControl(const Context<T>& context, BasicVector<T>* control) const {
  const VectorX<T>& state = get_input_port_estimated_state().Eval(context);
  const VectorX<T>& state_d = get_input_port_desired_state().Eval(context);
  const auto kp = get_input_port_kp().Eval(context);
  const auto kd = get_input_port_kd().Eval(context);

  // State error.
  const VectorX<T> controlled_state = (state_projection_.cast<T>() * state);
  const VectorX<T> controlled_state_diff = state_d - controlled_state;
  const VectorX<T> controlled_state_effort =
      ((kp.array() * controlled_state_diff.head(num_controlled_q_).array()).matrix()
       + (kd.array() * controlled_state_diff.tail(num_controlled_q_).array()).matrix());
  const VectorX<T> output_effort = output_projection_.cast<T>() * controlled_state_effort;

  // Sets output to the sum of all three terms.
  control->SetFromVector(output_effort);

  // std::cout << "KP:" << kp.transpose() << std::endl;
  // std::cout << "KD:" << kd.transpose() << std::endl;
  // std::cout << "HEADError:" << controlled_state_diff.head(num_controlled_q_).transpose() << std::endl;
  // std::cout << "TAILError:" << controlled_state_diff.tail(num_controlled_q_).transpose() << std::endl;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::PdChangableController)

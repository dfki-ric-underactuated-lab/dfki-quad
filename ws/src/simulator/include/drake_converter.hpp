#pragma once
#include <Eigen/Core>

#include "drake/common/value.h"
#include "drake/systems/framework/leaf_system.h"

template <typename AbstractInputPortType, typename AbstractOutputPortType>
class DrakeConverter : public drake::systems::LeafSystem<double> {
  typedef typename std::function<void(const AbstractInputPortType &input, AbstractOutputPortType &output)>
      ConvertFunction;
  ConvertFunction convert_function_;
  drake::systems::InputPort<double> *drake_input_port_;
  drake::systems::OutputPort<double> *drake_output_port_;

 public:
  void CalculateOutPutPort(const drake::systems::Context<double> &context, AbstractOutputPortType *output) const {
    convert_function_(drake_input_port_->Eval<AbstractInputPortType>(context), *output);
  };
  DrakeConverter(ConvertFunction convert_function) : convert_function_(convert_function) {
    drake_input_port_ = &this->DeclareAbstractInputPort("AbstractInputPort", drake::Value<AbstractInputPortType>());
    drake_output_port_ = &this->DeclareAbstractOutputPort(
        "AbstractOutputPort", &DrakeConverter<AbstractInputPortType, AbstractOutputPortType>::CalculateOutPutPort);
  };
};
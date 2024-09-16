#pragma once
#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "gait_sequence.hpp"

class ModelAdaptationInterface {
 public:
  /**
   * Updates the current state
   *
   * @param state the new state
   */
  virtual void UpdateState(const StateInterface& state) = 0;
  /**
   * Updates the model according to state data
   *
   * @param model the model to update
   * @return if the model was updated
   */
  virtual bool DoModelAdaptation(ModelInterface& model) = 0;
};
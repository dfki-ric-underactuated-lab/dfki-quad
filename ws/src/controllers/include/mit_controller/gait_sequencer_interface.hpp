#pragma once

#include "common/model_interface.hpp"
#include "common/quaternion_operations.hpp"
#include "common/state_interface.hpp"
#include "gait_sequence.hpp"
#include "gait_sequencer_types.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "potato_sim/potato_model.hpp"
#include "rclcpp/time.hpp"
#include "target.hpp"

class GaitSequencerInterface {
 protected:
  GaitSequencerInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  virtual void GetGaitSequence(GaitSequence& gait_sequence) = 0;
  virtual void UpdateState(const StateInterface& quad_state) = 0;
  virtual void UpdateModel(const ModelInterface& quad_model) = 0;

  virtual void UpdateTarget(const Target& new_target) = 0;
  virtual void GetGaitState(interfaces::msg::GaitState& state) = 0;
  virtual GS_Type GetType() const = 0;

  virtual ~GaitSequencerInterface() = default;
};

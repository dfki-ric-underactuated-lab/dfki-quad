#pragma once

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "feet_targets.hpp"
#include "gait_sequence.hpp"

class SwingLegControllerInterface {
 public:
  enum LegState {
    /*
     * GS has scheduled this leg for standing
     */
    STANCE,
    /*
     * GS has scheduled the leg for swinging, but SLC has not yet moved this foot
     */
    NOT_STARTED,
    /**
     * SLC is currently swinging this leg
     */
    SWINGING,
    /**
     * SLC has reached the end of the slc for this feet
     */
    REACHED
  };

 protected:
  SwingLegControllerInterface() = default;

 public:
  virtual void UpdateGaitSequence(const GaitSequence &gs) = 0;
  virtual void UpdateState(const StateInterface &state) = 0;
  virtual void UpdateModel(const ModelInterface &model) = 0;
  virtual void GetFeetTargets(FeetTargets &feet_targets) = 0;
  virtual void GetProgress(std::array<double, N_LEGS> &progress, std::array<LegState, N_LEGS> &swing_states) = 0;
  virtual void GetCurrentTrajs(std::array<Eigen::Vector3d, N_LEGS> &start_pos,
                               std::array<Eigen::Vector3d, N_LEGS> &end_pos) = 0;
};
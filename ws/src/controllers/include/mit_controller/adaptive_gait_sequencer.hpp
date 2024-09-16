#pragma once

#include <array>
#include <rclcpp/time.hpp>

#include "gait.hpp"
#include "gait_sequencer_interface.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "mit_controller/target.hpp"
#include "mit_controller_params.hpp"
#include "mpc_trajectory_planner.hpp"
#include "raibert_foot_step_planner.hpp"

class AdaptiveGaitSequencer : public GaitSequencerInterface {
 public:
  AdaptiveGaitSequencer(const AdaptiveGait& gait,
                        double k,
                        const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                        std::unique_ptr<StateInterface> quad_state,
                        std::unique_ptr<ModelInterface> quad_model,
                        unsigned int raibert_filtersize,
                        bool raibert_z_on_plane,
                        bool fix_standing_position,
                        double fix_position_distance_threshold,
                        double fix_position_angular_threshold,
                        double fix_position_velocity_threshold,
                        bool early_contact_detection);

  void GetGaitSequence(GaitSequence& gait_sequence) override;
  void UpdateState(const StateInterface& quad_state) override;
  void UpdateTarget(const Target& new_target) override;
  void GetGaitState(interfaces::msg::GaitState& state) override;
  void UpdateModel(const ModelInterface& quad_model) override;
  GS_Type GetType() const override;

  AdaptiveGait& Gait();

 private:
  Target target_;
  std::unique_ptr<StateInterface> quad_state_;
  std::unique_ptr<ModelInterface> quad_model_;
  AdaptiveGait gait_;
  RaibertFootStepPlanner foot_step_planner_;
  MPCTrajectoryPlanner trajectory_planner_;
  bool early_contact_detection_;
};
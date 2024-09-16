#pragma once

#include <Eigen/Core>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/port_switch.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake_vizualizers/forces_visualizer.hpp"
#include "drake_vizualizers/positions_visualizer.hpp"
#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "mpc_prediction.hpp"
#include "potato_model.hpp"
#include "wrench_sequence.hpp"

class PotatoSimulator {
 private:
  static constexpr std::string_view ROBOT_URDF = "src/common/model/urdf/brick.urdf";
  static constexpr std::string_view WORLD_URDF = "src/common/model/urdf/plane.urdf";
  static constexpr std::string_view WOLRD_FIX_LINK = "plane_base_link";
  static constexpr unsigned int NUM_FEET = 4;

  static constexpr double SIM_RATE = 0.001;

  // Drake related members
  drake::systems::DiagramBuilder<double> *builder_;
  drake::multibody::MultibodyPlant<double> *plant_;
  drake::geometry::SceneGraph<double> *scene_graph_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::systems::Simulator<double> *simulator_;
  drake::multibody::BodyIndex plant_body_index_;

  drake::multibody::ExternallyAppliedSpatialForceMultiplexer<double> *forces_mux_;
  ForcesVizualiser *mpc_open_loop_forces_viz_;
  PositionsVizualiser *gs_foot_position_viz_;
  PositionsVizualiser *gs_robot_position_viz_;
  PositionsVizualiser *mpc_open_loop_robot_position_viz_;
  PositionsVizualiser *foot_state_viz_;

  std::array<Eigen::Vector3d, NUM_FEET> feet_positions_;

  // Gamepad status

 public:
  PotatoSimulator(const BrickState &init_state);
  virtual ~PotatoSimulator();

  void SetGateSequence(const GaitSequence &gs);
  /**
   * Simulation will apply the first Wrench/Torque out of this sequence.
   * @param ws sequence of wrenches to apply
   */
  void SetWrenchSequence(const WrenchSequence &ws,
                         const GaitSequence &gs,
                         const MPCPrediction &mpp,
                         const Eigen::Vector3d &F_dist = Eigen::Vector3d::Zero());
  void SetMPCStatePrediction(const MPCPrediction &mpcp);

  void SetFootPositions(const std::array<Eigen::Vector3d, NUM_FEET> &feet_pos);

  void GetState(BrickState &state, const std::array<bool, NUM_FEET> &foot_contacts) const;
  BrickModel GetModel() const;
  void Simulate(double seconds);

  drake::geometry::Meshcat::Gamepad GetGamepad();
};
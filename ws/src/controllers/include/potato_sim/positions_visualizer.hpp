#pragma once

#include "drake/common/value.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

class PositionsVizualiser : public drake::systems::LeafSystem<double> {
 private:
  drake::geometry::Meshcat &meshcat_;

  const int num_positions_;
  const double point_size_;
  const std::string prefix_name_;

  drake::systems::InputPort<double> *positions_input_port_;
  drake::systems::InputPort<double> *visible_input_port_;

 public:
  PositionsVizualiser(drake::geometry::Meshcat &meshcat,
                      const int num_positions,
                      const double point_size,
                      const drake::geometry::Rgba &color,
                      const std::string &prefix_name);

  void updatePositions(const drake::systems::Context<double> &context) const;

  const drake::systems::InputPort<double> &get_position_input_port();
  const drake::systems::InputPort<double> &get_visible_input_port();
};

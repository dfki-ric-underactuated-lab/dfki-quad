#pragma once

#include "drake/common/value.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

class ForcesVizualiser : public drake::systems::LeafSystem<double> {
 private:
  drake::geometry::Meshcat &meshcat_;

  const int num_forces_;
  const std::string body_link_name_;

  const double arrow_len_multiplier_;
  const double arrow_width_;
  drake::geometry::Rgba color_;

  drake::systems::InputPort<double> *body_poses_input_port_;
  drake::systems::InputPort<double> *forces_input_port_;

 public:
  ForcesVizualiser(drake::geometry::Meshcat &meshcat,
                   const int num_forces,
                   const std::string &bodyLinkName,
                   const double arrowLenMultiplier);

  void updateArrows(const drake::systems::Context<double> &context) const;
};

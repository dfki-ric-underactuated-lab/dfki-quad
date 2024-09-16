#pragma once

#include "drake/common/value.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

class VectorsVizualiser : public drake::systems::LeafSystem<double> {
 private:
  drake::geometry::Meshcat &meshcat_;

  int num_vectors_;

  const double arrow_width_;
  drake::geometry::Rgba color_;
  std::string name_prefix_;

  // All in world coordinates
  drake::systems::InputPort<double> *vectors_origin_input_port_;
  drake::systems::InputPort<double> *vectors_input_port_;

 public:
  VectorsVizualiser(drake::geometry::Meshcat &meshcat,
                    const std::string &name_prefix,
                    const int num_vectors,
                    const double width,
                    const drake::geometry::Rgba &color);

  drake::systems::EventStatus updateArrows(const drake::systems::Context<double> &context) const;
  drake::systems::InputPort<double> &get_vectors_input_port();
  drake::systems::InputPort<double> &get_vectors_origin_input_port();

  void updateNumberOfVectors(const int num_arrows);
};

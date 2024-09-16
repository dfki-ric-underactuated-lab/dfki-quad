#pragma once

#include "drake/common/value.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

class PosesVizualiser : public drake::systems::LeafSystem<double> {
 private:
  drake::geometry::Meshcat &meshcat_;

  int num_poses_;

  const double width_;
  const double len_;
  std::string name_prefix_;

  drake::math::RigidTransformd z_to_x_;
  drake::math::RigidTransformd z_to_y_;
  drake::math::RigidTransformd move_up_;

  // All in world coordinates
  drake::systems::InputPort<double> *poses_input_port_;

 public:
  PosesVizualiser(drake::geometry::Meshcat &meshcat,
                  const std::string &name_prefix,
                  const int num_poses,
                  const double width,
                  const double len);

  drake::systems::EventStatus updatePoses(const drake::systems::Context<double> &context) const;
  drake::systems::InputPort<double> &get_poses_input_port();
  void updateNumberOfPoses(const int num_positions);
};

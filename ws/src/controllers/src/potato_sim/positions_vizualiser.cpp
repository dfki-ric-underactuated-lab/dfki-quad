#include <fmt/core.h>
#include <math.h>

#include "potato_sim/positions_visualizer.hpp"

PositionsVizualiser::PositionsVizualiser(drake::geometry::Meshcat &meshcat,
                                         const int num_positions,
                                         const double point_size,
                                         const drake::geometry::Rgba &color,
                                         const std::string &prefix_name)
    : meshcat_(meshcat), num_positions_(num_positions), point_size_(point_size), prefix_name_(prefix_name) {
  // create all position markes
  drake::geometry::Sphere point(point_size_ / 2.);

  drake::math::RigidTransform position_transform(Eigen::Vector3d{0, 0, 0});

  for (int i = 0; i < num_positions_; i++) {
    meshcat_.SetObject(fmt::format("{}_point_{}", prefix_name_, i), point, color);
    meshcat_.SetTransform(fmt::format("{}_point_{}", prefix_name_, i), position_transform);
  }

  // create ports
  positions_input_port_ =
      &DeclareAbstractInputPort("positions_input_port_", drake::Value<std::vector<Eigen::Vector3d>>());
  visible_input_port_ = &DeclareAbstractInputPort("visible_input_port_", drake::Value<std::vector<bool>>());

  DeclarePeriodicPublishEvent(0.01, 0.0, &PositionsVizualiser::updatePositions);
}

void PositionsVizualiser::updatePositions(const drake::systems::Context<double> &context) const {
  auto positions = positions_input_port_->Eval<std::vector<Eigen::Vector3d>>(context);
  auto enableds = visible_input_port_->Eval<std::vector<bool>>(context);

  for (int i = 0; i < num_positions_; i++) {
    if (enableds[i]) {
      meshcat_.SetProperty(fmt::format("{}_point_{}", prefix_name_, i), "visible", true);
      meshcat_.SetTransform(fmt::format("{}_point_{}", prefix_name_, i), drake::math::RigidTransform(positions[i]));
    } else {
      meshcat_.SetProperty(fmt::format("{}_point_{}", prefix_name_, i), "visible", false);
    }
  }
}

const drake::systems::InputPort<double> &PositionsVizualiser::get_position_input_port() {
  return *positions_input_port_;
}

const drake::systems::InputPort<double> &PositionsVizualiser::get_visible_input_port() { return *visible_input_port_; }

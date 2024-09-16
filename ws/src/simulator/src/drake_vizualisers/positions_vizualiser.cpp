#include <fmt/core.h>
#include <math.h>

#include "drake_vizualizers/positions_visualizer.hpp"

PositionsVizualiser::PositionsVizualiser(drake::geometry::Meshcat &meshcat,
                                         const int num_positions,
                                         const double point_size,
                                         const drake::geometry::Rgba &color,
                                         const std::string &prefix_name)
    : meshcat_(meshcat), num_positions_(0), point_size_(point_size), color_(color), prefix_name_(prefix_name) {
  updateNumberOfPositions(num_positions);  // Will internally update num_positions_
  // create ports
  positions_input_port_ =
      &DeclareAbstractInputPort("positions_input_port_", drake::Value<std::vector<Eigen::Vector3d>>());
  visible_input_port_ = &DeclareAbstractInputPort("visible_input_port_", drake::Value<std::vector<bool>>());

  DeclarePeriodicPublishEvent(0.01, 0.0, &PositionsVizualiser::updatePositions);
  DeclareForcedPublishEvent(&PositionsVizualiser::updatePositions);
}

drake::systems::EventStatus PositionsVizualiser::updatePositions(const drake::systems::Context<double> &context) const {
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
  return drake::systems::EventStatus::Succeeded();
}

void PositionsVizualiser::updateNumberOfPositions(const int num_positions) {
  if (num_positions < num_positions_) {
    // Remove the last
    for (int i = num_positions_; i > num_positions; i--) {
      meshcat_.Delete(fmt::format("{}_point_{}", prefix_name_, (i - 1)));
    }
  } else if (num_positions > num_positions_) {
    // Add more
    for (int i = num_positions_; i < num_positions; i++) {
      drake::geometry::Sphere point(point_size_ / 2.);
      drake::math::RigidTransform position_transform(Eigen::Vector3d{0, 0, 0});

      meshcat_.SetObject(fmt::format("{}_point_{}", prefix_name_, i), point, color_);
      meshcat_.SetTransform(fmt::format("{}_point_{}", prefix_name_, i), position_transform);
    }
  }  // otherwise do nothing as they are equal
  num_positions_ = num_positions;
}

const drake::systems::InputPort<double> &PositionsVizualiser::get_position_input_port() {
  return *positions_input_port_;
}

const drake::systems::InputPort<double> &PositionsVizualiser::get_visible_input_port() { return *visible_input_port_; }

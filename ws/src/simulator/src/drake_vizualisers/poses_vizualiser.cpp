#include <fmt/core.h>
#include <math.h>

#include <iostream>

#include "drake_vizualizers/poses_visualizer.hpp"

PosesVizualiser::PosesVizualiser(drake::geometry::Meshcat &meshcat,
                                 const std::string &name_prefix,
                                 const int num_poses,
                                 const double width,
                                 const double len)
    : name_prefix_(name_prefix), meshcat_(meshcat), num_poses_(0), width_(width), len_(len) {
  // create all arrows in meshcat
  move_up_ = drake::math::RigidTransformd(Eigen::Vector3d(0., 0., len_ / 2.0));
  z_to_x_ =
      drake::math::RigidTransform(drake::math::RotationMatrix(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY())));
  z_to_y_ =
      drake::math::RigidTransform(drake::math::RotationMatrix(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())));

  updateNumberOfPoses(num_poses);
  // create ports
  poses_input_port_ =
      &DeclareAbstractInputPort("poses_input_port", drake::Value<std::vector<drake::math::RigidTransform<double>>>());
  DeclarePeriodicPublishEvent(0.01, 0.0, &PosesVizualiser::updatePoses);
  DeclareForcedPublishEvent(&PosesVizualiser::updatePoses);
}

drake::systems::EventStatus PosesVizualiser::updatePoses(const drake::systems::Context<double> &context) const {
  auto poses = poses_input_port_->Eval<std::vector<drake::math::RigidTransform<double>>>(context);

  for (int i = 0; i < num_poses_; i++) {
    meshcat_.SetTransform(fmt::format("{}_pose_{}", name_prefix_, i), poses[i]);
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::InputPort<double> &PosesVizualiser::get_poses_input_port() { return *poses_input_port_; }

void PosesVizualiser::updateNumberOfPoses(const int num_poses) {
  if (num_poses < num_poses_) {
    // Remove the last
    for (int i = num_poses_; i > num_poses; i--) {
      meshcat_.Delete(fmt::format("{}_pose_{}", name_prefix_, (i - 1)));
    }
  } else if (num_poses > num_poses_) {
    // Add more
    auto cyl = drake::geometry::Cylinder(width_, len_);
    drake::geometry::Rgba color_red(1.0, 0.0, 0, 1);
    drake::geometry::Rgba color_green(0.0, 1.0, 0.0, 1);
    drake::geometry::Rgba color_blue(0.0, 0., 1.0, 1);

    for (int i = num_poses_; i < num_poses; i++) {
      meshcat_.SetObject(fmt::format("{}_pose_{}/x", name_prefix_, i), cyl, color_red);
      meshcat_.SetObject(fmt::format("{}_pose_{}/y", name_prefix_, i), cyl, color_green);
      meshcat_.SetObject(fmt::format("{}_pose_{}/z", name_prefix_, i), cyl, color_blue);
      meshcat_.SetTransform(fmt::format("{}_pose_{}/x", name_prefix_, i), z_to_x_ * move_up_);
      meshcat_.SetTransform(fmt::format("{}_pose_{}/y", name_prefix_, i), z_to_y_ * move_up_);
      meshcat_.SetTransform(fmt::format("{}_pose_{}/z", name_prefix_, i), move_up_);
    }

  }  // otherwise do nothing as they are equal
  num_poses_ = num_poses;
}

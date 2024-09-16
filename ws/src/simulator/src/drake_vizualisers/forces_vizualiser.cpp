#include <fmt/core.h>
#include <math.h>

#include <iostream>

#include "drake_vizualizers/forces_visualizer.hpp"

ForcesVizualiser::ForcesVizualiser(drake::geometry::Meshcat &meshcat,
                                   const int num_forces,
                                   const std::string &bodyLinkName,
                                   const double arrowLenMultiplier)
    : meshcat_(meshcat),
      num_forces_(num_forces),
      body_link_name_(bodyLinkName),
      arrow_len_multiplier_(arrowLenMultiplier),
      arrow_width_(arrow_len_multiplier_ * 2) {
  // create all arrows in meshcat
  drake::geometry::Cylinder arrow_neck(arrow_width_, arrow_len_multiplier_);
  drake::geometry::MeshcatCone arrow_head(2 * arrow_width_, 2 * arrow_width_, 2 * arrow_width_);
  drake::geometry::Rgba color_red(1, 1.0, 0, 1);
  drake::geometry::Rgba color_red_late(1, 0, 0, 0.1);

  drake::math::RigidTransform arrow_neck_transform(Eigen::Vector3d{0.0, 0.0, 0.0});
  drake::math::RigidTransform arrow_head_transform(drake::math::RollPitchYaw(0., 0.0, 0.),
                                                   Eigen::Vector3d{0.0, 0.0, 0.0});

  for (int i = 0; i < num_forces_; i++) {
    color_ = color_red;
    if (i > 3) {
      color_ = color_red_late;
    }
    meshcat_.SetObject(fmt::format("arrow_{}/neck", i), arrow_neck, color_);
    meshcat_.SetObject(fmt::format("arrow_{}/head", i), arrow_head, color_);

    // meshcat_.SetTransform(fmt::format("arrow_{}/neck", i), arrow_neck_transform);
    // meshcat_.SetTransform(fmt::format("arrow_{}/head", i), arrow_head_transform);
  }

  // create ports
  body_poses_input_port_ = &DeclareAbstractInputPort("bpdy_poses_input_port",
                                                     drake::Value<std::vector<drake::math::RigidTransform<double>>>());
  forces_input_port_ = &DeclareAbstractInputPort(
      "forces_input_port_", drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>());
  DeclarePeriodicPublishEvent(0.01, 0.0, &ForcesVizualiser::updateArrows);
  DeclareForcedPublishEvent(&ForcesVizualiser::updateArrows);
}

drake::systems::EventStatus ForcesVizualiser::updateArrows(const drake::systems::Context<double> &context) const {
  auto forces = forces_input_port_->Eval<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(context);
  auto body_poses = body_poses_input_port_->Eval<std::vector<drake::math::RigidTransform<double>>>(context);

  for (int i = 0; i < num_forces_; i++) {
    Eigen::Vector3d force_W = forces[i].F_Bq_W.get_coeffs()(Eigen::seq(3, Eigen::last));

    if (force_W.norm() <= 0.01) {
      meshcat_.SetProperty(fmt::format("arrow_{}", i), "visible", false);
    } else {
      double len = force_W.norm() * arrow_len_multiplier_;
      auto &originTObody_INworld = body_poses[i];
      Eigen::Vector3d p_WoBq_W = originTObody_INworld * forces[i].p_BoBq_B;
      Eigen::Vector3d p_WoFhm_W = p_WoBq_W - force_W.normalized() * (len * 0.5 + 2 * arrow_width_);

      Eigen::Quaterniond zToFORCE;
      zToFORCE.setFromTwoVectors(Eigen::Vector3d::UnitZ(), force_W);
      static Eigen::Quaterniond flipneck(0.0, 1.0, 0.0, 0.0);

      meshcat_.SetObject(fmt::format("arrow_{}/neck", i), drake::geometry::Cylinder(arrow_width_, len), color_);
      meshcat_.SetTransform(fmt::format("arrow_{}/head", i),
                            drake::math::RigidTransformd(zToFORCE * flipneck, p_WoBq_W));
      meshcat_.SetTransform(fmt::format("arrow_{}/neck", i), drake::math::RigidTransformd(zToFORCE, p_WoFhm_W));
      meshcat_.SetProperty(fmt::format("arrow_{}", i), "visible", true);
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::InputPort<double> &ForcesVizualiser::get_body_poses_input_port() { return *body_poses_input_port_; }

drake::systems::InputPort<double> &ForcesVizualiser::get_forces_input_port() { return *forces_input_port_; }

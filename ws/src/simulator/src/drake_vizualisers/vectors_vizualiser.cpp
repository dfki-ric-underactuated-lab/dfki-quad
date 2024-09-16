#include <fmt/core.h>
#include <math.h>

#include <iostream>

#include "drake_vizualizers/vectors_visualizer.hpp"

VectorsVizualiser::VectorsVizualiser(drake::geometry::Meshcat &meshcat,
                                     const std::string &name_prefix,
                                     const int num_vectors,
                                     const double width,
                                     const drake::geometry::Rgba &color)
    : meshcat_(meshcat), num_vectors_(0), arrow_width_(width), color_(color), name_prefix_(name_prefix) {
  updateNumberOfVectors(num_vectors);
  // create ports
  vectors_origin_input_port_ =
      &DeclareAbstractInputPort("bpdy_poses_input_port", drake::Value<std::vector<Eigen::Vector3d>>());
  vectors_input_port_ = &DeclareAbstractInputPort("forces_input_port_", drake::Value<std::vector<Eigen::Vector3d>>());
  DeclarePeriodicPublishEvent(0.01, 0.0, &VectorsVizualiser::updateArrows);
  DeclareForcedPublishEvent(&VectorsVizualiser::updateArrows);
}

drake::systems::EventStatus VectorsVizualiser::updateArrows(const drake::systems::Context<double> &context) const {
  auto vectors = vectors_input_port_->Eval<std::vector<Eigen::Vector3d>>(context);
  auto vector_origins = vectors_origin_input_port_->Eval<std::vector<Eigen::Vector3d>>(context);

  for (int i = 0; i < num_vectors_; i++) {
    if (vectors[i].norm() <= 0.01) {
      meshcat_.SetProperty(fmt::format("{}_vector_{}/arrow", name_prefix_, i), "visible", false);
      meshcat_.SetProperty(fmt::format("{}_vector_{}/box", name_prefix_, i), "visible", true);
      meshcat_.SetTransform(fmt::format("{}_vector_{}/box", name_prefix_, i),
                            drake::math::RigidTransformd(vector_origins[i]));
    } else {
      double len = vectors[i].norm();

      Eigen::Quaterniond zToVec;
      zToVec.setFromTwoVectors(Eigen::Vector3d::UnitZ(), vectors[i]);
      static Eigen::Quaterniond flipneck(0.0, 1.0, 0.0, 0.0);

      meshcat_.SetObject(fmt::format("{}_vector_{}/arrow/neck", name_prefix_, i),
                         drake::geometry::Cylinder(arrow_width_, len),
                         color_);
      meshcat_.SetTransform(fmt::format("{}_vector_{}/arrow/head", name_prefix_, i),
                            drake::math::RigidTransformd(zToVec * flipneck, vector_origins[i] + vectors[i]));
      meshcat_.SetTransform(fmt::format("{}_vector_{}/arrow/neck", name_prefix_, i),
                            drake::math::RigidTransformd(
                                zToVec, vector_origins[i] + vectors[i].normalized() * (len - 2 * arrow_width_) * 0.5));
      meshcat_.SetProperty(fmt::format("{}_vector_{}/box", name_prefix_, i), "visible", false);
      meshcat_.SetProperty(fmt::format("{}_vector_{}/arrow", name_prefix_, i), "visible", true);
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::InputPort<double> &VectorsVizualiser::get_vectors_input_port() { return *vectors_input_port_; }

drake::systems::InputPort<double> &VectorsVizualiser::get_vectors_origin_input_port() {
  return *vectors_origin_input_port_;
}

void VectorsVizualiser::updateNumberOfVectors(const int num_vectors) {
  if (num_vectors < num_vectors_) {
    // Remove the last
    for (int i = num_vectors_; i > num_vectors; i--) {
      meshcat_.Delete(fmt::format("{}_vector_{}", name_prefix_, (i - 1)));
    }
  } else if (num_vectors > num_vectors_) {
    // Add more
    drake::geometry::Cylinder arrow_neck(arrow_width_, 0.1);
    drake::geometry::MeshcatCone arrow_head(2 * arrow_width_, 2 * arrow_width_, 2 * arrow_width_);
    drake::geometry::Box box(
        2 * arrow_width_, 2 * arrow_width_, 2 * arrow_width_);  // will be displayed if vector has length zero

    drake::math::RigidTransform arrow_neck_transform(Eigen::Vector3d{0.0, 0.0, 0.0});
    drake::math::RigidTransform arrow_head_transform(drake::math::RollPitchYaw(0., 0.0, 0.),
                                                     Eigen::Vector3d{0.0, 0.0, 0.0});

    for (int i = num_vectors_; i < num_vectors; i++) {
      meshcat_.SetObject(fmt::format("{}_vector_{}/arrow/neck", name_prefix_, i), arrow_neck, color_);
      meshcat_.SetObject(fmt::format("{}_vector_{}/arrow/head", name_prefix_, i), arrow_head, color_);
      meshcat_.SetObject(fmt::format("{}_vector_{}/box", name_prefix_, i), box, color_);
    }
  }  // otherwise do nothing as they are equal
  num_vectors_ = num_vectors;
}

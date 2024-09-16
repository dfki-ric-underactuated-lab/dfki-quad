#include "potato_sim/potato_sim.hpp"

#include "common/quaternion_operations.hpp"
#include "rclcpp/rclcpp.hpp"

PotatoSimulator::PotatoSimulator(const BrickState &init_state) {
  builder_ = new drake::systems::DiagramBuilder<double>;
  std::tie(plant_, scene_graph_) = drake::multibody::AddMultibodyPlantSceneGraph(builder_, SIM_RATE);
  auto parser = drake::multibody::Parser(plant_, scene_graph_);
  auto bodies = parser.AddModels(ROBOT_URDF);
  assert(bodies.size() == 1);
  plant_body_index_ = plant_->GetBodyByName("base_link").index();
  parser.AddModels(WORLD_URDF);
  plant_->WeldFrames(plant_->world_frame(), plant_->GetBodyByName(WOLRD_FIX_LINK).body_frame());
  plant_->set_contact_model(drake::multibody::ContactModel::kHydroelastic);
  plant_->set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
  plant_->Finalize();

  forces_mux_ = builder_->AddSystem<drake::multibody::ExternallyAppliedSpatialForceMultiplexer>(1);

  meshcat_ = std::make_shared<drake::geometry::Meshcat>();  // TODO: make meshcat a shared pointer, or maybe make all a
                                                            // sahred pointer
  mpc_open_loop_forces_viz_ =
      builder_->AddSystem<ForcesVizualiser>(*meshcat_.get(), NUM_FEET * MPC_PREDICTION_HORIZON, "base_link", 0.015);
  mpc_open_loop_robot_position_viz_ = builder_->AddSystem<PositionsVizualiser>(
      *meshcat_.get(), MPC_PREDICTION_HORIZON, 0.05, drake::geometry::Rgba(1.0, 0.0, 0.0, 1.0), "mpc_ol");
  gs_foot_position_viz_ = builder_->AddSystem<PositionsVizualiser>(
      *meshcat_.get(), NUM_FEET * GAIT_SEQUENCE_SIZE, 0.05, drake::geometry::Rgba(0.0, 0.0, 1.0, 1.0), "gs_foot_pos");
  gs_robot_position_viz_ = builder_->AddSystem<PositionsVizualiser>(
      *meshcat_.get(), GAIT_SEQUENCE_SIZE, 0.05, drake::geometry::Rgba(0.0, 0.0, 1.0, 1.0), "gs_robot_pos");
  foot_state_viz_ = builder_->AddSystem<PositionsVizualiser>(
      *meshcat_.get(), NUM_FEET, 0.08, drake::geometry::Rgba(0.0, 1.0, 0.0, 1.0), "foot_pos");

  builder_->Connect(forces_mux_->get_output_port(), plant_->get_applied_spatial_force_input_port());

  drake::visualization::AddDefaultVisualization(builder_, meshcat_);

  diagram_ = builder_->Build();
  simulator_ = new drake::systems::Simulator<double>(*diagram_);
  plant_->SetFreeBodyPose(&(plant_->GetMyMutableContextFromRoot(&(simulator_->get_mutable_context()))),
                          plant_->GetBodyByName("base_link"),
                          drake::math::RigidTransform(Eigen::Vector3d(0., 0., 0.25)));

  drake::Value<std::vector<bool>> gs_robot_position_all_enabled;
  gs_robot_position_all_enabled.get_mutable_value().resize(GAIT_SEQUENCE_SIZE, true);
  drake::Value<std::vector<bool>> mpc_robot_position_all_enabled;
  mpc_robot_position_all_enabled.get_mutable_value().resize(MPC_PREDICTION_HORIZON, true);
  drake::Value<std::vector<bool>> foot_position_all_enabled;
  foot_position_all_enabled.get_mutable_value().resize(NUM_FEET, true);
  foot_state_viz_->get_visible_input_port().FixValue(
      &foot_state_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), foot_position_all_enabled);

  gs_robot_position_viz_->get_visible_input_port().FixValue(
      &gs_robot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()),
      gs_robot_position_all_enabled);
  mpc_open_loop_robot_position_viz_->get_visible_input_port().FixValue(
      &mpc_open_loop_robot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()),
      mpc_robot_position_all_enabled);
  plant_->SetFreeBodyPose(&plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()),
                          plant_->get_body(plant_body_index_),
                          drake::math::RigidTransform<double>(init_state.orientation_, init_state.position_));
  meshcat_->Flush();
}

PotatoSimulator::~PotatoSimulator() {}

void PotatoSimulator::SetGateSequence(const GaitSequence &gs) {
  static drake::Value<std::vector<Eigen::Vector3d>> positions;
  positions.get_mutable_value().resize(GAIT_SEQUENCE_SIZE);
  static drake::Value<std::vector<Eigen::Vector3d>> foot_steps;
  foot_steps.get_mutable_value().resize(NUM_FEET * GAIT_SEQUENCE_SIZE);
  static drake::Value<std::vector<bool>> foot_step_enabled;
  foot_step_enabled.get_mutable_value().resize(NUM_FEET * GAIT_SEQUENCE_SIZE);

  for (unsigned int gait_step = 0; gait_step < GAIT_SEQUENCE_SIZE; gait_step++) {
    positions.get_mutable_value()[gait_step] = gs.reference_trajectory_position[gait_step];
    for (unsigned int foot_idx = 0; foot_idx < NUM_FEET; foot_idx++) {
      foot_steps.get_mutable_value()[gait_step * NUM_FEET + foot_idx] = gs.foot_position_sequence[gait_step][foot_idx];
      foot_step_enabled.get_mutable_value()[gait_step * NUM_FEET + foot_idx] = gs.contact_sequence[gait_step][foot_idx];
    }
  }

  gs_foot_position_viz_->get_position_input_port().FixValue(
      &gs_foot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), foot_steps);
  gs_foot_position_viz_->get_visible_input_port().FixValue(
      &gs_foot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), foot_step_enabled);
  gs_robot_position_viz_->get_position_input_port().FixValue(
      &gs_robot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), positions);
}

void PotatoSimulator::SetWrenchSequence(const WrenchSequence &ws,
                                        const GaitSequence &gs,
                                        const MPCPrediction &mpp,
                                        const Eigen::Vector3d &F_dist) {
  static drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>> forces_to_viz;
  forces_to_viz.get_mutable_value().resize(NUM_FEET * MPC_PREDICTION_HORIZON);
  static drake::Value<std::vector<drake::math::RigidTransform<double>>> forces_body_poses;
  forces_body_poses.get_mutable_value().resize(NUM_FEET * MPC_PREDICTION_HORIZON);
  static drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>> forces_to_apply;
  forces_to_apply.get_mutable_value().resize(NUM_FEET + 1);
  for (unsigned int mpc_step = 0; mpc_step < MPC_PREDICTION_HORIZON; mpc_step++) {
    drake::math::RigidTransform<double> body_pose_in_world(mpp.orientation[mpc_step], mpp.position[mpc_step]);
    for (unsigned int foot_idx = 0; foot_idx < NUM_FEET; foot_idx++) {
      forces_to_viz.get_mutable_value()[mpc_step * NUM_FEET + foot_idx].body_index = plant_body_index_;
      if (gs.contact_sequence[mpc_step][foot_idx]) {
        forces_to_viz.get_mutable_value()[mpc_step * NUM_FEET + foot_idx].F_Bq_W =
            drake::multibody::SpatialForce<double>(Eigen::Vector3d::Zero(), ws.forces[mpc_step][foot_idx]);
        forces_to_viz.get_mutable_value()[mpc_step * NUM_FEET + foot_idx].p_BoBq_B =
            body_pose_in_world.inverse() * gs.foot_position_sequence[mpc_step][foot_idx];
        forces_body_poses.get_mutable_value()[mpc_step * NUM_FEET + foot_idx] =
            drake::math::RigidTransformd(mpp.orientation[mpc_step], mpp.position[mpc_step]);
      } else {
        forces_to_viz.get_mutable_value()[mpc_step * NUM_FEET + foot_idx].F_Bq_W =
            drake::multibody::SpatialForce<double>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        forces_to_viz.get_mutable_value()[mpc_step * NUM_FEET + foot_idx].p_BoBq_B = Eigen::Vector3d::Zero();
      }

      if (mpc_step == 0) {
        forces_to_apply.get_mutable_value()[foot_idx] = forces_to_viz.get_value()[mpc_step * NUM_FEET + foot_idx];
      }
    }
  }

  // Disturbances
  forces_to_apply.get_mutable_value()[NUM_FEET].body_index = plant_body_index_;
  forces_to_apply.get_mutable_value()[NUM_FEET].F_Bq_W =
      drake::multibody::SpatialForce<double>(Eigen::Vector3d::Zero(), F_dist);
  forces_to_apply.get_mutable_value()[NUM_FEET].p_BoBq_B = Eigen::Vector3d::Zero();
  // forces_to_viz.get_mutable_value()[NUM_FEET * MPC_PREDICTION_HORIZON] = forces_to_apply.get_value()[NUM_FEET];
  // forces_body_poses.get_mutable_value()[NUM_FEET * MPC_PREDICTION_HORIZON] =
  // drake::math::RigidTransformd(mpp.orientation[0],mpp.position[0]);

  mpc_open_loop_forces_viz_->get_input_port(0).FixValue(
      &mpc_open_loop_forces_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), forces_body_poses);
  mpc_open_loop_forces_viz_->get_input_port(1).FixValue(
      &mpc_open_loop_forces_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), forces_to_viz);
  forces_mux_->get_input_port(0).FixValue(&forces_mux_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()),
                                          forces_to_apply);
}

void PotatoSimulator::SetMPCStatePrediction(const MPCPrediction &mpcp) {
  static drake::Value<std::vector<Eigen::Vector3d>> positions;
  positions.get_mutable_value().resize(MPC_PREDICTION_HORIZON);

  for (unsigned int mpc_step = 0; mpc_step < MPC_PREDICTION_HORIZON; mpc_step++) {
    positions.get_mutable_value()[mpc_step] = mpcp.position[mpc_step];
  }
  mpc_open_loop_robot_position_viz_->get_position_input_port().FixValue(
      &mpc_open_loop_robot_position_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), positions);
}

void PotatoSimulator::GetState(BrickState &state, const std::array<bool, NUM_FEET> &foot_contacts) const {
  state.time_stamp_ = StateInterface::TimePoint(
      StateInterface::TimePoint::duration(long(simulator_->get_context().get_time() * std::nano::den)));

  auto brick_state = plant_->get_state_output_port().Eval(plant_->GetMyContextFromRoot(simulator_->get_context()));
  auto brick_acceleration = plant_->EvalBodySpatialAccelerationInWorld(
      plant_->GetMyContextFromRoot(simulator_->get_context()), plant_->get_body(plant_body_index_));

  state.orientation_ = Eigen::Quaterniond(brick_state(0), brick_state(1), brick_state(2), brick_state(3));
  state.position_ = brick_state.block<3, 1>(4, 0);
  state.angular_vel_ = brick_state.block<3, 1>(7, 0);
  state.linear_vel_ = brick_state.block<3, 1>(10, 0);
  state.linear_acc_ = brick_acceleration.translational();
  state.angular_acc_ = brick_acceleration.rotational();
  state.feet_contacts_ = foot_contacts;
  state.virt_feet_positions_ = feet_positions_;
}

void PotatoSimulator::SetFootPositions(const std::array<Eigen::Vector3d, NUM_FEET> &feet_pos) {
  feet_positions_ = feet_pos;
  static drake::Value<std::vector<Eigen::Vector3d>> positions;
  positions.get_mutable_value().resize(NUM_FEET);
  for (unsigned int foot_idx = 0; foot_idx < NUM_FEET; foot_idx++) {
    positions.get_mutable_value()[foot_idx] = feet_pos[foot_idx];
  }

  foot_state_viz_->get_position_input_port().FixValue(
      &foot_state_viz_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context()), positions);
}

BrickModel PotatoSimulator::GetModel() const {
  const drake::multibody::SpatialInertia<double> I =
      plant_->CalcSpatialInertia(plant_->GetMyContextFromRoot(simulator_->get_context()),
                                 plant_->GetFrameByName("base_link"),
                                 {plant_body_index_});
  auto base_inertia_ =
      I.CalcRotationalInertia().ShiftToCenterOfMassInPlace(I.get_mass(), I.get_com()).CopyToFullMatrix3();

  return BrickModel(
      base_inertia_,
      plant_->get_body(plant_body_index_).get_mass(plant_->GetMyContextFromRoot(simulator_->get_context())));
}

void PotatoSimulator::Simulate(double seconds) {
  double current_time = simulator_->get_context().get_time();
  meshcat_->Flush();
  simulator_->AdvanceTo(current_time + seconds);
  meshcat_->Flush();
}

drake::geometry::Meshcat::Gamepad PotatoSimulator::GetGamepad() { return meshcat_->GetGamepad(); }

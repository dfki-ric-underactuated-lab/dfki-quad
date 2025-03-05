#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "common/custom_qos.hpp"
#include "common/eigen_msg_conversions.hpp"
#include "common/sequence_containers.hpp"
#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake_vizualizers/forces_visualizer.hpp"
#include "drake_vizualizers/poses_visualizer.hpp"
#include "drake_vizualizers/positions_visualizer.hpp"
#include "drake_vizualizers/vectors_visualizer.hpp"
#include "interfaces/msg/gait_sequence.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/joint_state.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/position_sequence.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/msg/vector_sequence.hpp"
#include "interfaces/msg/wbc_target.hpp"
#include "rclcpp/rclcpp.hpp"

#define GS_SIZE 11
#define OPEN_LOOP_SIZE 10

using namespace std::chrono_literals;
using namespace drake;
using std::placeholders::_1;

class DrakeVisualizer : public rclcpp::Node {
 public:
  DrakeVisualizer() : Node("drake_visualizer"), leg_cmd_recv_(false) {
    // parameters
    this->declare_parameter("topic", 0);  // 0: joint_states; 1: joint_cmd; 2: leg_joint_cmd; 3:quad_state
    this->declare_parameter("model_urdf", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("foot_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

    // read parameters
    visualizer_topic_ = static_cast<VisualizerTopic>(this->get_parameter("topic").as_int());

    // Set up Drake Stuff
    std::tie(plant, scene_graph) = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser(plant).AddModels(this->get_parameter("model_urdf").as_string());
    plant->Finalize();
    meshcat_ = std::make_shared<geometry::Meshcat>();
    visualization.publish_contacts = true;     // false for the "shadow" model rendering
    visualization.publish_period = 1.0 / 30.;  // 30Hz
    visualization::ApplyVisualizationConfig(visualization, &builder, nullptr, nullptr, nullptr, meshcat_);

    // Subscribers
    switch (visualizer_topic_) {
      case QUAD_STATE:
        quad_state_sub_ = this->create_subscription<interfaces::msg::QuadState>(
            "quad_state", QOS_BEST_EFFORT_NO_DEPTH, std::bind(&DrakeVisualizer::quad_subscriber_callback, this, _1));
        leg_cmd_sub_ = this->create_subscription<interfaces::msg::LegCmd>(
            "leg_cmd", QOS_BEST_EFFORT_NO_DEPTH, [this](interfaces::msg::LegCmd::SharedPtr msg) {
              leg_cmd_msg_ = *msg;
              leg_cmd_recv_ = true;
            });
        ee_pos_viz_ =
            builder.AddSystem<PositionsVizualiser>(*meshcat_, 4, 0.03, geometry::Rgba(1, 0, 1), "leg_cmd_pos");
        ee_force_viz_ = builder.AddSystem<ForcesVizualiser>(*meshcat_, 4, "", 0.015);
        break;
      case JOINT_STATES:
        joint_state_sub_ = this->create_subscription<interfaces::msg::JointState>(
            "joint_states", QOS_BEST_EFFORT_NO_DEPTH, std::bind(&DrakeVisualizer::joint_subscriber_callback, this, _1));
        break;
      case JOINT_CMD:
        joint_cmd_sub_ = this->create_subscription<interfaces::msg::JointCmd>(
            "joint_cmd",
            QOS_BEST_EFFORT_NO_DEPTH,
            std::bind(&DrakeVisualizer::joint_cmd_subscriber_callback, this, _1));
        break;
      case LEG_JOINT_CMD:
        joint_cmd_sub_ = this->create_subscription<interfaces::msg::JointCmd>(
            "leg_joint_cmd",
            QOS_RELIABLE_NO_DEPTH,
            std::bind(&DrakeVisualizer::joint_cmd_subscriber_callback, this, _1));
        break;
    }
    gait_sequence_sub_ = this->create_subscription<interfaces::msg::GaitSequence>(
        "gait_sequence", QOS_BEST_EFFORT_NO_DEPTH, [this](interfaces::msg::GaitSequence::SharedPtr msg) {
          gait_sequence_msg_ = *msg;
        });

    foot_step_plan_viz_ =
        builder.AddSystem<PositionsVizualiser>(*meshcat_, 0, 0.03, geometry::Rgba(1, 0, 0), "planned_foot_steps");
    swing_leg_traj_sub_ = this->create_subscription<interfaces::msg::VectorSequence>(
        "swing_leg_trajs", QOS_BEST_EFFORT_NO_DEPTH, [this](interfaces::msg::VectorSequence::SharedPtr msg) {
          swing_leg_traj_msg_ = *msg;
        });
    swing_leg_traj_viz_ =
        builder.AddSystem<VectorsVizualiser>(*meshcat_, "swing_trajs", 0, 0.01, geometry::Rgba(0, 0, 1));

    target_trajectory_viz_ =
        builder.AddSystem<PositionsVizualiser>(*meshcat_, 0, 0.005, geometry::Rgba(1, 0, 0), "planned_positions");

    open_loop_trajectory_sub_ = this->create_subscription<interfaces::msg::PositionSequence>(
        "open_loop_trajectory", QOS_BEST_EFFORT_NO_DEPTH, [this](interfaces::msg::PositionSequence::SharedPtr msg) {
          open_loop_trajectory_msg_ = *msg;
        });
    open_loop_trajectory_viz_ =
        builder.AddSystem<PositionsVizualiser>(*meshcat_, 0, 0.005, geometry::Rgba(0, 0, 1), "open_loop_positions");

    open_loop_poses_viz_ = builder.AddSystem<PosesVizualiser>(*meshcat_, "open_loop_poses", 0, 0.0025, 0.1);

    wbc_target_sub_ = this->create_subscription<interfaces::msg::WBCTarget>(
        "wbc_target", QOS_BEST_EFFORT_NO_DEPTH, [this](interfaces::msg::WBCTarget::SharedPtr msg) {
          wbc_target_msg_ = *msg;
        });
    wbc_target_msg_.feet_pos_targets.fill(eigenToRosMsg<geometry_msgs::msg::Vector3>(Eigen::Vector3d::Zero()));
    wbc_target_msg_.feet_wrenches.fill(eigenToRosMsg<geometry_msgs::msg::Vector3>(Eigen::Vector3d::Zero()));
    wbc_target_msg_.feet_vel_targets.fill(eigenToRosMsg<geometry_msgs::msg::Vector3>(Eigen::Vector3d::Zero()));
    wbc_target_msg_.feet_acc_targets.fill(eigenToRosMsg<geometry_msgs::msg::Vector3>(Eigen::Vector3d::Zero()));
    wbc_target_msg_.feet_contacts.fill(false);

    wbc_target_pos_contact_viz_ = builder.AddSystem<PositionsVizualiser>(
        *meshcat_, 4, 0.05, geometry::Rgba(1, 0, 1), "wbc_target_feet_pos_active");
    wbc_target_pos_no_contact_viz_ = builder.AddSystem<PositionsVizualiser>(
        *meshcat_, 4, 0.05, geometry::Rgba(0, 1, 1), "wbc_target_feet_pos_not_active");
    wbc_target_vel_viz_ =
        builder.AddSystem<VectorsVizualiser>(*meshcat_, "wbc_target_feet_vel", 4, 0.005, geometry::Rgba(0, 0, 1));
    wbc_target_force_viz_ =
        builder.AddSystem<VectorsVizualiser>(*meshcat_, "wbc_target_feet_force", 4, 0.005, geometry::Rgba(0, 1, 0));

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();

    // Timers
    update_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::microseconds(static_cast<int>(1e6 / update_freq_)),
                                         std::bind(&DrakeVisualizer::timer_callback_update_viz, this));

    // Initialize the joint_msg_ to not have seg fault before receiving the
    // first message
    joint_msg_.position.fill(0);
    joint_msg_.velocity.fill(0);
    joint_msg_.effort.fill(0);
    joint_msg_.acceleration.fill(0);
    joint_cmd_msg_.position.fill(0);
    joint_cmd_msg_.velocity.fill(0);
    joint_cmd_msg_.effort.fill(0);
    quad_state_.joint_state.position.fill(0);
    geometry_msgs::msg::Point zeroP;
    geometry_msgs::msg::Vector3 zeroV;
    zeroP.x = 0;
    zeroP.y = 0;
    zeroP.z = 0;
    zeroV.x = 0;
    zeroV.y = 0;
    zeroV.z = 0;
    leg_cmd_msg_.ee_pos.fill(0);
    leg_cmd_msg_.ee_force.fill(0);
    leg_cmd_msg_.kp.fill(0);
  }

  ~DrakeVisualizer() = default;

  // Helper functions
  void joint_subscriber_callback(const interfaces::msg::JointState::SharedPtr msg) { joint_msg_ = *msg; };

  void joint_cmd_subscriber_callback(const interfaces::msg::JointCmd::SharedPtr msg) { joint_cmd_msg_ = *msg; };

  void quad_subscriber_callback(const interfaces::msg::QuadState::SharedPtr msg) { quad_state_ = *msg; };

  void timer_callback_update_viz() {
    // Update the Drake Visualizer
    static Eigen::VectorXd quad_state(19);
    static Value<std::vector<bool>> ee_pos_visible(std::vector<bool>(4, false));
    static Value<std::vector<Eigen::Vector3d>> ee_pos(std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero()));
    static Value<std::vector<multibody::ExternallyAppliedSpatialForce<double>>> ee_force(
        std::vector<multibody::ExternallyAppliedSpatialForce<double>>(4));
    static Value<std::vector<math::RigidTransformd>> ee_body_pose(
        std::vector<math::RigidTransformd>(4, math::RigidTransformd::Identity()));

    // Quaternions
    quad_state(0) = 1.0;
    quad_state(1) = 0.0;
    quad_state(2) = 0.0;
    quad_state(3) = 0.0;
    // Base Position
    quad_state(4) = 0.0;
    quad_state(5) = 0.0;
    quad_state(6) = 0.0;

    // Joint Positions
    switch (visualizer_topic_) {
      case QUAD_STATE: {
        quad_state(0) = quad_state_.pose.pose.orientation.w;
        quad_state(1) = quad_state_.pose.pose.orientation.x;
        quad_state(2) = quad_state_.pose.pose.orientation.y;
        quad_state(3) = quad_state_.pose.pose.orientation.z;
        quad_state(4) = quad_state_.pose.pose.position.x;
        quad_state(5) = quad_state_.pose.pose.position.y;
        quad_state(6) = quad_state_.pose.pose.position.z;
        for (int i = 0; i < 12; i++) {
          quad_state(i + 7) = quad_state_.joint_state.position[i];
        }
        plant->SetPositions(&plant->GetMyMutableContextFromRoot(context_.get()),
                            quad_state);  // Nececary to use the other transformations
        auto world_to_body =
            plant->GetBodyByName("base_link").EvalPoseInWorld(plant->GetMyContextFromRoot(*(context_.get())));
        for (int i = 0; i < 4; i++) {
          if (leg_cmd_msg_.kp[i] > std::numeric_limits<double>::epsilon()) {
            // Only if the P gain is set we have a position to viz
            ee_pos_visible.get_mutable_value()[i] = true;
            // Transform this into world
            ee_pos.get_mutable_value()[i] =
                world_to_body
                * Eigen::Vector3d(
                    leg_cmd_msg_.ee_pos[i * 3 + 0], leg_cmd_msg_.ee_pos[i * 3 + 1], leg_cmd_msg_.ee_pos[i * 3 + 2]);
          } else {
            ee_pos_visible.get_mutable_value()[i] = false;
          }
          // Forces will always be applied and hence visualized
          ee_force.get_mutable_value()[i].F_Bq_W =
              multibody::SpatialForce<double>(Eigen::Vector3d::Zero(),
                                              world_to_body.rotation()
                                                  * Eigen::Vector3d(leg_cmd_msg_.ee_force[i * 3 + 0],
                                                                    leg_cmd_msg_.ee_force[i * 3 + 1],
                                                                    leg_cmd_msg_.ee_force[i * 3 + 2]));
          static auto foot_names = this->get_parameter("foot_names").as_string_array();
          ee_force.get_mutable_value()[i].p_BoBq_B =
              plant
                  ->CalcRelativeTransform(plant->GetMyContextFromRoot(*(context_.get())),
                                          plant->GetFrameByName("base_link"),
                                          plant->GetFrameByName(foot_names[i]))
                  .translation();  // TODO: verify if this in the right coordinates
          ee_body_pose.get_mutable_value()[i] = world_to_body;
        }
        ee_pos_viz_->get_position_input_port().FixValue(&ee_pos_viz_->GetMyMutableContextFromRoot(context_.get()),
                                                        ee_pos);
        ee_pos_viz_->get_visible_input_port().FixValue(&ee_pos_viz_->GetMyMutableContextFromRoot(context_.get()),
                                                       ee_pos_visible);
        ee_force_viz_->get_forces_input_port().FixValue(&ee_force_viz_->GetMyMutableContextFromRoot(context_.get()),
                                                        ee_force);
        ee_force_viz_->get_body_poses_input_port().FixValue(&ee_force_viz_->GetMyMutableContextFromRoot(context_.get()),
                                                            ee_body_pose);
      } break;
      case JOINT_STATES:
        for (int i = 0; i < 12; i++) {
          quad_state(i + 7) = joint_msg_.position[i];
        }

        break;
      case JOINT_CMD:
      case LEG_JOINT_CMD:
        for (int i = 0; i < 12; i++) {
          quad_state(i + 7) = joint_cmd_msg_.position[i];
        }
        break;
    }

    foot_step_plan_viz_->updateNumberOfPositions(4 * GS_SIZE);
    static std::vector<bool> foot_step_viz;
    static std::vector<Eigen::Vector3d> foot_steps;
    foot_step_viz.resize(4 * GS_SIZE);
    foot_steps.resize(4 * GS_SIZE);
    for (int i = 0; i < GS_SIZE; i++) {
      foot_step_viz[0 * GS_SIZE + i] = gait_sequence_msg_.contact_sequence_0[i];
      foot_step_viz[1 * GS_SIZE + i] = gait_sequence_msg_.contact_sequence_1[i];
      foot_step_viz[2 * GS_SIZE + i] = gait_sequence_msg_.contact_sequence_2[i];
      foot_step_viz[3 * GS_SIZE + i] = gait_sequence_msg_.contact_sequence_3[i];
      foot_steps[0 * GS_SIZE + i] = {gait_sequence_msg_.foot_position_sequence_x_0[i],
                                     gait_sequence_msg_.foot_position_sequence_y_0[i],
                                     gait_sequence_msg_.foot_position_sequence_z_0[i]};
      foot_steps[1 * GS_SIZE + i] = {gait_sequence_msg_.foot_position_sequence_x_1[i],
                                     gait_sequence_msg_.foot_position_sequence_y_1[i],
                                     gait_sequence_msg_.foot_position_sequence_z_1[i]};
      foot_steps[2 * GS_SIZE + i] = {gait_sequence_msg_.foot_position_sequence_x_2[i],
                                     gait_sequence_msg_.foot_position_sequence_y_2[i],
                                     gait_sequence_msg_.foot_position_sequence_z_2[i]};
      foot_steps[3 * GS_SIZE + i] = {gait_sequence_msg_.foot_position_sequence_x_3[i],
                                     gait_sequence_msg_.foot_position_sequence_y_3[i],
                                     gait_sequence_msg_.foot_position_sequence_z_3[i]};
    }
    foot_step_plan_viz_->get_visible_input_port().FixValue(
        &foot_step_plan_viz_->GetMyMutableContextFromRoot(context_.get()), foot_step_viz);
    foot_step_plan_viz_->get_position_input_port().FixValue(
        &foot_step_plan_viz_->GetMyMutableContextFromRoot(context_.get()), foot_steps);

    swing_leg_traj_viz_->updateNumberOfVectors(4);
    static std::vector<Eigen::Vector3d> swing_leg_vectors;
    static std::vector<Eigen::Vector3d> swing_leg_vector_origins;
    swing_leg_vectors.resize(4);
    swing_leg_vector_origins.resize(4);
    for (int i = 0; i < 4; i++) {
      swing_leg_vectors[i] = {
          swing_leg_traj_msg_.vector_x[i], swing_leg_traj_msg_.vector_y[i], swing_leg_traj_msg_.vector_z[i]};
      swing_leg_vector_origins[i] = {
          swing_leg_traj_msg_.origin_x[i], swing_leg_traj_msg_.origin_y[i], swing_leg_traj_msg_.origin_z[i]};
    }
    swing_leg_traj_viz_->get_vectors_input_port().FixValue(
        &swing_leg_traj_viz_->GetMyMutableContextFromRoot(context_.get()), swing_leg_vectors);
    swing_leg_traj_viz_->get_vectors_origin_input_port().FixValue(
        &swing_leg_traj_viz_->GetMyMutableContextFromRoot(context_.get()), swing_leg_vector_origins);

    open_loop_trajectory_viz_->updateNumberOfPositions(OPEN_LOOP_SIZE);
    open_loop_poses_viz_->updateNumberOfPoses(OPEN_LOOP_SIZE);
    static std::vector<bool> open_loop_position_viz;
    static std::vector<Eigen::Vector3d> open_loop_positions;
    static std::vector<drake::math::RigidTransformd> open_loop_poses;
    open_loop_position_viz.resize(OPEN_LOOP_SIZE);
    open_loop_positions.resize(OPEN_LOOP_SIZE);
    open_loop_poses.resize(OPEN_LOOP_SIZE);
    for (int i = 0; i < OPEN_LOOP_SIZE; i++) {
      open_loop_position_viz[i] = open_loop_trajectory_msg_.valid[i];
      open_loop_positions[i] = {
          open_loop_trajectory_msg_.x[i], open_loop_trajectory_msg_.y[i], open_loop_trajectory_msg_.z[i]};
      if (open_loop_trajectory_msg_.qw[i] == 0.0 && open_loop_trajectory_msg_.qx[i] == 0.0
          && open_loop_trajectory_msg_.qy[i] == 0.0 && open_loop_trajectory_msg_.qz[i] == 0.0) {
        open_loop_trajectory_msg_.qw[i] = 1.0;
        open_loop_position_viz[i] = false;
      }
      open_loop_poses[i] = drake::math::RigidTransformd(Eigen::Quaterniond(open_loop_trajectory_msg_.qw[i],
                                                                           open_loop_trajectory_msg_.qx[i],
                                                                           open_loop_trajectory_msg_.qy[i],
                                                                           open_loop_trajectory_msg_.qz[i]),
                                                        open_loop_positions[i]);
    }
    open_loop_trajectory_viz_->get_visible_input_port().FixValue(
        &open_loop_trajectory_viz_->GetMyMutableContextFromRoot(context_.get()), open_loop_position_viz);
    open_loop_trajectory_viz_->get_position_input_port().FixValue(
        &open_loop_trajectory_viz_->GetMyMutableContextFromRoot(context_.get()), open_loop_positions);
    open_loop_poses_viz_->get_poses_input_port().FixValue(
        &open_loop_poses_viz_->GetMyMutableContextFromRoot(context_.get()), open_loop_poses);
    target_trajectory_viz_->updateNumberOfPositions(GS_SIZE);
    static std::vector<bool> target_position_viz;
    static std::vector<Eigen::Vector3d> target_positions;
    target_position_viz.resize(GS_SIZE);
    target_positions.resize(GS_SIZE);
    for (int i = 0; i < GS_SIZE; i++) {
      target_position_viz[i] = true;
      target_positions[i] = {gait_sequence_msg_.reference_trajectory_position_x[i],
                             gait_sequence_msg_.reference_trajectory_position_y[i],
                             gait_sequence_msg_.reference_trajectory_position_z[i]};
    }
    target_trajectory_viz_->get_visible_input_port().FixValue(
        &target_trajectory_viz_->GetMyMutableContextFromRoot(context_.get()), target_position_viz);
    target_trajectory_viz_->get_position_input_port().FixValue(
        &target_trajectory_viz_->GetMyMutableContextFromRoot(context_.get()), target_positions);

    wbc_target_pos_contact_viz_->get_position_input_port().FixValue(
        &wbc_target_pos_contact_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_pos_targets));
    wbc_target_pos_contact_viz_->get_visible_input_port().FixValue(
        &wbc_target_pos_contact_viz_->GetMyMutableContextFromRoot(context_.get()),
        to_vector(wbc_target_msg_.feet_contacts));
    wbc_target_pos_no_contact_viz_->get_position_input_port().FixValue(
        &wbc_target_pos_no_contact_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_pos_targets));
    wbc_target_pos_no_contact_viz_->get_visible_input_port().FixValue(
        &wbc_target_pos_no_contact_viz_->GetMyMutableContextFromRoot(context_.get()),
        to_vector(invert(wbc_target_msg_.feet_contacts)));
    wbc_target_vel_viz_->get_vectors_origin_input_port().FixValue(
        &wbc_target_vel_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_pos_targets));
    wbc_target_vel_viz_->get_vectors_input_port().FixValue(
        &wbc_target_vel_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_vel_targets));

    wbc_target_force_viz_->get_vectors_origin_input_port().FixValue(
        &wbc_target_force_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_pos_targets));
    wbc_target_force_viz_->get_vectors_input_port().FixValue(
        &wbc_target_force_viz_->GetMyMutableContextFromRoot(context_.get()),
        rosMsgSeqToEigenSeq<std::vector<Eigen::Vector3d>>(wbc_target_msg_.feet_wrenches));

    plant->SetPositions(&plant->GetMyMutableContextFromRoot(context_.get()), quad_state);
    diagram_->ForcedPublish(*context_);
  };

  enum VisualizerTopic { JOINT_STATES, JOINT_CMD, LEG_JOINT_CMD, QUAD_STATE };

 private:
  double update_freq_ = 30.0;
  // Drake Variables
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double> *plant;
  geometry::SceneGraph<double> *scene_graph;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> context_;
  // Empty Visualization Config for visualizing inertia and contact geometries.
  visualization::VisualizationConfig visualization;
  std::shared_ptr<geometry::Meshcat> meshcat_;

  // Subscribers
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_sub_;
  rclcpp::Subscription<interfaces::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<interfaces::msg::JointCmd>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<interfaces::msg::LegCmd>::SharedPtr leg_cmd_sub_;
  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Internal Variables for Tracking State of the Class
  // Internal Pub/Sub variables for use in other functions in the class
  interfaces::msg::QuadState quad_state_;
  interfaces::msg::JointState joint_msg_;
  interfaces::msg::JointCmd joint_cmd_msg_;
  interfaces::msg::LegCmd leg_cmd_msg_;
  bool leg_cmd_recv_;
  VisualizerTopic visualizer_topic_;

  // drake vizualisers for leg_cmd message
  PositionsVizualiser *ee_pos_viz_;
  ForcesVizualiser *ee_force_viz_;

  // MPC related subscribers and visualizers
  PositionsVizualiser *foot_step_plan_viz_;
  rclcpp::Subscription<interfaces::msg::GaitSequence>::SharedPtr gait_sequence_sub_;
  interfaces::msg::GaitSequence gait_sequence_msg_;
  VectorsVizualiser *swing_leg_traj_viz_;
  rclcpp::Subscription<interfaces::msg::VectorSequence>::SharedPtr swing_leg_traj_sub_;
  interfaces::msg::VectorSequence swing_leg_traj_msg_;

  PositionsVizualiser *target_trajectory_viz_;
  rclcpp::Subscription<interfaces::msg::PositionSequence>::SharedPtr open_loop_trajectory_sub_;
  interfaces::msg::PositionSequence open_loop_trajectory_msg_;
  PositionsVizualiser *open_loop_trajectory_viz_;
  PosesVizualiser *open_loop_poses_viz_;

  rclcpp::Subscription<interfaces::msg::WBCTarget>::SharedPtr wbc_target_sub_;
  interfaces::msg::WBCTarget wbc_target_msg_;
  PositionsVizualiser *wbc_target_pos_contact_viz_;
  PositionsVizualiser *wbc_target_pos_no_contact_viz_;
  VectorsVizualiser *wbc_target_vel_viz_;
  VectorsVizualiser *wbc_target_force_viz_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<DrakeVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

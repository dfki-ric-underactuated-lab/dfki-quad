#pragma once

#define VIZ_IN_SIM

#include <Eigen/Core>
#include <vector>

#include "common/custom_qos.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/port_switch.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake_converter.hpp"
#include "interfaces/msg/contact_state.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/joint_state.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/msg/simulation_disturbance.hpp"
#include "interfaces/srv/reset_simulation.hpp"
#include "interfaces/srv/step_simulation.hpp"
#include "pd_changeable_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_publisher.hpp"
#include "ros_spinner.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"

/**
 * ROS2 Node which provides a drake based simulation.
 */
class DrakeSimulator {
 private:
  // Drake related members
  drake::systems::DiagramBuilder<double>* builder_;
  drake::multibody::MultibodyPlant<double>* plant_;
  drake::geometry::SceneGraph<double>* scene_graph_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::systems::Simulator<double>* simulator_;
  drake::systems::controllers::PdChangableController<double>* pid_controller_;
  drake::systems::Adder<double>* controller_adder_;
  drake::systems::Saturation<double>* actuator_limit_saturation_;

  // ROS 2 and Drake related members
  ROSPub<rosgraph_msgs::msg::Clock, 0>* clock_pub_;
  ROSPub<sensor_msgs::msg::Imu,
         0,
         std::vector<drake::math::RigidTransformd>,
         std::vector<drake::multibody::SpatialVelocity<double>>,
         std::vector<drake::multibody::SpatialAcceleration<double>>>* imu_pub_;
  ROSPub<interfaces::msg::ContactState, 0, std::array<Eigen::Vector3d, 4>>* contact_result_pub_;
  ROSPub<interfaces::msg::JointState, 3>* joint_state_pub_;
  ROSPub<interfaces::msg::QuadState,
         3,
         std::array<Eigen::Vector3d, 4>,
         std::vector<drake::math::RigidTransformd>,
         std::vector<drake::multibody::SpatialVelocity<double>>,
         std::vector<drake::multibody::SpatialAcceleration<double>>>* quad_state_pub_;

  DrakeConverter<drake::multibody::ContactResults<double>, std::array<Eigen::Vector3d, 4>>* contact_forces_calc_;

  rclcpp::Service<interfaces::srv::ResetSimulation>::SharedPtr reset_sim_service_;
  rclcpp::Service<interfaces::srv::StepSimulation>::SharedPtr step_sim_service_;

  bool restart_sim_;
  std::condition_variable restart_sim_cond_var_;
  std::mutex restart_sim_lock_;

  // ROS 2 node handle
  rclcpp::Node::SharedPtr node_;

 public:
  explicit DrakeSimulator();
  virtual ~DrakeSimulator();
  /**
   * Runs the simulation for a specific amount of seconds while sending and receving the required ROS messages
   * @param for_seconds
   */
  void run();
};
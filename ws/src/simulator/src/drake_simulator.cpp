#include "drake_simulator.hpp"

#include <drake/multibody/plant/externally_applied_spatial_force.h>

#include <common/sequence_containers.hpp>
#include <iostream>

DrakeSimulator::DrakeSimulator() : restart_sim_(true) {
  // Creates the ROS node
  node_ = std::make_shared<rclcpp::Node>("drake_simulator");
  // Creates the drake Diagram builder
  builder_ = new drake::systems::DiagramBuilder<double>;
  // Declares the ROS parameters in order to load them from a config file or via command line
  // Note that the first four are not statically typed as they dont have an initial value
  node_->declare_parameter("robot_urdf", rclcpp::ParameterType::PARAMETER_STRING);
  node_->declare_parameter("robot_foot_collision_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  node_->declare_parameter("world_urdf", rclcpp::ParameterType::PARAMETER_STRING);
  node_->declare_parameter("world_fix_link", rclcpp::ParameterType::PARAMETER_STRING);
  node_->declare_parameter<double>("simulator_realtime_rate", 0);
  node_->declare_parameter<double>("simulator_discretization_frequency", 1000);
  node_->declare_parameter<double>("imu_publish_frequency", 100);
  node_->declare_parameter<double>("joint_state_publish_frequency", 100);
  node_->declare_parameter<double>("sim_clock_publish_frequency", 1000);
  node_->declare_parameter<double>("command_receive_frequency", 1000);
  node_->declare_parameter<bool>("visualisation", false);
  node_->declare_parameter<double>("init_kp", 5.0);
  node_->declare_parameter<double>("init_kd", 0.0);
  node_->declare_parameter<bool>("respect_effort_limits", false);
  node_->declare_parameter<bool>("publish_quad_state", false);
  node_->declare_parameter<std::string>("reference_link", "base_link");
  node_->declare_parameter<std::string>("imu_link", "link_imu");
  node_->declare_parameter<bool>("manually_step_sim", false);
  node_->declare_parameter<double>("initial_robot_height", 0.3);
  node_->declare_parameter<std::vector<double>>("initial_joint_positions",
                                                {0,
                                                 1.3376901149749756,
                                                 -2.6728670597076416,
                                                 0,
                                                 1.3716193437576294,
                                                 -2.679778575897217,
                                                 0,
                                                 1.2717167139053345,
                                                 -2.6854333877563477,
                                                 0,
                                                 1.2823981046676636,
                                                 -2.7017698287963867});
  node_->declare_parameter<double>("visualisation_update_rate", 0.05);

  bool respect_effort_limits = node_->get_parameter("respect_effort_limits").as_bool();
  // rate of the simulator
  auto sim_rate = (1. / node_->get_parameter("simulator_discretization_frequency").as_double());
  // creating the drake plant and adding the quadroped as well as the world
  std::tie(plant_, scene_graph_) = drake::multibody::AddMultibodyPlantSceneGraph(builder_, sim_rate);
  auto parser = drake::multibody::Parser(plant_, scene_graph_);
  auto robot_model_idx = parser.AddModels(node_->get_parameter("robot_urdf").as_string())[0];
  parser.AddModels(node_->get_parameter("world_urdf").as_string());
  // fixing the worldbody to the actual drake world
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetBodyByName(node_->get_parameter("world_fix_link").as_string()).body_frame());
  // setting up the drake contact model (Taken from the ACROMONK sim)
  plant_->set_contact_model(drake::multibody::ContactModel::kHydroelastic);
  plant_->set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kLagged);
  plant_->Finalize();
  // Search for the foot collision ids by the names specified in the urdf and create LUT for fast indexing:
  auto robot_foot_collision_names = node_->get_parameter("robot_foot_collision_names").as_string_array();
  std::map<drake::geometry::GeometryId, unsigned int> robot_feet_collision_LUT;
  for (auto &geo_id : scene_graph_->model_inspector().GetAllGeometryIds()) {
    auto geo_name = scene_graph_->model_inspector().GetName(geo_id);
    auto found = std::find(robot_foot_collision_names.begin(), robot_foot_collision_names.end(), geo_name);
    if (found != robot_foot_collision_names.end()) {
      robot_feet_collision_LUT[geo_id] = found - robot_foot_collision_names.begin();
    }
  }
  RCLCPP_ERROR_EXPRESSION(node_->get_logger(),
                          robot_feet_collision_LUT.size() != robot_foot_collision_names.size(),
                          "Not all robot_foot_collision_names where found in URDF.");
  // Adding the quad_state publisher if enabled
  bool publish_quad_state = node_->get_parameter("publish_quad_state").as_bool();
  auto &reference_link = plant_->GetBodyByName(node_->get_parameter("reference_link").as_string());
  auto reference_link_idx = reference_link.index();
  if (publish_quad_state) {
    quad_state_pub_ = builder_->AddSystem<ROSPub<interfaces::msg::QuadState,
                                                 3,
                                                 std::array<Eigen::Vector3d, 4>,
                                                 std::vector<drake::math::RigidTransformd>,
                                                 std::vector<drake::multibody::SpatialVelocity<double>>,
                                                 std::vector<drake::multibody::SpatialAcceleration<double>>>>(
        1. / node_->get_parameter("joint_state_publish_frequency").as_double(),
        *node_,
        "quad_state",
        QOS_RELIABLE_NO_DEPTH,
        std::array<int, 3>{37, 12, 18},
        [robot_feet_collision_LUT, reference_link_idx](
            const std::array<Eigen::VectorXd, 3> &port_value,
            const std::array<const drake::AbstractValue *, 4> &abstract_port_value,
            const drake::systems::Context<double> &context,
            interfaces::msg::QuadState &ros_msg) {
          ros_msg.header.stamp =
              ROSPub<interfaces::msg::QuadState, 2, drake::multibody::ContactResults<double>>::GetRosTime(context);
          ros_msg.joint_state.header.stamp =
              ROSPub<interfaces::msg::QuadState, 2, drake::multibody::ContactResults<double>>::GetRosTime(context);

          auto rigid_transform =
              abstract_port_value[1]->get_value<std::vector<drake::math::RigidTransformd>>()[reference_link_idx];
          auto spatial_vel =
              abstract_port_value[2]
                  ->get_value<std::vector<drake::multibody::SpatialVelocity<double>>>()[reference_link_idx];
          auto spatial_acc =
              abstract_port_value[3]
                  ->get_value<std::vector<drake::multibody::SpatialAcceleration<double>>>()[reference_link_idx];

          ros_msg.pose.pose.position.x = rigid_transform.translation().x();
          ros_msg.pose.pose.position.y = rigid_transform.translation().y();
          ros_msg.pose.pose.position.z = rigid_transform.translation().z();
          ros_msg.pose.pose.orientation.w = rigid_transform.rotation().ToQuaternion().w();
          ros_msg.pose.pose.orientation.x = rigid_transform.rotation().ToQuaternion().x();
          ros_msg.pose.pose.orientation.y = rigid_transform.rotation().ToQuaternion().y();
          ros_msg.pose.pose.orientation.z = rigid_transform.rotation().ToQuaternion().z();
          ros_msg.twist.twist.linear.x = spatial_vel.translational().x();
          ros_msg.twist.twist.linear.y = spatial_vel.translational().y();
          ros_msg.twist.twist.linear.z = spatial_vel.translational().z();
          ros_msg.twist.twist.angular.x = spatial_vel.rotational().x();
          ros_msg.twist.twist.angular.y = spatial_vel.rotational().y();
          ros_msg.twist.twist.angular.z = spatial_vel.rotational().z();
          ros_msg.acceleration.linear.x = spatial_acc.translational().x();
          ros_msg.acceleration.linear.y = spatial_acc.translational().y();
          ros_msg.acceleration.linear.z = spatial_acc.translational().z();
          ros_msg.acceleration.angular.x = spatial_acc.rotational().x();
          ros_msg.acceleration.angular.y = spatial_acc.rotational().y();
          ros_msg.acceleration.angular.z = spatial_acc.rotational().z();

          Eigen::Vector<double, 12>::Map(ros_msg.joint_state.position.data()) = port_value[0](Eigen::seqN(7, 12));

          Eigen::Vector<double, 12>::Map(ros_msg.joint_state.velocity.data()) = port_value[0](Eigen::seqN(25, 12));

          Eigen::Vector<double, 12>::Map(ros_msg.joint_state.effort.data()) = port_value[1](Eigen::seqN(0, 12));

          Eigen::Vector<double, 12>::Map(ros_msg.joint_state.acceleration.data()) = port_value[2](Eigen::seqN(6, 12));
          // Creates them all the time new to init with 0
          const auto &contact_forces = abstract_port_value[0]->get_value<std::array<Eigen::Vector3d, 4>>();
          for (unsigned int i = 0; i < 4; i++) {
            ros_msg.foot_contact[i] = !contact_forces[i].isZero(std::numeric_limits<double>::epsilon());
            Eigen::Map<Eigen::Vector3d>(ros_msg.ground_contact_force.data() + 3 * i) = contact_forces[i];
          }
          // TODO: set belly contact correct
          ros_msg.belly_contact = false;
        });
  }
  // Calculating the contact forces
  contact_forces_calc_ =
      builder_->AddSystem<DrakeConverter<drake::multibody::ContactResults<double>, std::array<Eigen::Vector3d, 4>>>(
          [robot_feet_collision_LUT](const drake::multibody::ContactResults<double> &contact_result,
                                     std::array<Eigen::Vector3d, 4> &contact_forces) {
            for (unsigned int i = 0; i < 4; i++) {
              contact_forces[i].setZero();
            }
            for (int i = 0; i < contact_result.num_hydroelastic_contacts(); i++) {
              const auto &contact_inf = contact_result.hydroelastic_contact_info(i);
              if (robot_feet_collision_LUT.find(contact_inf.contact_surface().id_N())
                  != robot_feet_collision_LUT.end()) {
                // contacts[(robot_feet_collision_LUT.at(contact_inf.contact_surface().id_N()))] = true;
                contact_forces[(robot_feet_collision_LUT.at(contact_inf.contact_surface().id_N()))] =
                    contact_inf.F_Ac_W().translational();
              } else if (robot_feet_collision_LUT.find(contact_inf.contact_surface().id_M())
                         != robot_feet_collision_LUT.end()) {
                // contacts[(robot_feet_collision_LUT.at(contact_inf.contact_surface().id_M()))] = true;
                contact_forces[(robot_feet_collision_LUT.at(contact_inf.contact_surface().id_M()))] =
                    contact_inf.F_Ac_W().translational();
              }
            }
          });

  // Adding the contact result pub
  contact_result_pub_ = builder_->AddSystem<ROSPub<interfaces::msg::ContactState, 0, std::array<Eigen::Vector3d, 4>>>(
      1. / node_->get_parameter("imu_publish_frequency").as_double(),
      *node_,
      "contact_state",
      QOS_RELIABLE_NO_DEPTH,
      std::array<int, 0>{},
      [](const std::array<Eigen::VectorXd, 0> &port_value,
         const std::array<const drake::AbstractValue *, 1> &abstract_port_value,
         const drake::systems::Context<double> &context,
         interfaces::msg::ContactState &ros_msg) {
        (void)port_value;
        const auto &contact_forces = abstract_port_value[0]->get_value<std::array<Eigen::Vector3d, 4>>();
        for (unsigned int i = 0; i < 4; i++) {
          ros_msg.header.stamp = ROSPub<sensor_msgs::msg::Imu, 1>::GetRosTime(context);
          ros_msg.ground_contact_force[i] = contact_forces[i].norm();
        }
      });
  // Adding the IMU publisher
  int imu_link_idx = plant_->GetBodyByName(node_->get_parameter("imu_link").as_string()).index();
  imu_pub_ = builder_->AddSystem<ROSPub<sensor_msgs::msg::Imu,
                                        0,
                                        std::vector<drake::math::RigidTransformd>,
                                        std::vector<drake::multibody::SpatialVelocity<double>>,
                                        std::vector<drake::multibody::SpatialAcceleration<double>>>>(
      1. / node_->get_parameter("imu_publish_frequency").as_double(),
      *node_,
      "imu_measurement",
      QOS_RELIABLE_NO_DEPTH,
      std::array<int, 0>{},
      [imu_link_idx](const std::array<Eigen::VectorXd, 0> &port_value,
                     const std::array<const drake::AbstractValue *, 3> &abstract_port_value,
                     const drake::systems::Context<double> &context,
                     sensor_msgs::msg::Imu &ros_msg) {
        (void)port_value;  // Unused because no vector input
        ros_msg.header.stamp = ROSPub<sensor_msgs::msg::Imu, 1>::GetRosTime(context);
        auto pose = abstract_port_value[0]->get_value<std::vector<drake::math::RigidTransformd>>()[imu_link_idx];
        auto spatial_vel =
            abstract_port_value[1]->get_value<std::vector<drake::multibody::SpatialVelocity<double>>>()[imu_link_idx];
        auto spatial_acc = abstract_port_value[2]
                               ->get_value<std::vector<drake::multibody::SpatialAcceleration<double>>>()[imu_link_idx];

        auto ang_vel_unrot = pose.rotation().inverse() * spatial_vel.rotational();
        auto lin_acc_unrot = pose.rotation().inverse() * spatial_acc.translational();

        ros_msg.orientation.w = pose.rotation().ToQuaternion().w();
        ros_msg.orientation.x = pose.rotation().ToQuaternion().x();
        ros_msg.orientation.y = pose.rotation().ToQuaternion().y();
        ros_msg.orientation.z = pose.rotation().ToQuaternion().z();
        ros_msg.angular_velocity.x = ang_vel_unrot.x();
        ros_msg.angular_velocity.y = ang_vel_unrot.y();
        ros_msg.angular_velocity.z = ang_vel_unrot.z();
        ros_msg.linear_acceleration.x = lin_acc_unrot.x();
        ros_msg.linear_acceleration.y = lin_acc_unrot.y();
        ros_msg.linear_acceleration.z = lin_acc_unrot.z();
      });
  // Adding the joint state publisher
  joint_state_pub_ = builder_->AddSystem<ROSPub<interfaces::msg::JointState, 3>>(
      1. / node_->get_parameter("joint_state_publish_frequency").as_double(),
      *node_,
      "joint_states",
      QOS_RELIABLE_NO_DEPTH,
      std::array<int, 3>{37, 12, 18},
      [](const std::array<Eigen::VectorXd, 3> &port_value,
         const drake::systems::Context<double> &context,
         interfaces::msg::JointState &ros_msg) {
        ros_msg.header.stamp = ROSPub<sensor_msgs::msg::Imu, 2>::GetRosTime(context);

        Eigen::Vector<double, 12>::Map(ros_msg.position.data()) = port_value[0](Eigen::seqN(7, 12));

        Eigen::Vector<double, 12>::Map(ros_msg.velocity.data()) = port_value[0](Eigen::seqN(25, 12));

        Eigen::Vector<double, 12>::Map(ros_msg.effort.data()) = port_value[1](Eigen::seqN(0, 12));

        Eigen::Vector<double, 12>::Map(ros_msg.acceleration.data()) = port_value[2](Eigen::seqN(6, 12));
      });
  // Adding the clock publisher
  clock_pub_ = builder_->AddSystem<ROSPub<rosgraph_msgs::msg::Clock, 0>>(
      1. / node_->get_parameter("sim_clock_publish_frequency").as_double(),
      *node_,
      "/clock",
      QOS_RELIABLE_NO_DEPTH,
      std::array<int, 0>{},
      [](const std::array<Eigen::VectorXd, 0> &port_value,
         const drake::systems::Context<double> &context,
         rosgraph_msgs::msg::Clock &ros_msg) {
        (void)port_value;
        ros_msg.clock = ROSPub<rosgraph_msgs::msg::Clock, 0>::GetRosTime(context);
      });

  std::vector<double> inital_joint_positions = node_->get_parameter("initial_joint_positions").as_double_array();
  assert(inital_joint_positions.size() == 12);
  Eigen::VectorXd init_val(24);  // first 12 pose, second 12 vel
  init_val.setZero();
  init_val.block(0, 0, 12, 1) = Eigen::Map<Eigen::Matrix<double, 12, 1>>(inital_joint_positions.data());
  // Adds the ROS spinner and register subcribers for needed topics
  auto spinner = builder_->AddSystem<ROSSpin>(sim_rate, node_);
  auto &pos_port = spinner->DeclareSubscriberAndPort<interfaces::msg::JointCmd>(
      "joint_cmd",
      "pid_target",
      QOS_RELIABLE_NO_DEPTH,
      24,
      init_val,
      [](const interfaces::msg::JointCmd &joint_cmd, Eigen::VectorXd &output) {
        for (unsigned int i = 0; i < 12; i++) {
          output[i] = joint_cmd.position[i];
        }
        for (unsigned int i = 0; i < 12; i++) {
          output[i + 12] = joint_cmd.velocity[i];
        }
      });
  auto &eff_port = spinner->DeclareSubscriberAndPort<interfaces::msg::JointCmd>(
      "joint_cmd",
      "effort",
      QOS_RELIABLE_NO_DEPTH,
      12,
      Eigen::Vector<double, 12>::Zero(),
      [](const interfaces::msg::JointCmd &joint_cmd, Eigen::VectorXd &output) {
        for (unsigned int i = 0; i < 12; i++) {
          output[i] = joint_cmd.effort[i];
        }
      });
  auto &kp_port = spinner->DeclareSubscriberAndPort<interfaces::msg::JointCmd>(
      "joint_cmd",
      "kp",
      QOS_RELIABLE_NO_DEPTH,
      12,
      Eigen::Vector<double, 12>::Ones() * node_->get_parameter("init_kp").as_double(),
      [](const interfaces::msg::JointCmd &joint_cmd, Eigen::VectorXd &output) {
        for (unsigned int i = 0; i < 12; i++) {
          output[i] = joint_cmd.kp[i];
        }
      });
  auto &kd_port = spinner->DeclareSubscriberAndPort<interfaces::msg::JointCmd>(
      "joint_cmd",
      "kd",
      QOS_RELIABLE_NO_DEPTH,
      12,
      Eigen::Vector<double, 12>::Ones() * node_->get_parameter("init_kd").as_double(),
      [](const interfaces::msg::JointCmd &joint_cmd, Eigen::VectorXd &output) {
        for (unsigned int i = 0; i < 12; i++) {
          output[i] = joint_cmd.kd[i];
        }
      });

  std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>> Fext_init_vec(1);
  Fext_init_vec[0].body_index = reference_link_idx;
  Fext_init_vec[0].p_BoBq_B.setZero();
  Fext_init_vec[0].F_Bq_W.SetZero();

  auto &disturbance_port =
      spinner->DeclareSubscriberAndPort<interfaces::msg::SimulationDisturbance,
                                        std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(
          "simulation_disturbance",
          "disturbance",
          QOS_RELIABLE_NO_DEPTH,
          Fext_init_vec,
          [reference_link_idx](const auto &sim_dist, auto &output) {
            drake::multibody::ExternallyAppliedSpatialForce<double> Fext;
            Fext.body_index = reference_link_idx;
            Eigen::Vector3d f, tau;
            for (int i = 0; i < 3; ++i) {
              f(i) = sim_dist.force[i];
              tau(i) = sim_dist.tau[i];
            }

            Fext.F_Bq_W = drake::multibody::SpatialForce<double>(tau, f);

            output[0] = Fext;
          });
  // Creating state projectin matrix and PID controller to control position and velocity commands coming from
  // ROS
  int i = 0;
  for (auto name : plant_->GetStateNames()) {
    std::cout << "(" << i++ << ") " << name << ", ";
  }
  std::cout << std::endl;

  Eigen::Matrix<double, 24, 37> state_project_pid;
  state_project_pid.setZero();
  state_project_pid(Eigen::seqN(0, 12), Eigen::seqN(7, 12)).setIdentity();
  state_project_pid(Eigen::seqN(12, 12), Eigen::seqN(25, 12)).setIdentity();
  // Creates and added which is used to combine the output of the PID controller and effort commands that are
  // coming from ROS
  controller_adder_ = builder_->AddSystem<drake::systems::Adder<double>>(2, 12);
  if (respect_effort_limits) {
    actuator_limit_saturation_ = builder_->AddSystem<drake::systems::Saturation<double>>(
        plant_->GetEffortLowerLimits(), plant_->GetEffortUpperLimits());
  }
  pid_controller_ = builder_->AddSystem<drake::systems::controllers::PdChangableController<double>>(state_project_pid);
  // Connects everything
  builder_->Connect(plant_->get_body_poses_output_port(), imu_pub_->get_input_port(0));
  builder_->Connect(plant_->get_body_spatial_velocities_output_port(), imu_pub_->get_input_port(1));
  builder_->Connect(plant_->get_body_spatial_accelerations_output_port(), imu_pub_->get_input_port(2));

  builder_->Connect(disturbance_port, plant_->get_applied_spatial_force_input_port());

  builder_->Connect(plant_->get_state_output_port(), joint_state_pub_->get_input_port(0));
  builder_->Connect(plant_->get_generalized_acceleration_output_port(robot_model_idx),
                    joint_state_pub_->get_input_port(2));
  builder_->Connect(pid_controller_->get_output_port_control(), controller_adder_->get_input_port(0));
  if (respect_effort_limits) {
    builder_->Connect(controller_adder_->get_output_port(), actuator_limit_saturation_->get_input_port());
    builder_->Connect(actuator_limit_saturation_->get_output_port(), plant_->get_actuation_input_port());
    builder_->Connect(actuator_limit_saturation_->get_output_port(), joint_state_pub_->get_input_port(1));
  } else {
    builder_->Connect(controller_adder_->get_output_port(), plant_->get_actuation_input_port());
    builder_->Connect(controller_adder_->get_output_port(), joint_state_pub_->get_input_port(1));
  }
  builder_->Connect(plant_->get_state_output_port(), pid_controller_->get_input_port_estimated_state());
  builder_->Connect(pos_port, pid_controller_->get_input_port_desired_state());
  builder_->Connect(kp_port, pid_controller_->get_input_port_kp());
  builder_->Connect(kd_port, pid_controller_->get_input_port_kd());
  builder_->Connect(eff_port, controller_adder_->get_input_port(1));
  builder_->Connect(plant_->get_contact_results_output_port(), contact_forces_calc_->get_input_port());
  builder_->Connect(contact_forces_calc_->get_output_port(), contact_result_pub_->get_input_port());
  if (publish_quad_state) {
    builder_->Connect(plant_->get_state_output_port(), quad_state_pub_->get_input_port(0));
    builder_->Connect(contact_forces_calc_->get_output_port(), quad_state_pub_->get_input_port(3));
    builder_->Connect(plant_->get_body_poses_output_port(), quad_state_pub_->get_input_port(4));
    builder_->Connect(plant_->get_body_spatial_velocities_output_port(), quad_state_pub_->get_input_port(5));
    builder_->Connect(plant_->get_body_spatial_accelerations_output_port(), quad_state_pub_->get_input_port(6));
    builder_->Connect(plant_->get_generalized_acceleration_output_port(robot_model_idx),
                      quad_state_pub_->get_input_port(2));
    if (respect_effort_limits) {
      builder_->Connect(actuator_limit_saturation_->get_output_port(0), quad_state_pub_->get_input_port(1));
    } else {
      builder_->Connect(controller_adder_->get_output_port(0), quad_state_pub_->get_input_port(1));
    }
  }

  // If visualitation required, a meshcat instance will be created
  if (node_->get_parameter("visualisation").as_bool()) {
    meshcat_ = std::make_shared<drake::geometry::Meshcat>();

    drake::visualization::VisualizationConfig viz_config;
    viz_config.publish_period = node_->get_parameter("visualisation_update_rate").as_double();
    drake::visualization::ApplyVisualizationConfig(viz_config, builder_, nullptr, nullptr, nullptr, meshcat_);
  }
  // builds the whole mutlibody system diagram and hands it over to the drake simulator
  diagram_ = builder_->Build();
  auto init_context = diagram_->CreateDefaultContext();
  // Sets a spawb location for the Quadroped

  Eigen::VectorXd init_pose(19);
  init_pose.setZero();
  init_pose(0) = 1.0;  // Quaternion W
  init_pose(6) = node_->get_parameter("initial_robot_height").as_double();
  init_pose.block(7, 0, 12, 1) = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(inital_joint_positions.data());

  plant_->SetPositions(&plant_->GetMyMutableContextFromRoot(init_context.get()), init_pose);
  auto com =
      plant_->get_body(reference_link_idx).CalcCenterOfMassInBodyFrame(plant_->GetMyContextFromRoot(*init_context));
  RCLCPP_INFO(node_->get_logger(), "COM in body frame: [%f, %f, %f]", com.x(), com.y(), com.z());

  const drake::multibody::SpatialInertia<double> I =
      plant_->CalcSpatialInertia(plant_->GetMyContextFromRoot(*init_context),
                                 plant_->GetFrameByName("base_link"),
                                 plant_->GetBodyIndices(robot_model_idx));

  auto base_inertia = I.CalcRotationalInertia().CopyToFullMatrix3();
  auto base_mass = I.get_mass();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Inertia " << base_inertia);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Mass  " << base_mass);
  for (int jt = 0; jt < plant_->num_joints(); jt++) {
    RCLCPP_INFO(
        node_->get_logger(), "Joint [%d]: %s", jt, plant_->get_joint(drake::multibody::JointIndex(jt)).name().c_str());
  }

  simulator_ = new drake::systems::Simulator<double>(*diagram_, std::move(init_context));
  // Sets the desired realtime rate
  simulator_->set_target_realtime_rate(node_->get_parameter("simulator_realtime_rate").as_double());
  // Add sim reset callback
  bool manually_step_sim = node_->get_parameter("manually_step_sim").as_bool();
  reset_sim_service_ = node_->create_service<interfaces::srv::ResetSimulation>(
      "reset_sim",
      [this, spinner, manually_step_sim, inital_joint_positions](
          const std::shared_ptr<interfaces::srv::ResetSimulation::Request> request,
          const std::shared_ptr<interfaces::srv::ResetSimulation::Response> response) {
        auto new_context = diagram_->CreateDefaultContext();
        spinner->StopSimAsync();
        std::unique_lock lock(restart_sim_lock_);
        if (manually_step_sim) {
          simulator_->AdvanceTo(simulator_->get_context().get_time()
                                + 4.0);  // In manually stepping we run the sim to end
        } else {                         // If auto sim, than we have to wait for another thread stopping the sim
          restart_sim_cond_var_.wait(lock, [this] { return !restart_sim_; });
        }

        Eigen::VectorXd init_pose(19);
        init_pose.setZero();
        init_pose(0) = request->pose.orientation.w;
        init_pose(1) = request->pose.orientation.x;
        init_pose(2) = request->pose.orientation.y;
        init_pose(3) = request->pose.orientation.z;
        init_pose(4) = request->pose.position.x;
        init_pose(5) = request->pose.position.y;
        init_pose(6) = request->pose.position.z;

        if (request->joint_positions.size() == 12) {
          RCLCPP_INFO(node_->get_logger(), "Joint positions will be reset to requested values");
          init_pose.block(7, 0, 12, 1) =
              Eigen::Map<const Eigen::Matrix<double, 12, 1>>(request->joint_positions.data());
        } else {
          RCLCPP_INFO(node_->get_logger(),
                      "No or the wrong number of joint positions specified, joint positions will be reset to "
                      "default values");
          init_pose.block(7, 0, 12, 1) = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(inital_joint_positions.data());
        }
        plant_->SetPositions(&plant_->GetMyMutableContextFromRoot(new_context.get()), init_pose);

        Eigen::VectorXd init_values(24);  // first 12 are positions, second 12 are velocity
        init_values.setZero();
        init_values.block(0, 0, 12, 1) = init_pose.block(7, 0, 12, 1);
        spinner->SetDiscreteInitialPortValue(0, init_values);
        spinner->Reset();
        simulator_->reset_context(std::move(new_context));
        response->success = true;
        RCLCPP_INFO(node_->get_logger(), "Simulation reset was performed");
        restart_sim_ = true;
        lock.unlock();
        restart_sim_cond_var_.notify_all();
      });

  if (node_->get_parameter("manually_step_sim").as_bool()) {
    step_sim_service_ = node_->create_service<interfaces::srv::StepSimulation>(
        "step_sim",
        [this](const std::shared_ptr<interfaces::srv::StepSimulation::Request> request,
               const std::shared_ptr<interfaces::srv::StepSimulation::Response> response) {
          simulator_->AdvanceTo(simulator_->get_context().get_time() + request->dt);
          response->time = simulator_->get_context().get_time();
        });
  }
};

DrakeSimulator::~DrakeSimulator() {
  delete clock_pub_;
  delete imu_pub_;
  delete joint_state_pub_;
};

void DrakeSimulator::run() {
  if (node_->get_parameter("manually_step_sim").as_bool()) {
    while (rclcpp::ok()) {
      rclcpp::spin(node_);
    }
  } else {
    std::thread reset_thread([this]() {
      while (rclcpp::ok()) {
        rclcpp::spin(node_);
      }
    });
    while (rclcpp::ok()) {
      auto ret = simulator_->AdvanceTo(simulator_->get_context().get_time() + 4.0);
      if (ret.reason() == drake::systems::SimulatorStatus::kReachedTerminationCondition) {  // sim reset
        restart_sim_lock_.lock();
        restart_sim_ = false;
        restart_sim_lock_.unlock();
        restart_sim_cond_var_.notify_all();
        std::unique_lock lock(restart_sim_lock_);
        restart_sim_cond_var_.wait(lock, [this] { return restart_sim_; });
      }
    }
    reset_thread.join();
  }
  rclcpp::shutdown();
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  {
    DrakeSimulator sim;
    do {
      sim.run();
    } while (rclcpp::ok());
  }
};
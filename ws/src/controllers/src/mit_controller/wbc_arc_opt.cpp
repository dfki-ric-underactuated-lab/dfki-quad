#include "mit_controller/wbc_arc_opt.hpp"

#include <wbc/core/PluginLoader.hpp>

#include "mit_controller/flush_denormals.hpp"

WBCArcOPT::WBCArcOPT(std::unique_ptr<StateInterface> state,
                     std::string solver,
                     std::string scene,
                     const std::string& model_urdf,
                     const std::array<std::string, ModelInterface::N_LEGS>& feet_names,
                     const std::array<std::string, ModelInterface::NUM_JOINTS>& joint_names,
                     double mu,
                     const Eigen::Matrix<double, 6, 1>& com_pose_weight,
                     const Eigen::Matrix<double, 3, 1>& foot_pose_weight,
                     const Eigen::Matrix<double, 3, 1>& foot_force_weight,
                     const Eigen::Matrix<double, 6, 1>& com_pose_p_gain,
                     const Eigen::Matrix<double, 6, 1>& com_pose_d_gain,
                     const Eigen::Matrix<double, 3, 1>& feet_pose_p_gain,
                     const Eigen::Matrix<double, 3, 1>& feet_pose_d_gain,
                     const Eigen::Matrix<double, 6, 1>& com_pose_saturation,
                     const Eigen::Matrix<double, 3, 1>& feet_pose_saturation,
                     double solver_tolerances,
                     const std::string& hpipm_mode,
                     double acceleration_penalty,
                     double contact_wrench_penalty,
                     bool flush_denormals)
    : quad_state_(std::move(state)),
      feet_names_(feet_names),
      wbc_robot_model_(std::make_shared<wbc::RobotModelPinocchio>()),
      active_foot_contacts_(ModelInterface::N_LEGS),
      last_wbc_model_update_time_(0.0),
      flush_denormals_(flush_denormals) {
  joint_cmd_.clear();
  // Configure robot model
  wbc::RobotModelConfig model_config;
  model_config.file_or_string = model_urdf;
  model_config.floating_base = true;
  std::cout << "Loading URDF from " << model_urdf << std::endl;
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
    active_foot_contacts_[feet_idx].frame_id = feet_names_[feet_idx];
    active_foot_contacts_[feet_idx].active = quad_state_->GetFeetContacts()[feet_idx];
    active_foot_contacts_[feet_idx].mu = mu;
  }
  model_config.contact_points = active_foot_contacts_;
  auto config_ret = wbc_robot_model_->configure(model_config);
  if (!config_ret) {
    throw std::runtime_error("Error in configuring wbc_robot_model");
  }

  // Solver: loaded as plugin by name, its options are set through the generic QPSolver interface (as in hyper_2)
  std::cout << "Configuring wbc solver: " << solver << std::endl;
  if (!wbc::PluginLoader::loadPlugin("libwbc-solvers-" + solver + ".so")) {
    const char* dl_error = dlerror();
    throw std::runtime_error(fmt::format(
        "Could not load wbc solver plugin libwbc-solvers-{0}.so: {1}", solver, dl_error ? dl_error : "unknown error"));
  }
  wbc_solver_ = std::shared_ptr<wbc::QPSolver>(wbc::QPSolverFactory::createInstance(solver));
  if (!wbc_solver_) {
    throw std::runtime_error(fmt::format("Unknown wbc solver {0}", solver));
  }
  if (solver == "hpipm") {
    std::cout << "Set wbc solver mode to " << hpipm_mode << std::endl;
    wbc_solver_->setMode(hpipm_mode);
  }
  if (solver_tolerances > 0.0) {
    if (wbc_solver_->setTolerance(solver_tolerances)) {
      std::cout << "Set wbc solver tolerance to " << solver_tolerances << std::endl;
    } else {
      std::cout << "WBC solver " << solver << " has no tolerance setting, ignoring wbc_solver_tolerances = "
                << solver_tolerances << std::endl;
    }
  }

  // Setup scene
  std::cout << "Set wbc acceleration penalty to " << acceleration_penalty << ", contact wrench penalty to "
            << contact_wrench_penalty << std::endl;
  if (scene == "AccelerationSceneReducedTSID") {
    auto reduced_scene =
        std::make_unique<wbc::AccelerationSceneReducedTSID>(wbc_robot_model_, wbc_solver_, WBC_CYCLE_DT);
    reduced_scene->setAccelerationPenalty(acceleration_penalty);
    reduced_scene->setContactWrenchPenalty(contact_wrench_penalty);
    wbc_scene_ = std::move(reduced_scene);
  } else if (scene == "AccelerationSceneTSID") {
    auto tsid_scene = std::make_unique<wbc::AccelerationSceneTSID>(wbc_robot_model_, wbc_solver_, WBC_CYCLE_DT);
    tsid_scene->setAccelerationPenalty(acceleration_penalty);
    tsid_scene->setContactWrenchPenalty(contact_wrench_penalty);
    wbc_scene_ = std::move(tsid_scene);
  } else {
    throw std::runtime_error(fmt::format("Unknown wbc scene {0}", scene));
  }

  // Configure tasks
  std::vector<wbc::TaskPtr> wbc_tasks;
#ifdef COM_TASK
  wbc_com_task_ = std::make_shared<wbc::SpatialAccelerationTask>(
      wbc::TaskConfig("body_pose",
                      0,
                      com_pose_weight,  // vector: xyz, roll pitch yaw
                      1),
      wbc_robot_model_,
      "base_link");  // reference in world frame
  const auto& com_weights = wbc_com_task_->weights;
  fmt::print("com weights: {} \n", fmt::join(com_weights.data(), com_weights.data() + com_weights.size(), ", "));
  wbc_tasks.push_back(wbc_com_task_);
#else
  std::cout << "COM_TASK is deactivated" << std::endl;
  (void)com_pose_weight;
#endif

  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
#ifdef FEET_FORCE_TASK
    wbc_foot_contact_tasks_[feet_idx] =
        std::make_shared<wbc::ContactForceTask>(wbc::TaskConfig(fmt::format("foot_contact_{}", feet_idx),
                                                                0,
                                                                foot_force_weight,  // 3: force
                                                                quad_state_->GetFeetContacts()[feet_idx]),
                                                wbc_robot_model_);  // reference in world frame
    wbc_tasks.push_back(wbc_foot_contact_tasks_[feet_idx]);
#else
    std::cout << "FEET_FORCE_TASK is deactivated" << std::endl;
    (void)foot_force_weight;
#endif
#ifdef FEET_POS_TASK
    wbc_foot_pose_tasks_[feet_idx] = std::make_shared<wbc::SpatialAccelerationTask>(
        wbc::TaskConfig(fmt::format("foot_pose_{}", feet_idx),
                        0,
                        stack(foot_pose_weight, Eigen::Vector3d::Zero().eval()),  // 6 dof vector
                        !quad_state_->GetFeetContacts()[feet_idx]),
        wbc_robot_model_,
        feet_names[feet_idx]);  // reference in world frame
    wbc_tasks.push_back(wbc_foot_pose_tasks_[feet_idx]);
#else
    std::cout << "FEET_POS_TASK is deactivated" << std::endl;
    (void)foot_pose_weight;
#endif
  }

  config_ret = wbc_scene_->configure(wbc_tasks);
  if (!config_ret) {
    throw std::runtime_error("Error in configuring wbc_scene");
  }

  // Cartesian PD-Controllers
#ifdef COM_TASK
  com_cart_pd_controller_.setPGain(com_pose_p_gain);  // position, orientation (6)
  com_cart_pd_controller_.setDGain(com_pose_d_gain);
  com_cart_pd_controller_.setMaxCtrlOutput(com_pose_saturation);
#else
  (void)com_pose_p_gain;
  (void)com_pose_d_gain;
#endif
#ifdef FEET_POS_TASK
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
    feet_cart_pd_controllers_[feet_idx].setPGain(stack(feet_pose_p_gain, Eigen::Vector3d::Zero().eval()));
    feet_cart_pd_controllers_[feet_idx].setDGain(stack(feet_pose_d_gain, Eigen::Vector3d::Zero().eval()));
    feet_cart_pd_controllers_[feet_idx].setMaxCtrlOutput(stack(feet_pose_saturation, Eigen::Vector3d::Zero().eval()));
  }
#else
  (void)feet_pose_p_gain;
  (void)feet_pose_d_gain;
#endif

  // Setup states
  joint_state_.resize(ModelInterface::NUM_JOINTS);
  auto wbc_joint_names = wbc_robot_model_->jointNames();
  assert(wbc_joint_names.size() == ModelInterface::NUM_JOINTS);

  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (unsigned int joint_idx = 0; joint_idx < ModelInterface::N_JOINTS_PER_LEG; joint_idx++) {
      unsigned int flat_idx = leg_idx * ModelInterface::N_JOINTS_PER_LEG + joint_idx;
      auto found_it = std::find(wbc_joint_names.begin(), wbc_joint_names.end(), joint_names[flat_idx]);
      if (found_it == wbc_joint_names.end()) {
        throw std::runtime_error(fmt::format("Joint %s cant be found in URDF", joint_names[flat_idx]));
      }
      joint_idxs_[leg_idx][joint_idx] = std::distance(wbc_joint_names.begin(), found_it);

      std::cout << "joint [" << leg_idx << ", " << joint_idx << "] " << joint_names[flat_idx] << " ["
                << joint_idxs_[leg_idx][joint_idx] << "]" << std::endl;
    }
  }
}

void WBCArcOPT::UpdateTarget(const Eigen::Quaterniond& orientation,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& lin_vel,
                             const Eigen::Vector3d& ang_vel) {
  com_target_.pose.orientation = orientation;
  com_target_.pose.position = position;
  com_target_.twist.linear = lin_vel;
  com_target_.twist.angular = ang_vel;
  com_target_.acceleration.setZero();
}

void WBCArcOPT::UpdateFootContact(const WBCInterface<JointTorqueCommands>::FootContact& foot_contact) {
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
#ifdef FEET_FORCE_TASK
    wbc_foot_contact_tasks_[feet_idx]->setActivation(foot_contact[feet_idx]);
#endif
#ifdef FEET_POS_TASK
    wbc_foot_pose_tasks_[feet_idx]->setActivation(!foot_contact[feet_idx]);
#endif
    active_foot_contacts_[feet_idx].active = foot_contact[feet_idx];
  }
  wbc_robot_model_->setContacts(active_foot_contacts_);  // TODO: fasten this up via getActiveContacts
}

void WBCArcOPT::UpdateWrenches(const WBCInterface<JointTorqueCommands>::Wrenches& wrenches) {
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
    wbc::types::Wrench target_wrench;
    target_wrench.force = wrenches[feet_idx];
    target_wrench.torque.setZero();
#ifdef FEET_FORCE_TASK
    wbc_foot_contact_tasks_[feet_idx]->setReference(target_wrench);
#endif
  }
}

void WBCArcOPT::UpdateFeetTarget(const FeetTargets& feet_targets) {
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
    feet_targets_[feet_idx].pose.position = feet_targets.positions[feet_idx];
    feet_targets_[feet_idx].pose.orientation.setIdentity();
    feet_targets_[feet_idx].twist.linear = feet_targets.velocities[feet_idx];
    feet_targets_[feet_idx].twist.angular.setZero();
    feet_targets_[feet_idx].acceleration.linear = feet_targets.accelerations[feet_idx];
    feet_targets_[feet_idx].acceleration.angular.setZero();
  }
}

void WBCArcOPT::UpdateState(const StateInterface& quad_state) {
  // Floating base state:
  floating_base_state_.pose.position = quad_state.GetPositionInWorld();
  floating_base_state_.pose.orientation = quad_state.GetOrientationInWorld();
  floating_base_state_.twist.linear = quad_state.GetLinearVelInWorld();
  floating_base_state_.twist.angular = quad_state.GetAngularVelInWorld();
  floating_base_state_.acceleration.linear.setZero();
  floating_base_state_.acceleration.angular.setZero();
  // floating_base_state_.frame_id = "world";  // TODO: might be not important

  // Joint State
  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (unsigned int joint_idx = 0; joint_idx < ModelInterface::N_JOINTS_PER_LEG; joint_idx++) {
      joint_state_.position[joint_idxs_[leg_idx][joint_idx]] = quad_state.GetJointPositions()[leg_idx][joint_idx];
      joint_state_.velocity[joint_idxs_[leg_idx][joint_idx]] = quad_state.GetJointVelocities()[leg_idx][joint_idx];
      joint_state_.acceleration[joint_idxs_[leg_idx][joint_idx]] = 0;  // TODO: this stays 0?
      // TODO: what happend to effort?
    }
  }
  auto wbc_model_update_tic = std::chrono::high_resolution_clock::now();
  wbc_robot_model_->update(joint_state_.position,
                           joint_state_.velocity,
                           joint_state_.acceleration,
                           floating_base_state_.pose,
                           floating_base_state_.twist,
                           floating_base_state_.acceleration);
  auto wbc_model_update_toc = std::chrono::high_resolution_clock::now();
  last_wbc_model_update_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(wbc_model_update_toc - wbc_model_update_tic).count();
}

void WBCArcOPT::UpdateModel(const ModelInterface& quad_model) {
  pinocchio::Model* wbc_model_temp =
      std::static_pointer_cast<wbc::RobotModelPinocchio>(wbc_robot_model_)->getInternalModel();
  uint idx = wbc_model_temp->getFrameId("base_link");
  assert(idx != wbc_model_temp->frames.size());

  wbc_model_temp->inertias[wbc_model_temp->frames[idx].parent].mass() = quad_model.getBaseMass();
  wbc_model_temp->inertias[wbc_model_temp->frames[idx].parent].lever() = quad_model.getBaseCOM();
  wbc_model_temp->inertias[wbc_model_temp->frames[idx].parent].inertia() =
      pinocchio::Symmetric3(quad_model.getBaseInertia());
}

WBCReturn WBCArcOPT::GetJointCommand(JointTorqueVelocityPositionCommands& joint_command) {
  bool success = true;
  // Update PD controllers
  const auto& body_state = floating_base_state_;
  auto body_pose_reference_out = com_cart_pd_controller_.update(
      com_target_.pose, com_target_.twist, com_target_.acceleration, body_state.pose, body_state.twist);
#ifdef COM_TASK
  wbc_com_task_->setReference(body_pose_reference_out);
#endif
//  std::cout << "com_target_                 " << com_target_.pose.position.transpose() << std::endl;
//  std::cout << "body_state_                 " << body_state.pose.position.transpose() << std::endl;
//  std::cout << "com_target_ - body_state_   " << com_cart_pd_controller_.getControlError().transpose() << std::endl;
//  std::cout << "com_target_ - body_state_ (vel)  " << (com_target_.twist.linear - body_state.twist.linear).transpose()
//            << std::endl;
//  std::cout << "com_target_ - body_state_ (ang)  " << (com_target_.twist.angular -
//  body_state.twist.angular).transpose()
//            << std::endl;
//  std::cout << "body_pose_reference_out lin " << body_pose_reference_out.acceleration.linear.transpose() << std::endl;
//  std::cout << "body_pose_reference_out ang " << body_pose_reference_out.acceleration.angular.transpose() <<
//  std::endl;
#ifdef FEET_POS_TASK
  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
    const auto& target = feet_targets_[feet_idx];
    const auto& reference_out = feet_cart_pd_controllers_[feet_idx].update(
        target.pose,
        target.twist,
        target.acceleration,
        wbc_robot_model_->pose(feet_names_[feet_idx]),  // TODO: or get them from somewhere else?
        wbc_robot_model_->twist(feet_names_[feet_idx]));

    //    if (reference_out.acceleration.linear[2] > 10.0) {
    //      std::cout << "control state " << feet_idx << " - " << state.pose.position.transpose() << std::endl;
    //      std::cout << "control arget " << feet_idx << " - " << target.pose.position.transpose() << std::endl;
    //      std::cout << "ceference_out " << feet_idx << " - " << reference_out.acceleration.linear.transpose() <<
    //      std::endl;
    //    }
    wbc_foot_pose_tasks_[feet_idx]->setReference(reference_out);
  }
#endif
  auto wbc_update_tic = std::chrono::high_resolution_clock::now();
  auto& qp = wbc_scene_->update();
  auto wbc_update_toc = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point wbc_solve_tic, wbc_solve_toc;
  try {
    wbc_solve_tic = std::chrono::high_resolution_clock::now();
    // only the solver call runs with subnormal numbers flushed to zero (wbc.arc_opt.flush_denormals)
    ScopedFlushDenormals flush(flush_denormals_);
    joint_cmd_ = wbc_scene_->solve(qp);
  } catch (const std::runtime_error& err) {
    std::cout << " ARC-OPT solver failed: " << err.what() << std::endl;
    std::cout << " Setting zero torque as cmd " << err.what() << std::endl;
    joint_cmd_.resize(joint_state_.position.size());
    for (unsigned int i = 0; i < joint_cmd_.position.size(); i++) {
      joint_cmd_.effort[i] = 0.0;  // TOOD: very small value to have correct control mode
      joint_cmd_.acceleration[i] = 0.0;
      joint_cmd_.velocity[i] = wbc::unset;
      joint_cmd_.position[i] = wbc::unset;
    }
    success = false;
  }
  wbc_solve_toc = std::chrono::high_resolution_clock::now();

  //
  //  wbc_scene_->updateTasksStatus();
  //  for (unsigned int feet_idx = 0; feet_idx < ModelInterface::N_LEGS; feet_idx++) {
  //    if (wbc_scene_->getTask(fmt::format("foot_pose_{}", feet_idx))->activation) {
  //      std::cout << "contact task: " << feet_idx
  //                << " is: " << wbc_scene_->getTask(fmt::format("foot_pose_{}", feet_idx))->activation << std::endl;
  //      std::cout << "\ty\t\t\t"
  //                << wbc_scene_->getTasksStatus().getElementByName(fmt::format("foot_pose_{}",
  //                feet_idx)).y.transpose()
  //                << std::endl;
  //      std::cout
  //          << "\ty_ref\t\t\t"
  //          << wbc_scene_->getTasksStatus().getElementByName(fmt::format("foot_pose_{}",
  //          feet_idx)).y_ref.transpose()
  //          << std::endl;
  //      std::cout
  //          << "\ty_solution\t\t\t"
  //          << wbc_scene_->getTasksStatus().getElementByName(fmt::format("foot_pose_{}",
  //          feet_idx)).y_solution.transpose()
  //          << std::endl;
  //    }
  //  }

  // joint_integrator_.integrate(joint_state_, joint_cmd_, WBC_CYCLE_DT);

  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (unsigned int joint_idx = 0; joint_idx < ModelInterface::N_JOINTS_PER_LEG; joint_idx++) {
      joint_command.torque[leg_idx][joint_idx] = joint_cmd_.effort[joint_idxs_[leg_idx][joint_idx]];
      joint_command.velocity[leg_idx][joint_idx] =
          joint_state_.velocity[joint_idxs_[leg_idx][joint_idx]]
          + joint_cmd_.acceleration[joint_idxs_[leg_idx][joint_idx]] * WBC_CYCLE_DT;
      joint_command.position[leg_idx][joint_idx] = joint_state_.position[joint_idxs_[leg_idx][joint_idx]]
                                                   + joint_command.velocity[leg_idx][joint_idx] * WBC_CYCLE_DT;
      // TODO: THIS DT to use where?
    }
  }

  return {
      success,
      std::chrono::duration_cast<std::chrono::duration<double>>(wbc_update_toc - wbc_update_tic).count()
          + last_wbc_model_update_time_,
      std::chrono::duration_cast<std::chrono::duration<double>>(wbc_solve_toc - wbc_solve_tic).count(),
  };
}

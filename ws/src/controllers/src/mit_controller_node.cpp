#include "mit_controller_node.hpp"

MITController::MITController(const std::string &nodeName)
    : Node(nodeName), last_gait_sequence_mode_(GaitSequence::KEEP), first_quad_state_received_(false) {
  this->declare_parameter("initial_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("slc_swing_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("wbc.inverse_dynamics.transformation_filter_size", 20);
  this->declare_parameter("slc_world_blend", 1.0);
  this->declare_parameter("mpc_alpha", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_state_weights_stand", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mpc_state_weights_move", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mpc_mu", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_fmin", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("mpc_fmax", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("gs_shoulder_positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("leg_control_mode", rclcpp::ParameterType::PARAMETER_INTEGER);
  this->declare_parameter("raibert.k", 0.03);
  this->declare_parameter("raibert.z_on_plane", false);
  this->declare_parameter("raibert.filtersize", 20);
  this->declare_parameter("fix_standing_position", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("fix_position_distance_threshold", 0.1);
  this->declare_parameter("fix_position_angular_threshold", 0.26);
  this->declare_parameter("fix_position_velocity_threshold", 0.1);
  this->declare_parameter("maximum_swing_leg_progress_to_update_target", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("early_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("late_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("lost_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("late_contact_reschedule_swing_phase", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.foot_position_based_on_target_height",
                          rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.foot_position_based_on_target_orientation",
                          rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("wbc.inverse_dynamics.target_velocity_blend", 0.0);
  this->declare_parameter("wbc.arc_opt.model_urdf", rclcpp::ParameterType::PARAMETER_STRING);
  this->declare_parameter("wbc.arc_opt.feet_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  this->declare_parameter("wbc.arc_opt.joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  this->declare_parameter("wbc.arc_opt.mu", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("wbc.arc_opt.com_pose_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.com_pose_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.com_pose_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.foot_pose_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.foot_force_weight", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.feet_pose_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("wbc.arc_opt.feet_pose_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter<std::vector<double>>("wbc.arc_opt.com_pose_saturation",
                                               {std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max(),
                                                std::numeric_limits<double>::max()});
  this->declare_parameter<std::vector<double>>(
      "wbc.arc_opt.feet_pose_saturation",
      {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});

  this->declare_parameter("use_model_adaptation", false);
  this->declare_parameter("controller_heartbeat_dt", 0.5);
  this->declare_parameter<std::vector<double>>("model_parameter_weights",
                                               {1.0, 0.7, 0.7, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  this->declare_parameter<std::string>("mpc_solver", "PARTIAL_CONDENSING_OSQP");
  this->declare_parameter<int>("mpc_condensed_size", MPC_PREDICTION_HORIZON);
  this->declare_parameter<std::string>("mpc_hpipm_mode", "SPEED_ABS");
  this->declare_parameter<std::string>("wbc.arc_opt.solver", "EiquadprogSolver");
  this->declare_parameter<std::string>("wbc.arc_opt.scene", "AccelerationSceneReducedTSID");
  this->declare_parameter<int>("mpc_warm_start", 1);

  leg_control_mode_ = static_cast<LEGControlMode>(this->get_parameter("leg_control_mode").as_int());
  early_contact_detection_ = this->get_parameter("early_contact_detection").as_bool();
  late_contact_detection_ = this->get_parameter("late_contact_detection").as_bool();
  lost_contact_detection_ = this->get_parameter("lost_contact_detection").as_bool();
  late_contact_reschedule_swing_phase_ = this->get_parameter("late_contact_reschedule_swing_phase").as_bool();
  use_model_adaptation_ = this->get_parameter("use_model_adaptation").as_bool();
  RCLCPP_INFO_EXPRESSION(this->get_logger(), early_contact_detection_, "Early contact detection is activated");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), late_contact_detection_, "Late contact detection is activated");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), lost_contact_detection_, "Lost contact detection is activated");
  // Load PD gains
  switch (leg_control_mode_) {
    case CARTESIAN_JOINT_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in CARTESIAN_JOINT_CONTROL mode");
      this->declare_parameter("cartesian_joint_control_gains.swing_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.swing_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.stance_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_joint_control_gains.stance_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("cartesian_joint_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().size() == 3);
      cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.swing_Kp").as_double_array().data());
      cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.swing_Kd").as_double_array().data());
      cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.stance_Kp").as_double_array().data());
      cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_joint_control_gains.stance_Kd").as_double_array().data());
      break;
    case CARTESIAN_STIFFNESS_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in CARTESIAN_STIFFNESS_CONTROL mode");
      this->declare_parameter("cartesian_stiffness_control_gains.swing_Kp",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.swing_Kd",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.stance_Kp",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("cartesian_stiffness_control_gains.stance_Kd",
                              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("cartesian_stiffness_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.stance_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("cartesian_stiffness_control_gains.stance_Kd").as_double_array().size() == 3);
      cartesian_stiffness_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.swing_Kp").as_double_array().data());
      cartesian_stiffness_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.swing_Kd").as_double_array().data());
      cartesian_stiffness_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.stance_Kp").as_double_array().data());
      cartesian_stiffness_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("cartesian_stiffness_control_gains.stance_Kd").as_double_array().data());
      break;
    case JOINT_CONTROL:
      [[fallthrough]];
    case JOINT_TORQUE_CONTROL:
      RCLCPP_INFO(this->get_logger(), "Controller running in JOINT_CONTROL or JOINT_TORQUE_CONTROL mode");
      this->declare_parameter("joint_control_gains.swing_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.swing_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.stance_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("joint_control_gains.stance_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      assert(this->get_parameter("joint_control_gains.swing_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.swing_Kd").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.stance_Kp").as_double_array().size() == 3);
      assert(this->get_parameter("joint_control_gains.stance_Kd").as_double_array().size() == 3);
      joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.swing_Kp").as_double_array().data());
      joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.swing_Kd").as_double_array().data());
      joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.stance_Kp").as_double_array().data());
      joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("joint_control_gains.stance_Kd").as_double_array().data());
      break;
  }
  feet_status_.fill(STANCE);  // Start with current stance

  // Prepare target
  target_.active.hybrid_x_dot = true;
  target_.active.hybrid_y_dot = true;
  target_.active.wz = true;
  target_.active.z = true;
  target_.active.z_dot = false;
  target_.active.roll = true;
  target_.active.pitch = true;
  target_.active.yaw = false;
  target_.z = this->get_parameter("initial_height").as_double();
  target_.hybrid_x_dot = 0.0;
  target_.hybrid_y_dot = 0.0;
  target_.z_dot = 0.0;
  target_.wz = 0.0;
  target_.roll = 0.0;
  target_.pitch = 0.0;
  target_.active.wx = true;
  target_.active.wy = true;

  //
  target_.active.x = false;
  target_.active.y = false;
  target_.x = 0;
  target_.y = 0;

  // prepare ROS related things
  control_loop_call_back_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  slc_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  mpc_call_back_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  model_adaptation_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // assert that it fits to WBC
  switch (leg_control_mode_) {
    case JOINT_CONTROL:
      [[fallthrough]];
    case JOINT_TORQUE_CONTROL:
      assert(typeid(wbc_.get()) == typeid(WBCInterface<JointTorqueVelocityPositionCommands> *)
             or typeid(wbc_.get()) == typeid(WBCInterface<JointTorqueCommands> *));
      leg_joint_cmd_publisher_ =
          this->create_publisher<interfaces::msg::JointCmd>("leg_joint_cmd", QOS_RELIABLE_NO_DEPTH);
      break;
    case CARTESIAN_STIFFNESS_CONTROL:
      [[fallthrough]];
    case CARTESIAN_JOINT_CONTROL:
      assert(typeid(wbc_.get()) == typeid(WBCInterface<CartesianCommands> *));
      model_adaptation_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      leg_cmd_publisher_ = this->create_publisher<interfaces::msg::LegCmd>("leg_cmd", QOS_RELIABLE_NO_DEPTH);
      break;
  }
  quad_model_publisher_ =
      this->create_publisher<interfaces::msg::QuadModel>("quad_model_update", QOS_RELIABLE_NO_DEPTH);
  quad_state_subscription_ = this->create_subscription<interfaces::msg::QuadState>(
      "quad_state",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&MITController::QuadStateUpdateCallback, this, std::placeholders::_1));

  if (PUBLISH_SWING_LEG_TRAJECTORIES) {
    swing_leg_trajs_publisher_ =
        this->create_publisher<interfaces::msg::VectorSequence>("swing_leg_trajs", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_GAIT_STATE) {
    gait_state_publisher_ = this->create_publisher<interfaces::msg::GaitState>("gait_state", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_OPEN_LOOP_TRAJECTORY) {
    open_loop_publisher_ =
        this->create_publisher<interfaces::msg::PositionSequence>("open_loop_trajectory", QOS_BEST_EFFORT_NO_DEPTH);
  }
  if (PUBLISH_SOLVE_TIME) {
    solve_time_publisher_ =
        this->create_publisher<interfaces::msg::MPCDiagnostics>("solve_time", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_WBC_SOLVE_TIME) {
    wbc_solve_time_publisher_ =
        this->create_publisher<interfaces::msg::WBCReturn>("wbc_solve_time", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_WBC_TARGET) {
    wbc_target_publisher_ = this->create_publisher<interfaces::msg::WBCTarget>("wbc_target", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_GAIT_SEQUENCE) {
    gait_sequence_publisher_ =
        this->create_publisher<interfaces::msg::GaitSequence>("gait_sequence", QOS_BEST_EFFORT_NO_DEPTH);
  }

  if (PUBLISH_HEARTBEAT) {
    controller_heartbeat_publisher_ =
        this->create_publisher<interfaces::msg::ControllerInfo>("controller_heartbeat", QOS_BEST_EFFORT_NO_DEPTH);
    heartbeat_loop_timer_ =
        rclcpp::create_timer(this,
                             this->get_clock(),
                             std::chrono::duration<double>(this->get_parameter("controller_heartbeat_dt").as_double()),
                             std::bind(&MITController::HartbeatCallback, this));
  }

  change_leg_driver_mode_client_ = this->create_client<interfaces::srv::ChangeLegDriverMode>("switch_op_mode");
  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  // prepare gaits
  this->declare_parameter("gait_sequencer", "Simple");
  this->declare_parameter("simple_gait_sequencer.gait", "STAND");
  this->declare_parameter("simple_gait_sequencer.manual_gait.period", 0.5);
  this->declare_parameter<std::vector<double>>("simple_gait_sequencer.manual_gait.duty_factor", {0.6, 0.6, 0.6, 0.6});
  this->declare_parameter<std::vector<double>>("simple_gait_sequencer.manual_gait.phase_offset", {0.0, 0.5, 0.5, 0.0});
  this->declare_parameter("adaptive_gait_sequencer.gait.swing_time", 0.2);
  this->declare_parameter("adaptive_gait_sequencer.gait.min_v_cmd_factor", 0.5);
  this->declare_parameter("adaptive_gait_sequencer.gait.filter_size", 10);
  this->declare_parameter("adaptive_gait_sequencer.gait.zero_velocity_threshold", 0.03);
  this->declare_parameter("adaptive_gait_sequencer.gait.standing_foot_position_threshold", 0.08);
  this->declare_parameter("adaptive_gait_sequencer.gait.max_correction_cycles", 2.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.correct_all", false);
  this->declare_parameter("adaptive_gait_sequencer.gait.correction_period", 0.6);
  this->declare_parameter("adaptive_gait_sequencer.gait.disturbance_correction", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.min_v", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.offset_delay", 0.0);
  this->declare_parameter("adaptive_gait_sequencer.gait.max_stride_length", 0.0);

  this->declare_parameter("adaptive_gait_sequencer.gait.switch_offsets", true);
  this->declare_parameter<std::vector<double>>("adaptive_gait_sequencer.gait.phase_offset", {0.0, 0.5, 0.5, 0.0});

  on_setparam_callback_handler_ =
      this->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult res;
        for (auto &param : params) {
          if (param.get_name().find("mpc_state_weights") != std::string::npos
              and param.as_double_array().size() != (MPC::STATE_SIZE - 1)) {
            res.successful = false;
            res.reason = "MPC state weight vector has wrong size";
            return res;
          } else if (param.get_name().find("cartesian_joint_control_gains") != std::string::npos
                     and param.as_double_array().size() != N_JOINTS_PER_LEG)

          {
            res.successful = false;
            res.reason = "cartesian_joint_control_gains vector has wrong size";
            return res;
          } else if (param.get_name().find("cartesian_stiffness_control_gains") != std::string::npos
                     and param.as_double_array().size() != N_JOINTS_PER_LEG)

          {
            res.successful = false;
            res.reason = "cartesian_stiffness_control_gains vector has wrong size";
            return res;
          }
        }
        res.successful = true;
        return res;
      });

  parameter_event_callback_handle_ = parameter_event_handler_->add_parameter_event_callback([this](
                                                                                                const rcl_interfaces::
                                                                                                    msg::ParameterEvent
                                                                                                        &param_event) {
    bool gait_update = false;
    for (const auto &param : param_event.changed_parameters) {
      if (param.name.find("gait") != std::string::npos) {
        if ((param.name.find("adaptive_gait_sequencer.gait") != std::string::npos)
            && (dynamic_cast<AdaptiveGaitSequencer *>(gs_.get()))) {
          auto ad_gs = dynamic_cast<AdaptiveGaitSequencer *>(gs_.get());
          AdaptiveGait &gait = ad_gs->Gait();
          if (param.name == "adaptive_gait_sequencer.gait.phase_offset") {
            auto po = this->get_parameter("adaptive_gait_sequencer.gait.phase_offset").as_double_array();
            gait.set_offset(to_array<N_LEGS>(po));
          } else if (param.name == "adaptive_gait_sequencer.gait.swing_time") {
            gait.set_swing_time(this->get_parameter("adaptive_gait_sequencer.gait.swing_time").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.filter_size") {
            gait.set_filter_size(this->get_parameter("adaptive_gait_sequencer.gait.filter_size").as_int());
          } else if (param.name == "adaptive_gait_sequencer.gait.zero_velocity_threshold") {
            gait.set_zero_velocity_threshold(
                this->get_parameter("adaptive_gait_sequencer.gait.zero_velocity_threshold").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.switch_offsets") {
            gait.set_offset_switch(this->get_parameter("adaptive_gait_sequencer.gait.switch_offsets").as_bool());
          } else if (param.name == "adaptive_gait_sequencer.gait.standing_foot_position_threshold") {
            gait.set_standing_foot_position_threshold(
                this->get_parameter("adaptive_gait_sequencer.gait.standing_foot_position_threshold").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.min_v_cmd_factor") {
            gait.set_min_v_cmd_factor(this->get_parameter("adaptive_gait_sequencer.gait.min_v_cmd_factor").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.max_correction_cycles") {
            gait.set_max_correction_cycles(
                this->get_parameter("adaptive_gait_sequencer.gait.max_correction_cycles").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.correct_all") {
            gait.set_correct_all(this->get_parameter("adaptive_gait_sequencer.gait.correct_all").as_bool());
          } else if (param.name == "adaptive_gait_sequencer.gait.correction_period") {
            gait.set_correction_period(
                this->get_parameter("adaptive_gait_sequencer.gait.correction_period").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.disturbance_correction") {
            gait.set_disturbance_correction(
                this->get_parameter("adaptive_gait_sequencer.gait.disturbance_correction").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.min_v") {
            gait.set_min_v(this->get_parameter("adaptive_gait_sequencer.gait.min_v").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.offset_delay") {
            gait.set_offset_delay(this->get_parameter("adaptive_gait_sequencer.gait.offset_delay").as_double());
          } else if (param.name == "adaptive_gait_sequencer.gait.max_stride_length") {
            gait.set_max_stride_length(
                this->get_parameter("adaptive_gait_sequencer.gait.max_stride_length").as_double());
          }
        } else {
          gait_update = true;
        }
      } else if (param.name == "mpc_state_weights_stand") {
        RCLCPP_INFO(this->get_logger(), "Updating mpc state weights stand");
        mpc_lock_.lock();
        state_weights_stand_ =
            Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(param.value.double_array_value.data());
        if (last_gait_sequence_mode_ == GaitSequence::KEEP) {
          reinterpret_cast<MPC *>(mpc_.get())->SetStateWeights(state_weights_stand_);
        }
        mpc_lock_.unlock();
      } else if (param.name == "mpc_state_weights_move") {
        RCLCPP_INFO(this->get_logger(), "Updating mpc state weights move");
        mpc_lock_.lock();
        state_weights_move_ =
            Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(param.value.double_array_value.data());
        if (last_gait_sequence_mode_ == GaitSequence::MOVE) {
          reinterpret_cast<MPC *>(mpc_.get())->SetStateWeights(state_weights_move_);
        }
        mpc_lock_.unlock();
      } else if (param.name == "mpc_alpha") {
        RCLCPP_INFO(this->get_logger(), "Updating mpc input weights");
        mpc_lock_.lock();
        reinterpret_cast<MPC *>(mpc_.get())->SetInputWeights(param.value.double_value);
        mpc_lock_.unlock();
      } else if (param.name == "mpc_fmax") {
        RCLCPP_INFO(this->get_logger(), "Updating mpc fmax");
        mpc_lock_.lock();
        reinterpret_cast<MPC *>(mpc_.get())->SetFmax(param.value.double_value);
        mpc_lock_.unlock();
      } else if (param.name == "mpc_mu") {
        RCLCPP_INFO(this->get_logger(), "Updating mpc mu");
        mpc_lock_.lock();
        reinterpret_cast<MPC *>(mpc_.get())->SetMu(param.value.double_value);
        mpc_lock_.unlock();
      } else if (param.name
                 == "cartesian_joint_control_gains.swing_Kp") {  // TODO: syncronisation, maybe use atomic vars?
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_joint_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kp") {
        cartesian_joint_control_swing_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.swing_Kd") {
        cartesian_joint_control_swing_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kp") {
        cartesian_joint_control_stance_Kp_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "cartesian_stiffness_control_gains.stance_Kd") {
        cartesian_joint_control_stance_Kd_ = Eigen::Map<const Eigen::Vector3d>(param.value.double_array_value.data());
      } else if (param.name == "early_contact_detection") {
        early_contact_detection_ = param.value.bool_value;
      } else if (param.name == "late_contact_detection") {
        late_contact_detection_ = param.value.bool_value;
      } else if (param.name == "lost_contact_detection") {
        lost_contact_detection_ = param.value.bool_value;
      } else if (param.name == "use_model_adaptation") {
        use_model_adaptation_ = param.value.bool_value;
      } else if (param.name == "late_contact_reschedule_swing_phase") {
        late_contact_reschedule_swing_phase_ = param.value.bool_value;
      } else if (param.name == "wbc.inverse_dynamics.foot_position_based_on_target_height") {
        if (typeid(wbc_.get()) == typeid(InverseDynamics)) {
          wbc_lock_.lock();
          reinterpret_cast<InverseDynamics *>(wbc_.get())->setFootPositionBasedOnTargetHeight(param.value.bool_value);
          wbc_lock_.unlock();
        }
      } else if (param.name == "wbc.inverse_dynamics.target_velocity_blend") {
        if (typeid(wbc_.get()) == typeid(InverseDynamics)) {
          wbc_lock_.lock();
          reinterpret_cast<InverseDynamics *>(wbc_.get())->setTargetVelocityBlend(param.value.double_value);
          wbc_lock_.unlock();
        }
      } else if (param.name == "wbc.inverse_dynamics.foot_position_based_on_target_orientation") {
        if (typeid(wbc_.get()) == typeid(InverseDynamics)) {
          wbc_lock_.lock();
          reinterpret_cast<InverseDynamics *>(wbc_.get())
              ->setFootPositionBasedOnTargetOrientation(param.value.bool_value);
          wbc_lock_.unlock();
        }
      } else if (param.name == "slc_swing_height") {
        slc_lock_.lock();
        reinterpret_cast<SwingLegController *>(slc_.get())->SetSwingHeight(param.value.double_value);
        slc_lock_.unlock();
      } else if (param.name == "slc_world_blend") {
        slc_lock_.lock();
        reinterpret_cast<SwingLegController *>(slc_.get())->SetWorldBlend(param.value.double_value);
        slc_lock_.unlock();
      } else if (param.name == "wbc.inverse_dynamics.transformation_filter_size") {
        wbc_lock_.lock();
        reinterpret_cast<InverseDynamics *>(wbc_.get())
            ->setFootPositionBasedOnTargetOrientation(param.value.integer_value);
        wbc_lock_.unlock();
      } else if (param.name == "maximum_swing_leg_progress_to_update_target") {
        slc_lock_.lock();
        reinterpret_cast<SwingLegController *>(slc_.get())
            ->SetMaximumSwingProgressToUpdateTarget(param.value.double_value);
        slc_lock_.unlock();
      } else if (param.name == "raibert.z_on_plane") {
        gait_update = true;
      } else if (param.name == "raibert.k") {
        gait_update = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Changing parameter %s is not yet suported", param.name.c_str());
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Changed parameter %s sucessfully", param.name.c_str());
    }
    if (gait_update) {
      RCLCPP_INFO(this->get_logger(), "Gait related parameter has changed, reloading gait sequencer");
      auto gs = GetGaitSequencerFromParams();
      if (gs) {
        gait_sequencer_lock_.lock();
        gs_ = std::move(gs);
        gait_sequencer_lock_.unlock();
      }
    }
  });

  // Control parts
  assert(this->get_parameter("mpc_state_weights_stand").as_double_array().size() == (MPC::STATE_SIZE - 1));
  assert(this->get_parameter("mpc_state_weights_move").as_double_array().size() == (MPC::STATE_SIZE - 1));
  state_weights_stand_ = Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(
      this->get_parameter("mpc_state_weights_stand").as_double_array().data());
  state_weights_move_ = Eigen::Map<const Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1>>(
      this->get_parameter("mpc_state_weights_move").as_double_array().data());

  while (!first_quad_state_received_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         2000,
                         "Waiting for first quad state message to properly initialize controller");
    rclcpp::spin_some(this->get_node_base_interface());
  }
  RCLCPP_INFO(this->get_logger(), "First quad_state received, initializing controller");

  gs_ = GetGaitSequencerFromParams();
  if (!gs_) {
    RCLCPP_ERROR(this->get_logger(), "Could not load gait sequencer from params");
    rclcpp::shutdown();
  }

  auto mpc_solver_name = this->get_parameter("mpc_solver").as_string();
  ocp_qp_solver_t mpc_solver = PARTIAL_CONDENSING_OSQP;
  if (mpc_solver_name == "PARTIAL_CONDENSING_HPIPM") {
    mpc_solver = PARTIAL_CONDENSING_HPIPM;
  } else if (mpc_solver_name == "PARTIAL_CONDENSING_OSQP") {
    mpc_solver = PARTIAL_CONDENSING_OSQP;
  } else if (mpc_solver_name == "FULL_CONDENSING_HPIPM") {
    mpc_solver = FULL_CONDENSING_HPIPM;
  } else if (mpc_solver_name == "FULL_CONDENSING_DAQP") {
    mpc_solver = FULL_CONDENSING_DAQP;
  } else if (mpc_solver_name == "FULL_CONDENSING_QPOASES") {
    mpc_solver = FULL_CONDENSING_QPOASES;
  } else if (mpc_solver_name == "PARTIAL_CONDENSING_QPDUNES") {
    mpc_solver = PARTIAL_CONDENSING_QPDUNES;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown mpc solver: %s", mpc_solver_name.c_str());
    exit(-1);
  }
  mpc_ = std::make_unique<MPC>(this->get_parameter("mpc_alpha").as_double(),
                               state_weights_stand_,
                               this->get_parameter("mpc_mu").as_double(),
                               this->get_parameter("mpc_fmin").as_double(),
                               this->get_parameter("mpc_fmax").as_double(),
                               std::make_unique<QuadState>(quad_state_),
                               std::make_unique<QuadModelSymbolic>(quad_model_),
                               mpc_solver,
                               this->get_parameter("mpc_condensed_size").as_int(),
                               this->get_parameter("mpc_hpipm_mode").as_string(),
                               this->get_parameter("mpc_warm_start").as_int());

  Eigen::DiagonalMatrix<double, 10> model_parameter_weights;
  assert(this->get_parameter("model_parameter_weights").as_double_array().size() == 10);
  ma_ = std::make_unique<LeastSquaresModelAdaptation>(
      std::make_unique<QuadModelSymbolic>(quad_model_),
      std::make_unique<QuadState>(quad_state_),
      MODEL_ADAPTATION_BATCH_SIZE,
      Eigen::DiagonalMatrix<double, 10>(Eigen::Map<const Eigen::Matrix<double, 10, 1>>(
          this->get_parameter("model_parameter_weights").as_double_array().data())),
      0.5);
  slc_ = std::make_unique<SwingLegController>(
      this->get_parameter("slc_swing_height").as_double(),
      this->get_parameter("maximum_swing_leg_progress_to_update_target").as_double(),
      this->get_parameter("slc_world_blend").as_double(),
      std::make_unique<QuadModelSymbolic>(quad_model_),
      std::make_unique<QuadState>(quad_state_));

  gs_->UpdateTarget(target_);

  WBCType *wbc_ptr;

  if constexpr (USE_WBC) {
    std::string wbc_solver_name;
    std::string wbc_scene_name;
    wbc_solver_name = this->get_parameter("wbc.arc_opt.solver").as_string();
    wbc_scene_name = this->get_parameter("wbc.arc_opt.scene").as_string();
    wbc_ptr = reinterpret_cast<WBCType *>(new WBCArcOPT(
        std::make_unique<QuadState>(quad_state_),
        wbc_solver_name,
        wbc_scene_name,
        this->get_parameter("wbc.arc_opt.model_urdf").as_string(),
        to_array<ModelInterface::N_LEGS>(this->get_parameter("wbc.arc_opt.feet_names").as_string_array()),
        to_array<ModelInterface::NUM_JOINTS>(this->get_parameter("wbc.arc_opt.joint_names").as_string_array()),
        this->get_parameter("wbc.arc_opt.mu").as_double(),
        as_eigen_vector<6>(this->get_parameter("wbc.arc_opt.com_pose_weight").as_double_array()),
        as_eigen_vector<3>(this->get_parameter("wbc.arc_opt.foot_pose_weight").as_double_array()),
        as_eigen_vector<3>(this->get_parameter("wbc.arc_opt.foot_force_weight").as_double_array()),
        as_eigen_vector<6>(this->get_parameter("wbc.arc_opt.com_pose_Kp").as_double_array()),
        as_eigen_vector<6>(this->get_parameter("wbc.arc_opt.com_pose_Kd").as_double_array()),
        as_eigen_vector<3>(this->get_parameter("wbc.arc_opt.feet_pose_Kp").as_double_array()),
        as_eigen_vector<3>(this->get_parameter("wbc.arc_opt.feet_pose_Kd").as_double_array()),
        as_eigen_vector<6>(this->get_parameter("wbc.arc_opt.com_pose_saturation").as_double_array()),
        as_eigen_vector<3>(this->get_parameter("wbc.arc_opt.feet_pose_saturation").as_double_array())));
  } else {
    wbc_ptr = reinterpret_cast<WBCType *>(new InverseDynamics(
        std::make_unique<QuadModelSymbolic>(quad_model_),
        std::make_unique<QuadState>(quad_state_),
        this->get_parameter("wbc.inverse_dynamics.foot_position_based_on_target_height").as_bool(),
        this->get_parameter("wbc.inverse_dynamics.foot_position_based_on_target_orientation").as_bool(),
        this->get_parameter("wbc.inverse_dynamics.transformation_filter_size").as_int(),
        this->get_parameter("wbc.inverse_dynamics.target_velocity_blend").as_double()));
  }
  wbc_ = std::unique_ptr<WBCType>(wbc_ptr);

  // Now change modus of le driver acording to this controller
  auto req = std::make_shared<interfaces::srv::ChangeLegDriverMode::Request>();
  req->target_mode = static_cast<int>(leg_control_mode_);
  RCLCPP_INFO(this->get_logger(), "Waiting for leg driver service to become available");
  change_leg_driver_mode_client_->wait_for_service();
  auto client_request = change_leg_driver_mode_client_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), client_request);
  if (client_request.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Starting controller");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Couldn't reach leg driver - undefined behaviour from now on");
  }

  rclcpp::SubscriptionOptions quad_control_target_subscription_opts;
  quad_control_target_subscription_opts.callback_group = mpc_call_back_group_;
  quad_control_target_subscription_ = this->create_subscription<interfaces::msg::QuadControlTarget>(
      "quad_control_target",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&MITController::QuadControlTargetUpdateCallback, this, std::placeholders::_1));

  mpc_loop_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::duration<double>(MPC_CONTROL_DT),
                                         std::bind(&MITController::MPCLoopCallback, this),
                                         mpc_call_back_group_);

  model_adaptation_loop_timer_ =
      rclcpp::create_timer(this,
                           this->get_clock(),
                           std::chrono::duration<double>(MODEL_ADAPTATION_DT),
                           std::bind(&MITController::ModelAdaptationCallback, this),
                           model_adaptation_callback_group_);  // TODO: at some point when MPC is adapting think
                                                               // if it has to be in the saem callback group
}

void MITController::QuadStateUpdateCallback(interfaces::msg::QuadState::SharedPtr quad_state_msg) {
  first_quad_state_received_ = true;
  quad_state_lock_.lock();
  quad_state_ = *quad_state_msg;
  quad_state_lock_.unlock();
}
std::unique_ptr<GaitSequencerInterface> MITController::GetGaitSequencerFromParams() const {
  assert(this->get_parameter("gs_shoulder_positions").as_double_array().size() == (N_LEGS * 3));
  const std::string gait_sequencer = this->get_parameter("gait_sequencer").as_string();

  if (gait_sequencer == "Simple") {
    RCLCPP_INFO(this->get_logger(), "Creating simple gait sequencer");
    std::string gait_str = this->get_parameter("simple_gait_sequencer.gait").as_string();
    Gait gait = GaitDatabase::getGait(GaitDatabase::STAND, MPC_DT);  // Default is STAND
    if (gait_str == "Manual" or gait_str == "MANUAL") {
      auto df = this->get_parameter("simple_gait_sequencer.manual_gait.duty_factor").as_double_array();
      auto po = this->get_parameter("simple_gait_sequencer.manual_gait.phase_offset").as_double_array();
      if (df.size() != N_LEGS or po.size() != N_LEGS) {
        RCLCPP_ERROR(this->get_logger(), "manual gait parameters have wrong length");
        return nullptr;
      }
      gait = Gait(this->get_parameter("simple_gait_sequencer.manual_gait.period").as_double(),
                  to_array<N_LEGS>(df),
                  to_array<N_LEGS>(po),
                  MPC_DT);
    } else if (gait_str == "STAND") {
      gait = GaitDatabase::getGait(GaitDatabase::STAND, MPC_DT);
    } else if (gait_str == "STATIC_WALK") {
      gait = GaitDatabase::getGait(GaitDatabase::STATIC_WALK, MPC_DT);
    } else if (gait_str == "WALKING_TROT") {
      gait = GaitDatabase::getGait(GaitDatabase::WALKING_TROT, MPC_DT);
    } else if (gait_str == "TROT") {
      gait = GaitDatabase::getGait(GaitDatabase::TROT, MPC_DT);
    } else if (gait_str == "FLYING_TROT") {
      gait = GaitDatabase::getGait(GaitDatabase::FLYING_TROT, MPC_DT);
    } else if (gait_str == "PACE") {
      gait = GaitDatabase::getGait(GaitDatabase::PACE, MPC_DT);
    } else if (gait_str == "BOUND") {
      gait = GaitDatabase::getGait(GaitDatabase::BOUND, MPC_DT);
    } else if (gait_str == "ROTARY_GALLOP") {
      gait = GaitDatabase::getGait(GaitDatabase::ROTARY_GALLOP, MPC_DT);
    } else if (gait_str == "TRAVERSE_GALLOP") {
      gait = GaitDatabase::getGait(GaitDatabase::TRAVERSE_GALLOP, MPC_DT);
    } else if (gait_str == "PRONK") {
      gait = GaitDatabase::getGait(GaitDatabase::PRONK, MPC_DT);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown gait type [%s]", gait_str.c_str());
      return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Create gait of type [%s]", gait_str.c_str());

    return std::make_unique<SimpleGaitSequencer>(
        gait,
        this->get_parameter("raibert.k").as_double(),
        std::array<const Eigen::Vector3d, N_LEGS>{
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 3),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 6),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 9)},
        std::make_unique<QuadState>(quad_state_),
        // TODO: do they have to be gin an initial good state?
        std::make_unique<QuadModelSymbolic>(quad_model_),
        this->get_parameter("raibert.filtersize").as_int(),
        this->get_parameter("raibert.z_on_plane").as_bool(),
        this->get_parameter("fix_standing_position").as_bool(),
        this->get_parameter("fix_position_distance_threshold").as_double(),
        this->get_parameter("fix_position_angular_threshold").as_double(),
        this->get_parameter("fix_position_velocity_threshold").as_double(),
        early_contact_detection_);

  } else if (gait_sequencer == "Adaptive") {
    RCLCPP_INFO(this->get_logger(), "Creating adaptive gait sequencer");
    auto po = this->get_parameter("adaptive_gait_sequencer.gait.phase_offset").as_double_array();
    if (po.size() != N_LEGS) {
      RCLCPP_ERROR(this->get_logger(), "manual gait parameters have wrong length");
      return nullptr;
    }
    return std::make_unique<AdaptiveGaitSequencer>(
        AdaptiveGait(to_array<N_LEGS>(po),
                     MPC_DT,
                     this->get_parameter("adaptive_gait_sequencer.gait.swing_time").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.filter_size").as_int(),
                     this->get_parameter("adaptive_gait_sequencer.gait.zero_velocity_threshold").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.switch_offsets").as_bool(),
                     this->get_parameter("adaptive_gait_sequencer.gait.standing_foot_position_threshold").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.min_v_cmd_factor").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.max_correction_cycles").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.correct_all").as_bool(),
                     this->get_parameter("adaptive_gait_sequencer.gait.correction_period").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.disturbance_correction").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.min_v").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.offset_delay").as_double(),
                     this->get_parameter("adaptive_gait_sequencer.gait.max_stride_length").as_double()),
        this->get_parameter("raibert.k").as_double(),
        std::array<const Eigen::Vector3d, N_LEGS>{
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 3),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 6),
            Eigen::Map<const Eigen::Vector3d>(this->get_parameter("gs_shoulder_positions").as_double_array().data()
                                              + 9)},
        std::make_unique<QuadState>(quad_state_),
        std::make_unique<QuadModelSymbolic>(quad_model_),
        this->get_parameter("raibert.filtersize").as_int(),
        this->get_parameter("raibert.z_on_plane").as_bool(),
        this->get_parameter("fix_standing_position").as_bool(),
        this->get_parameter("fix_position_distance_threshold").as_double(),
        this->get_parameter("fix_position_angular_threshold").as_double(),
        this->get_parameter("fix_position_velocity_threshold").as_double(),
        early_contact_detection_);

  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown gait sequencer type [%s]", gait_sequencer.c_str());
    return nullptr;
  }
}

void MITController::QuadControlTargetUpdateCallback(interfaces::msg::QuadControlTarget::SharedPtr quad_target_msg) {
  target_.hybrid_x_dot = quad_target_msg->body_x_dot;
  target_.hybrid_y_dot = quad_target_msg->body_y_dot;
  target_.z = quad_target_msg->world_z;
  target_.wz = quad_target_msg->hybrid_theta_dot;
  target_.roll = quad_target_msg->roll;
  target_.pitch = quad_target_msg->pitch;
  gs_->UpdateTarget(target_);
}

void MITController::MPCLoopCallback() {
  // Get gait sequence and update other parts
  static GaitSequence gait_sequence_temp;
  static WrenchSequence ws_temp;
  static MPCPrediction mpc_prediction_temp;

  mpc_lock_.lock();
  gait_sequencer_lock_.lock();
  // Update States:
  quad_state_lock_.lock();
  QuadState quad_state_temp = quad_state_;
  quad_state_lock_.unlock();

  gs_->UpdateState(quad_state_temp);
  mpc_->UpdateState(quad_state_temp);
  slc_->UpdateState(quad_state_temp);
  gs_->GetGaitSequence(gait_sequence_temp);
  mpc_->UpdateGaitSequence(gait_sequence_temp);

  if (last_gait_sequence_mode_ != gait_sequence_temp.sequence_mode) {
    RCLCPP_DEBUG(this->get_logger(), "Changing MPC weights because sequence mode changed");
    switch (gait_sequence_temp.sequence_mode) {
      case GaitSequence::KEEP:
        reinterpret_cast<MPC *>(mpc_.get())->SetStateWeights(state_weights_stand_);
        if (PUBLISH_HEARTBEAT) {
          controller_heartbeat_.keep_pose_active = true;
        }
        break;  // TODO: this and next might have internal doubled
                // operations with target set
      case GaitSequence::MOVE:
        reinterpret_cast<MPC *>(mpc_.get())->SetStateWeights(state_weights_move_);
        if (PUBLISH_HEARTBEAT) {
          controller_heartbeat_.keep_pose_active = false;
        }
        break;
    }
    last_gait_sequence_mode_ = gait_sequence_temp.sequence_mode;
  }

  static SolverInformation solver_info;
  static double mpc_solve_time = 0.0;
  // Calculate controls
  auto mpc_start_time = std::chrono::high_resolution_clock::now();
  mpc_->GetWrenchSequence(ws_temp, mpc_prediction_temp, solver_info);  // This one might block
  auto mpc_end_time = std::chrono::high_resolution_clock::now();

  RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), !solver_info.success, "MPC solver did not converge, code %d", solver_info.return_code);

  RCLCPP_WARN_EXPRESSION(this->get_logger(),
                         solver_info.total_solver_time >= MPC_CONTROL_DT,
                         "MPC solver took longer [%f s] than MPC control dt [%f s]",
                         solver_info.total_solver_time,
                         MPC_CONTROL_DT);

  RCLCPP_DEBUG(this->get_logger(),
               "MPC solver returned [%d] after [%f s], [%s]",
               solver_info.return_code,
               solver_info.total_solver_time,
               (solver_info.success ? "true" : "false"));

  gs_wrench_sequence_lock_.lock();
  wrench_sequence_ = ws_temp;
  gait_sequence_ = gait_sequence_temp;
  mpc_prediction_ = mpc_prediction_temp;
  gs_updated_ = true;
  gs_wrench_sequence_lock_.unlock();

  // Only if this ran once, we can start the slc thread:
  if (control_loop_timer_ == nullptr) {
    slc_loop_timer_ = rclcpp::create_timer(this,
                                           this->get_clock(),
                                           std::chrono::duration<double>(SWING_LEG_DT),
                                           std::bind(&MITController::SLCLoopCallback, this),
                                           slc_callback_group_);
    control_loop_timer_ = rclcpp::create_timer(this,
                                               this->get_clock(),
                                               std::chrono::duration<double>(CONTROL_DT),
                                               std::bind(&MITController::ControlLoopCallback, this),
                                               control_loop_call_back_group_);
  }
  gait_sequencer_lock_.unlock();
  mpc_lock_.unlock();

  mpc_solve_time = std::chrono::duration_cast<std::chrono::duration<double>>(mpc_end_time - mpc_start_time).count();

  // publish solve time
  if (PUBLISH_SOLVE_TIME) {
    interfaces::msg::MPCDiagnostics solve_time_message;
    solve_time_message.header.stamp = this->get_clock()->now();
    solve_time_message.solve_time = mpc_solve_time;
    solve_time_message.acados_solve_qp_time = solver_info.acados_solve_QP_time;
    solve_time_message.acados_condensing_time = solver_info.acados_condensing_time;
    solve_time_message.acados_interface_time = solver_info.acados_interface_time;
    solve_time_message.acados_total_time = solver_info.total_solver_time;
    solve_time_message.acados_num_iter = solver_info.acados_num_iter;
    solve_time_message.acados_t_computed = solver_info.acados_t_computed;
    solve_time_publisher_->publish(solve_time_message);
  }

  if (PUBLISH_HEARTBEAT) {
    if (!solver_info.success) {
      controller_heartbeat_.num_mpc_solver_fail++;
    }
    if (solver_info.total_solver_time >= MPC_CONTROL_DT) {
      controller_heartbeat_.num_mpc_solver_overtime++;
    }
  }

#ifdef DEBUG_PRINTS
  RCLCPP_DEBUG(this->get_logger(),
               "Contacts are \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[0][0],
               gait_sequence_.contact_sequence[0][1],
               gait_sequence_.contact_sequence[0][2],
               gait_sequence_.contact_sequence[0][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is  \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[1][0],
               gait_sequence_.contact_sequence[1][1],
               gait_sequence_.contact_sequence[1][2],
               gait_sequence_.contact_sequence[1][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[2][0],
               gait_sequence_.contact_sequence[2][1],
               gait_sequence_.contact_sequence[2][2],
               gait_sequence_.contact_sequence[2][3]);
  RCLCPP_DEBUG(this->get_logger(),
               "Plan is \t [%i, %i, %i,%i]",
               gait_sequence_.contact_sequence[3][0],
               gait_sequence_.contact_sequence[3][1],
               gait_sequence_.contact_sequence[3][2],
               gait_sequence_.contact_sequence[3][3]);

  RCLCPP_DEBUG(this->get_logger(),
               "Forces are    \n\t\t [(%f,%f,%f), \t(%f,%f,%f), \n\t\t "
               "(%f,%f,%f),  \t(%f,%f,%f)]",
               ws_temp.forces[0][0].x(),
               ws_temp.forces[0][0].y(),
               ws_temp.forces[0][0].z(),
               ws_temp.forces[0][1].x(),
               ws_temp.forces[0][1].y(),
               ws_temp.forces[0][1].z(),
               ws_temp.forces[0][2].x(),
               ws_temp.forces[0][2].y(),
               ws_temp.forces[0][2].z(),
               ws_temp.forces[0][3].x(),
               ws_temp.forces[0][3].y(),
               ws_temp.forces[0][3].z());
#endif
  if (gait_state_publisher_ != nullptr) {
    static interfaces::msg::GaitState gait_state_msg;
    gs_->GetGaitState(gait_state_msg);
    gait_state_publisher_->publish(gait_state_msg);
  }

  if (open_loop_publisher_ != nullptr) {
    assert(MPC_PREDICTION_HORIZON == 20);  // fixed message size
    static interfaces::msg::PositionSequence open_loop_positions;
    for (unsigned int ol_idx = 0; ol_idx < MPC_PREDICTION_HORIZON; ol_idx++) {
      open_loop_positions.valid[ol_idx] = true;
      open_loop_positions.x[ol_idx] = mpc_prediction_.position[ol_idx].x();
      open_loop_positions.y[ol_idx] = mpc_prediction_.position[ol_idx].y();
      open_loop_positions.z[ol_idx] = mpc_prediction_.position[ol_idx].z();
      open_loop_positions.qw[ol_idx] = mpc_prediction_.orientation[ol_idx].w();
      open_loop_positions.qx[ol_idx] = mpc_prediction_.orientation[ol_idx].x();
      open_loop_positions.qy[ol_idx] = mpc_prediction_.orientation[ol_idx].y();
      open_loop_positions.qz[ol_idx] = mpc_prediction_.orientation[ol_idx].z();
    }
    open_loop_publisher_->publish(open_loop_positions);
  }

  if (PUBLISH_GAIT_SEQUENCE && (gait_sequence_publisher_ != nullptr)) {
    auto gs_msg = gait_sequence_to_msg(gait_sequence_);
    gait_sequence_publisher_->publish(gs_msg);
  }
}

void MITController::ModelAdaptationCallback() {
  if (use_model_adaptation_) {
    static QuadState quad_state_temp;
    quad_state_lock_.lock();
    quad_state_temp = quad_state_;
    quad_state_lock_.unlock();
    ma_->UpdateState(quad_state_temp);
    bool changed_model = ma_->DoModelAdaptation(quad_model_);
    if (changed_model) {
      RCLCPP_INFO(this->get_logger(), "Model Adaptation changed QuadModel");
      interfaces::msg::QuadModel quad_model_msg;
      mpc_lock_.lock();
      mpc_->UpdateModel(quad_model_);
      gs_->UpdateModel(quad_model_);
      mpc_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "Updated used Model in MPC and GS");
      slc_lock_.lock();
      slc_->UpdateModel(quad_model_);
      slc_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "Updated used Model in SLC");
      quad_model_msg.header.stamp = this->get_clock()->now();
      quad_model_msg.mass = quad_model_.GetMass();
      Eigen::Map<Eigen::Vector3d>(quad_model_msg.com.data()) = quad_model_.GetBodyToCOM().vector();
      Eigen::Map<Eigen::Matrix3d>(quad_model_msg.inertia.data()) = quad_model_.GetInertia();
      quad_model_publisher_->publish(quad_model_msg);
      RCLCPP_INFO(this->get_logger(), "Published Model update to other components");
      if (PUBLISH_HEARTBEAT) {
        controller_heartbeat_.num_model_updates++;
      }
    }
  }
}

void MITController::ControlLoopCallback() {
  // Quad state does not have to be locked, as it is running in a different
  // callback group This only makes sense to run, if the MPC and so on was
  // running at least once
  static WrenchSequence wrench_sequence_temp;
  static GaitSequence gait_sequence_temp;
  static FeetTargets feet_targets_temp;
  static QuadState quad_state_temp;
  static MPCPrediction mpc_prediction_temp;
  static std::array<double, ModelInterface::N_LEGS> feet_swing_progress_temp;
  static std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_temp;

  gs_wrench_sequence_lock_.lock();
  wrench_sequence_temp = wrench_sequence_;
  gait_sequence_temp = gait_sequence_;
  mpc_prediction_temp = mpc_prediction_;
  gs_wrench_sequence_lock_.unlock();
  targets_lock_.lock();
  feet_targets_temp = feet_targets_;
  feet_swing_progress_temp = feet_swing_progress_;
  feet_swing_states_temp = feet_swing_states_;
  targets_lock_.unlock();
  quad_state_lock_.lock();
  quad_state_temp = quad_state_;
  quad_state_lock_.unlock();

  wbc_lock_.lock();

  static double wbc_solve_time = 0.0;  // TODO: not static?
  auto wbc_start_time = std::chrono::high_resolution_clock::now();
  // Update WBC
  wbc_->UpdateState(quad_state_temp);
  //  wbc_->UpdateTarget(gait_sequence_temp.target_orientation,
  //                     gait_sequence_temp.target_position,
  //                     gait_sequence_temp.target_velocity,
  //                     gait_sequence_temp.target_twist);  // TODO: take MPC first prediction here?`

  wbc_->UpdateTarget(mpc_prediction_temp.orientation[1],
                     mpc_prediction_temp.position[1],
                     mpc_prediction_temp.linear_velocity[1],
                     mpc_prediction_temp.angular_velocity[1]);  // TODO: take MPC first prediction here?

  // Prepare for WBC
  auto wrenches = wrench_sequence_temp.forces[0];
  auto feet_targets = feet_targets_temp;
  auto gait = gait_sequence_temp.contact_sequence[0];

  // Update leg status
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    switch (feet_status_[leg_idx]) {
      case LOST_CONTACT:
        [[fallthrough]];
      case LATE_CONTACT:
        // Leave it in slipped and override anything until it gets contact again
        // or do the late contact movement until it gets contact again and ignore any planned stance
        if (quad_state_temp.GetFeetContacts()[leg_idx] and gait[leg_idx]) {
          // Regaining contact and schedule stance phase
          feet_status_[leg_idx] = STANCE;
        } else if (quad_state_temp.GetFeetContacts()[leg_idx] and !gait[leg_idx]) {
          // Regaining contact and scheduled flight phase
          feet_status_[leg_idx] = SWING;
        } else if (late_contact_reschedule_swing_phase_ and !gait[leg_idx])
        // Special feature to reschedule swing phase even if late contact (sometimes
        // legs are not properly touching the ground)
        {
          feet_status_[leg_idx] = SWING;
        }
        // Otherwise stay in that phase
        break;
      case EARLY_CONTACT:
        // Wait for scheduling to react to early contact
        if (gait[leg_idx]) {
          feet_status_[leg_idx] = STANCE;
        }
        break;
      case SWING:
        // Check for early or late contact:
        if (early_contact_detection_ and !gait[leg_idx] and quad_state_temp.GetFeetContacts()[leg_idx]
            and feet_swing_progress_temp[leg_idx] > 0.5) {  // Early contact
          feet_status_[leg_idx] = EARLY_CONTACT;
          early_contact_hold_position_[leg_idx] = quad_model_.GetFootPositionInWorld(leg_idx, quad_state_temp);
          RCLCPP_INFO(this->get_logger(), "Foot [%d] has early contact", leg_idx);
          if (PUBLISH_HEARTBEAT) {
            controller_heartbeat_.num_early_contacts++;
          }
        } else if (late_contact_detection_ and gait[leg_idx] and !quad_state_temp.GetFeetContacts()[leg_idx]) {
          feet_status_[leg_idx] = LATE_CONTACT;
          slip_hold_in_body_[leg_idx] = quad_model_.GetFootPositionInBodyFrame(
              leg_idx, Eigen::Map<const Eigen::Vector3d>(quad_state_temp.GetJointPositions()[leg_idx].data()));
          RCLCPP_INFO(this->get_logger(), "Foot [%d] has late contact", leg_idx);
        } else if (gait[leg_idx]) {
          feet_status_[leg_idx] = STANCE;  // rare case but this is the ideal one
        }
        break;
      case STANCE:
        // Check if flight phase scheduled or slip detected
        if (!gait[leg_idx]) {
          feet_status_[leg_idx] = SWING;  // Swing scheduled so going to swing phase
        } else if (lost_contact_detection_ and gait[leg_idx] and !quad_state_temp.GetFeetContacts()[leg_idx]) {
          feet_status_[leg_idx] = LOST_CONTACT;  // Slip detected going to slip
          slip_hold_in_body_[leg_idx] = quad_model_.GetFootPositionInBodyFrame(
              leg_idx, Eigen::Map<const Eigen::Vector3d>(quad_state_temp.GetJointPositions()[leg_idx].data()));
          RCLCPP_WARN(this->get_logger(),
                      "Foot [%d] lost contact and is kept a last contact position relative to body",
                      leg_idx);
        }
        break;
    }
  }

  //  RCLCPP_WARN_THROTTLE(
  //      this->get_logger(), this->get_clock(), 1.0, "Leg [%d] slipped, keeping position until it gets contact");
  // Apply leg commands
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    switch (feet_status_[leg_idx]) {
      case STANCE: {
        last_feet_pos_targets_[leg_idx] = gait_sequence_temp.foot_position_sequence[0][leg_idx];
        feet_targets.positions[leg_idx] = gait_sequence_temp.foot_position_sequence[0][leg_idx];
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
      } break;
      case SWING: {
        wrenches[leg_idx].setZero();
        // feet targets stay the same
        if ((feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::STANCE)
            or (feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::NOT_STARTED)) {
          // take last feet targets for now
          feet_targets.velocities[leg_idx].setZero();
          feet_targets.accelerations[leg_idx].setZero();
          feet_targets.positions[leg_idx] = last_feet_pos_targets_[leg_idx];
          RCLCPP_ERROR_EXPRESSION(this->get_logger(),
                                  feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::NOT_STARTED,
                                  "Controll loop reached Swing phase for foot [%d], but SLC is still in stance",
                                  leg_idx);
          RCLCPP_WARN_EXPRESSION(this->get_logger(),
                                 feet_swing_states_temp[leg_idx] == SwingLegControllerInterface::NOT_STARTED,
                                 "Controll loop reached Swing phase for foot [%d], but SLC has not yet started",
                                 leg_idx);
        }
      } break;
      case EARLY_CONTACT: {
        gait[leg_idx] = true;
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
        feet_targets.positions[leg_idx] = early_contact_hold_position_[leg_idx];
        // Next stance phase targets have to be transformed to current pose
        int next_stance_indx = int(gait_sequence_temp.swing_time_sequence[0][leg_idx] / MPC_DT) + 1;
        assert(gait_sequence_temp.contact_sequence[next_stance_indx][leg_idx] == true);
        wrenches[leg_idx] = quad_state_temp.GetOrientationInWorld()
                            * gait_sequence_temp.reference_trajectory_orientation[next_stance_indx].inverse()
                            * wrench_sequence_temp.forces[next_stance_indx][leg_idx];
      } break;
      case LOST_CONTACT: {
        // Keep foot on lost position
        // same behaviour as LATE_CONTACT for now
        [[fallthrough]];
      }
      case LATE_CONTACT: {
        gait[leg_idx] = false;
        feet_targets.velocities[leg_idx].setZero();
        feet_targets.accelerations[leg_idx].setZero();
        feet_targets.positions[leg_idx] = Eigen::Translation3d(quad_state_.GetPositionInWorld())
                                          * quad_state_.GetOrientationInWorld()
                                          * slip_hold_in_body_[leg_idx];  // Transformed to world
        wrenches[leg_idx].setZero();
      } break;
    }
  }

  // Set PD weights:
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    if (gait[leg_idx]) {  // Stance
      switch (leg_control_mode_) {
        case CARTESIAN_JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_joint_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_joint_control_stance_Kd_;
          break;
        case CARTESIAN_STIFFNESS_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_stiffness_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_stiffness_control_stance_Kd_;
          break;
        case JOINT_TORQUE_CONTROL:
          [[fallthrough]];
        case JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kp.data() + leg_idx * 3) = joint_control_stance_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kd.data() + leg_idx * 3) = joint_control_stance_Kd_;
          break;
      }
    } else {
      switch (leg_control_mode_) {  // Swing
        case CARTESIAN_JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_joint_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_joint_control_swing_Kd_;
          break;
        case CARTESIAN_STIFFNESS_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kp.data() + leg_idx * 3) = cartesian_stiffness_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_cmd_.kd.data() + leg_idx * 3) = cartesian_stiffness_control_swing_Kd_;
          break;
        case JOINT_TORQUE_CONTROL:
          [[fallthrough]];
        case JOINT_CONTROL:
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kp.data() + leg_idx * 3) = joint_control_swing_Kp_;
          Eigen::Map<Eigen::Vector3d>(leg_joint_cmd_.kd.data() + leg_idx * 3) = joint_control_swing_Kd_;
          break;
      }
    }
  }

  wbc_->UpdateFeetTarget(feet_targets);
  wbc_->UpdateFootContact(gait);
  wbc_->UpdateWrenches(wrenches);

  static WBCReturn wbc_return;
  switch (leg_control_mode_) {
    case CARTESIAN_JOINT_CONTROL:
      [[fallthrough]];
    case CARTESIAN_STIFFNESS_CONTROL: {
      leg_cmd_.header.stamp = this->get_clock()->now();
      CartesianCommands commands;
      wbc_return = reinterpret_cast<WBCInterface<CartesianCommands> *>(wbc_.get())->GetJointCommand(commands);
      assign(commands.position, leg_cmd_.ee_pos);
      assign(commands.velocity, leg_cmd_.ee_vel);
      assign(commands.force, leg_cmd_.ee_force);
      leg_cmd_publisher_->publish(leg_cmd_);
    } break;
    case JOINT_TORQUE_CONTROL:
      [[fallthrough]];
    case JOINT_CONTROL: {
      leg_joint_cmd_.header.stamp = this->get_clock()->now();
      JointTorqueVelocityPositionCommands commands;
      wbc_return =
          reinterpret_cast<WBCInterface<JointTorqueVelocityPositionCommands> *>(wbc_.get())->GetJointCommand(commands);
      assign(commands.velocity, leg_joint_cmd_.velocity);
      assign(commands.position, leg_joint_cmd_.position);
      assign(commands.torque, leg_joint_cmd_.effort);
      leg_joint_cmd_publisher_->publish(leg_joint_cmd_);
      break;
    }
  }
  auto wbc_end_time = std::chrono::high_resolution_clock::now();
  wbc_solve_time = std::chrono::duration_cast<std::chrono::duration<double>>(wbc_end_time - wbc_start_time).count();
  wbc_lock_.unlock();
  if (PUBLISH_WBC_SOLVE_TIME) {
    interfaces::msg::WBCReturn wbc_solve_time_message;
    wbc_solve_time_message.total_time = wbc_solve_time;
    wbc_solve_time_message.success = wbc_return.success;
    wbc_solve_time_message.qp_solve_time = wbc_return.qp_solve_time;
    wbc_solve_time_message.qp_update_time = wbc_return.qp_update_time;
    wbc_solve_time_message.header.stamp = this->get_clock()->now();
    wbc_solve_time_publisher_->publish(wbc_solve_time_message);
  }

  RCLCPP_ERROR_EXPRESSION(this->get_logger(), !wbc_return.success, "WBC solver did not converge");

  //  RCLCPP_WARN_EXPRESSION(this->get_logger(),
  //                         wbc_solve_time >= WBC_CYCLE_DT,
  //                         "WBC solver took longer [%f s] than MPC control dt [%f s]",
  //                         wbc_solve_time,
  //                         WBC_CYCLE_DT); //TODO: maybe enable!

  if (swing_leg_trajs_publisher_ != nullptr) {
    static interfaces::msg::VectorSequence swing_leg_traj;  // Only the pending and runnign (4 in total)
    std::array<Eigen::Vector3d, N_LEGS> start, end;
    slc_->GetCurrentTrajs(start, end);
    for (unsigned int foot_idx = 0; foot_idx < N_LEGS; foot_idx++) {
      Eigen::Vector3d vector_start_to_end = end[foot_idx] - start[foot_idx];
      swing_leg_traj.origin_x[foot_idx] = start[foot_idx].x();
      swing_leg_traj.origin_y[foot_idx] = start[foot_idx].y();
      swing_leg_traj.origin_z[foot_idx] = start[foot_idx].z();
      swing_leg_traj.vector_x[foot_idx] = vector_start_to_end.x();
      swing_leg_traj.vector_y[foot_idx] = vector_start_to_end.y();
      swing_leg_traj.vector_z[foot_idx] = vector_start_to_end.z();
    }
    swing_leg_trajs_publisher_->publish(swing_leg_traj);
  }

  if (PUBLISH_WBC_TARGET) {
    static interfaces::msg::WBCTarget wbc_target_msg_;
    wbc_target_msg_.header.stamp = this->get_clock()->now();
    eigenToRosMsg(mpc_prediction_temp.position[1], wbc_target_msg_.pose.position);
    eigenToRosMsg(mpc_prediction_temp.orientation[1], wbc_target_msg_.pose.orientation);
    eigenToRosMsg(mpc_prediction_temp.linear_velocity[1], wbc_target_msg_.twist.linear);
    eigenToRosMsg(mpc_prediction_temp.angular_velocity[1], wbc_target_msg_.twist.angular);
    wbc_target_msg_.feet_contacts = gait;
    eigenToRosMsg(wrenches, wbc_target_msg_.feet_wrenches);
    eigenToRosMsg(feet_targets.positions, wbc_target_msg_.feet_pos_targets);
    eigenToRosMsg(feet_targets.velocities, wbc_target_msg_.feet_vel_targets);
    eigenToRosMsg(feet_targets.accelerations, wbc_target_msg_.feet_acc_targets);
    wbc_target_publisher_->publish(wbc_target_msg_);
  }

  if (PUBLISH_HEARTBEAT) {
    if (wbc_solve_time >= WBC_CYCLE_DT) {
      controller_heartbeat_.num_wbc_overtime++;
    }
    if (!wbc_return.success) {
      controller_heartbeat_.num_wbc_solver_fail++;
    }
  }
}

void MITController::SLCLoopCallback() {
  static QuadState quad_state_temp;
  static GaitSequence gs_tmp;
  static std::array<double, ModelInterface::N_LEGS> feet_swing_progress_temp;
  static std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_temp;
  static FeetTargets feet_targets_temp;
  slc_lock_.lock();
  gs_wrench_sequence_lock_.lock();
  if (gs_updated_) {
    gs_tmp = gait_sequence_;
    gs_wrench_sequence_lock_.unlock();
    slc_->UpdateGaitSequence(gs_tmp);
  } else {
    gs_wrench_sequence_lock_.unlock();
  }

  quad_state_lock_.lock();
  quad_state_temp = quad_state_;
  quad_state_lock_.unlock();

  slc_->UpdateState(quad_state_temp);
  slc_->GetFeetTargets(feet_targets_temp);
  slc_->GetProgress(feet_swing_progress_temp, feet_swing_states_temp);

  targets_lock_.lock();
  feet_targets_ = feet_targets_temp;
  feet_swing_progress_ = feet_swing_progress_temp;
  feet_swing_states_ = feet_swing_states_temp;
  targets_lock_.unlock();
  slc_lock_.unlock();
}

void MITController::HartbeatCallback() {
  controller_heartbeat_.header.stamp = this->get_clock()->now();
  controller_heartbeat_publisher_->publish(controller_heartbeat_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MITController>("mit_controller_node");
  rclcpp::ExecutorOptions exopt;
  rclcpp::executors::MultiThreadedExecutor executor(exopt, 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

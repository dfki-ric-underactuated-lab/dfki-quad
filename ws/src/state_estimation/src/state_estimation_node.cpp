#include "state_estimation_node.hpp"

StateEstimationNode::StateEstimationNode(std::shared_ptr<ModelInterface> &quadModel)
    : rclcpp::Node("state_estimation_node"),
      quad_model_(quadModel),
      last_angular_velocity_(Eigen::Vector3d ::Zero()),
      joint_vel_filter_(FILTER_WINDOW),
      joint_acc_filter_(FILTER_WINDOW),
      joint_tau_filter_(FILTER_WINDOW),
      angular_vel_filter_(IMU_FILTER_WINDOW),
      linear_acc_filter_(IMU_FILTER_WINDOW),
      vicon_pos_filter_(VICON_FILTER_WINDOW),
      vicon_orient_filter_(VICON_FILTER_WINDOW),
      first_orientation_from_raw_imu_set_(false),
      kalman_filter_(nullptr) {
  // init first angular velocity
  last_joint_velocity_.fill(0.0);
  this->declare_parameter("kalman_update_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("state_publish_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("state_lf_publish_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("laying_on_ground_detection_offset", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("kalman.gyroscope_std", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("kalman.accelerometer_std", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("kalman.gyroscope_bias_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.accelerometer_bias_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.contact_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.belly_contact_point_measurement_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.prior.gyroscope_bias", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("kalman.prior.accelerometer_bias", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("kalman.prior.base_orientation_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.prior.base_velocity_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.prior.base_position_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.prior.gyroscope_bias_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.prior.accelerometer_bias_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.foot_step_measurement_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.foot_on_plane_std", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.imu_measures_g", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("kalman.assume_planar_ground", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("kalman.adapt_covariances", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("kalman.min_stance_percentage_for_contact", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("kalman.use_only_gait_contacts", false);

  assert(this->get_parameter("kalman.gyroscope_std").as_double_array().size() == 3);
  assert(this->get_parameter("kalman.accelerometer_std").as_double_array().size() == 3);
  assert(this->get_parameter("kalman.prior.gyroscope_bias").as_double_array().size() == 3);
  assert(this->get_parameter("kalman.prior.accelerometer_bias").as_double_array().size() == 3);
  kalman_params_ = KalmanFilter::Params{
      Eigen::Map<const Eigen::Vector3d>(this->get_parameter("kalman.gyroscope_std").as_double_array().data()),
      Eigen::Map<const Eigen::Vector3d>(this->get_parameter("kalman.accelerometer_std").as_double_array().data()),
      this->get_parameter("kalman.gyroscope_bias_std").as_double(),
      this->get_parameter("kalman.accelerometer_bias_std").as_double(),
      this->get_parameter("kalman.contact_std").as_double(),
      this->get_parameter("kalman.foot_on_plane_std").as_double(),
      Eigen::Map<const Eigen::Vector3d>(this->get_parameter("kalman.prior.gyroscope_bias").as_double_array().data()),
      Eigen::Map<const Eigen::Vector3d>(
          this->get_parameter("kalman.prior.accelerometer_bias").as_double_array().data()),
      this->get_parameter("kalman.prior.base_orientation_std").as_double(),
      this->get_parameter("kalman.prior.base_velocity_std").as_double(),
      this->get_parameter("kalman.prior.base_position_std").as_double(),
      this->get_parameter("kalman.prior.gyroscope_bias_std").as_double(),
      this->get_parameter("kalman.prior.accelerometer_bias_std").as_double(),
      this->get_parameter("kalman.foot_step_measurement_std").as_double(),
      this->get_parameter("kalman.belly_contact_point_measurement_std").as_double(),
      this->get_parameter("kalman.imu_measures_g").as_bool(),
      this->get_parameter("kalman.assume_planar_ground").as_bool(),
      this->get_parameter("kalman.adapt_covariances").as_bool()};
  min_stance_percentage_for_contact_ = this->get_parameter("kalman.min_stance_percentage_for_contact").as_double();
  use_only_gait_contacts_ = this->get_parameter("kalman.use_only_gait_contacts").as_bool();

  this->declare_parameter("contact_detection.force_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("contact_detection.use_measured_forces", rclcpp::ParameterType::PARAMETER_BOOL);

  this->declare_parameter("contact_detection.use_energy_obs_contact_detection", rclcpp::ParameterType::PARAMETER_BOOL);
  this->declare_parameter("contact_detection.energy_obs_kd", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("contact_detection.energy_obs_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("contact_detection.leg_in_motion_joint_velocity_threshold",
                          rclcpp::ParameterType::PARAMETER_DOUBLE);

  this->declare_parameter("contact_detection.update_threshold", false);
  this->declare_parameter("contact_detection.threshold_update_rate", 0.05);
  this->declare_parameter("contact_detection.threshold_offset", 2.5);
  this->declare_parameter("contact_detection.max_threshold_offset", 2.5);
  this->declare_parameter("contact_detection.threshold_filter_size", 60);
  this->declare_parameter("contact_detection.max_threshold_filter_size", 40);

  this->declare_parameter("contact_detection.energy_obs_filter_size", rclcpp::ParameterType::PARAMETER_INTEGER);
  this->declare_parameter("contact_detection.leg_swing_time", rclcpp::ParameterType::PARAMETER_DOUBLE);
  assert(this->get_parameter("contact_detection.force_threshold").as_double_array().size() == ModelInterface::N_LEGS);

  contact_detection_ = new ContactDetection(
      quadModel,
      to_array<ModelInterface::N_LEGS>(this->get_parameter("contact_detection.force_threshold").as_double_array()),
      this->get_parameter("contact_detection.energy_obs_kd").as_double(),
      this->get_parameter("contact_detection.energy_obs_threshold").as_double(),
      this->get_parameter("contact_detection.leg_in_motion_joint_velocity_threshold").as_double(),
      this->get_parameter("contact_detection.use_energy_obs_contact_detection").as_bool(),
      this->get_parameter("contact_detection.leg_swing_time").as_double(),
      this->get_parameter("contact_detection.energy_obs_filter_size").as_int());

  contact_detection_force_threshold_ =
      to_array<ModelInterface::N_LEGS>(this->get_parameter("contact_detection.force_threshold").as_double_array());
  if (this->get_parameter("contact_detection.use_measured_forces").as_bool()) {
    contact_state_subscription_ = this->create_subscription<interfaces::msg::ContactState>(
        "contact_state", QOS_BEST_EFFORT_NO_DEPTH, [this](const interfaces::msg::ContactState &msg) {
          for (int i = 0; i < ModelInterface::N_LEGS; i++) {
            ground_contact_force_[i] = msg.ground_contact_force[i];
            quad_state_msg_.foot_contact[i] = msg.ground_contact_force[i] >= contact_detection_force_threshold_[i];
          }
        });
  }
  assert(this->get_parameter("laying_on_ground_detection_offset").as_double_array().size() == ModelInterface::N_LEGS);
  laying_on_ground_detection_offset_ =
      to_array<ModelInterface::N_LEGS>(this->get_parameter("laying_on_ground_detection_offset").as_double_array());

  joint_states_subscription_ = this->create_subscription<interfaces::msg::JointState>(
      "joint_states",
      QOS_BEST_EFFORT_NO_DEPTH,
      std::bind(&StateEstimationNode::JointStatesCallback, this, std::placeholders::_1));
  this->declare_parameter("replace_kalman_filter_by_vicon", false);
  this->declare_parameter("vicon_topic", rclcpp::ParameterType::PARAMETER_STRING);
  replace_kalman_filter_by_vicon_ = this->get_parameter("replace_kalman_filter_by_vicon").as_bool();
#ifndef WITH_VICON
  replace_kalman_filter_by_vicon_ = false;
#endif
  if (!replace_kalman_filter_by_vicon_) {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_measurement",
        QOS_BEST_EFFORT_NO_DEPTH,
        std::bind(&StateEstimationNode::ImuCallback, this, std::placeholders::_1));
  }
#ifdef WITH_VICON
  else {
    vicon_receiver_ = this->create_subscription<vicon_receiver::msg::Position>(
        this->get_parameter("vicon_topic").as_string(),
        QOS_BEST_EFFORT_NO_DEPTH,
        std::bind(&StateEstimationNode::ViconCallback, this, std::placeholders::_1));
  }
#endif

  gait_state_subscription_ = this->create_subscription<interfaces::msg::GaitState>(
      "gait_state",
      QOS_BEST_EFFORT_NO_DEPTH,
      std::bind(&StateEstimationNode::GaitStateCallback, this, std::placeholders::_1));
  // init gait state message with standing gait
  gait_state_ = interfaces::msg::GaitState();
  for (int i = 0; i < ModelInterface::N_LEGS; ++i) {
    gait_state_.contact[i] = true;
    gait_state_.duty_factor[i] = 1.0;
  }

  quad_state_publisher_ = this->create_publisher<interfaces::msg::QuadState>("quad_state", QOS_RELIABLE_NO_DEPTH);
  quad_state_lf_publisher_ =
      this->create_publisher<interfaces::msg::QuadState>("quad_state/lf", QOS_BEST_EFFORT_NO_DEPTH);

  reset_state_estimation_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_state_estimation",
      std::bind(
          &StateEstimationNode::ResetStateEstimationCallback, this, std::placeholders::_1, std::placeholders::_2));

  reset_state_estimation_convariances_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_state_estimation_covariances",
      std::bind(&StateEstimationNode::ResetStateEstimationCovariancesCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  std::cout << "IMU OFFSET: " << quad_model_->GetBodyToIMU() * Eigen::Vector3d{0., 0., 0.} << std::endl;

  threshold_offset_ = this->get_parameter("contact_detection.threshold_offset").as_double();
  max_threshold_offset_ = this->get_parameter("contact_detection.max_threshold_offset").as_double();
  if (this->get_parameter("contact_detection.update_threshold").as_bool()) {
    int size_min_filter = this->get_parameter("contact_detection.threshold_filter_size").as_int();
    int size_max_filter = this->get_parameter("contact_detection.max_threshold_filter_size").as_int();
    for (int i = 0; i < ModelInterface::N_LEGS; ++i) {
      contact_detection_force_filter_[i] = MovingMin<double>(size_min_filter);
      max_contact_detection_force_filter_[i] = MovingMax<double>(size_max_filter);
    }

    contact_threshold_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::duration<double>(this->get_parameter("contact_detection.threshold_update_rate").as_double()),
        std::bind(&StateEstimationNode::ContactThresholdCallback, this));
  }
}

void StateEstimationNode::JointStatesCallback(const interfaces::msg::JointState &joint_msg) {
  // Make sure that the messages have the right size
  if (joint_msg.position.size() == ModelInterface::N_LEGS * ModelInterface::N_JOINTS_PER_LEG
      and joint_msg.velocity.size() == ModelInterface::N_LEGS * ModelInterface::N_JOINTS_PER_LEG
      // and joint_msg.acceleration.size() == ModelInterface::N_LEGS * ModelInterface::N_JOINTS_PER_LEG
      and joint_msg.effort.size() == ModelInterface::N_LEGS * ModelInterface::N_JOINTS_PER_LEG) {
    quad_state_msg_.header.stamp = joint_msg.header.stamp;  // Set time stamp according to joint measurement
    quad_state_msg_.joint_state.position = joint_msg.position;
    // // joint acceleration and velocity
    auto current_joint_time = QuadState::TimePoint(
        QuadState::TimePoint::duration(joint_msg.header.stamp.nanosec)
        + std::chrono::duration_cast<QuadState::TimePoint::duration>(std::chrono::seconds(joint_msg.header.stamp.sec)));
    quad_state_msg_.joint_state.velocity = joint_vel_filter_.update(joint_msg.velocity);
    auto acc =
        (joint_vel_filter_.get() - last_joint_velocity_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(current_joint_time - last_joint_time_).count();
    quad_state_msg_.joint_state.acceleration = joint_acc_filter_.update(acc);
    contact_detection_->Update(
        quad_state_msg_.joint_state,
        std::chrono::duration_cast<std::chrono::duration<double>>(current_joint_time - last_joint_time_).count());
    last_joint_time_ = current_joint_time;
    last_joint_velocity_ = joint_vel_filter_.get();
    quad_state_msg_.joint_state.effort = joint_tau_filter_.update(joint_msg.effort);
    std::array<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> contact_forces_map = {
        Eigen::Map<Eigen::Vector3d>(&quad_state_msg_.ground_contact_force[0 * ModelInterface::N_JOINTS_PER_LEG]),
        Eigen::Map<Eigen::Vector3d>(&quad_state_msg_.ground_contact_force[1 * ModelInterface::N_JOINTS_PER_LEG]),
        Eigen::Map<Eigen::Vector3d>(&quad_state_msg_.ground_contact_force[2 * ModelInterface::N_JOINTS_PER_LEG]),
        Eigen::Map<Eigen::Vector3d>(&quad_state_msg_.ground_contact_force[3 * ModelInterface::N_JOINTS_PER_LEG])};
    static std::array<double, ModelInterface::N_LEGS> unused;
    if (contact_state_subscription_ != nullptr) {
      contact_detection_->GetContacts(contact_forces_map, unused);
    } else {
      contact_detection_->GetContacts(quad_state_msg_.foot_contact, contact_forces_map, unused);
    }

    bool no_foot_contact = true;
    for (unsigned int i = 0; i < ModelInterface::N_LEGS; i++) {
      if (quad_state_msg_.foot_contact[i]) {
        no_foot_contact = false;
        break;
      }
    }
    // Detect belly in contact
    quad_state_msg_.belly_contact =
        quad_model_->IsLyingDown({Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                                                    + 0 * ModelInterface::N_JOINTS_PER_LEG),
                                  Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                                                    + 1 * ModelInterface::N_JOINTS_PER_LEG),
                                  Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                                                    + 2 * ModelInterface::N_JOINTS_PER_LEG),
                                  Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                                                    + 3 * ModelInterface::N_JOINTS_PER_LEG)},
                                 laying_on_ground_detection_offset_);

    std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> feet_positions{
        (quad_model_->GetBodyToIMU().inverse()
         * quad_model_->GetFootPositionInBodyFrame(
             0,
             Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                               + 0 * ModelInterface::N_JOINTS_PER_LEG))),
        (quad_model_->GetBodyToIMU().inverse()
         * quad_model_->GetFootPositionInBodyFrame(
             1,
             Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                               + 1 * ModelInterface::N_JOINTS_PER_LEG))),
        (quad_model_->GetBodyToIMU().inverse()
         * quad_model_->GetFootPositionInBodyFrame(
             2,
             Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                               + 2 * ModelInterface::N_JOINTS_PER_LEG))),
        (quad_model_->GetBodyToIMU().inverse()
         * quad_model_->GetFootPositionInBodyFrame(
             3,
             Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                               + 3 * ModelInterface::N_JOINTS_PER_LEG))),
    };

    if (kalman_update_step_timer_ == nullptr and first_orientation_from_raw_imu_set_) {
      RCLCPP_INFO(this->get_logger(), "Received first joint message, starting kalman update loop and publishing state");
      kalman_update_step_timer_ =
          rclcpp::create_timer(this,
                               this->get_clock(),
                               std::chrono::duration<double>(this->get_parameter("kalman_update_rate").as_double()),
                               std::bind(&StateEstimationNode::KalmanUpdateCallback, this));
      quad_state_publish_timer_ =
          rclcpp::create_timer(this,
                               this->get_clock(),
                               std::chrono::duration<double>(this->get_parameter("state_publish_rate").as_double()),
                               std::bind(&StateEstimationNode::PublishStateCallback, this));
      quad_state_lf_publish_timer_ =
          rclcpp::create_timer(this,
                               this->get_clock(),
                               std::chrono::duration<double>(this->get_parameter("state_lf_publish_rate").as_double()),
                               std::bind(&StateEstimationNode::PublishStateLfCallback, this));

      auto init_pose = Eigen::Vector3d(0, 0, 0);
      std::array<const Eigen::Vector3d, ModelInterface::N_LEGS> joint_states{
          Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()),
          Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                            + 1 * ModelInterface::N_JOINTS_PER_LEG),
          Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                            + 2 * ModelInterface::N_JOINTS_PER_LEG),
          Eigen::Map<const Eigen::Vector3d>(quad_state_msg_.joint_state.position.data()
                                            + 3 * ModelInterface::N_JOINTS_PER_LEG),
      };
      //
      //      Eigen::Quaterniond init_orient(quad_state_msg_.pose.pose.orientation.w,
      //                                     quad_state_msg_.pose.pose.orientation.x,
      //                                     quad_state_msg_.pose.pose.orientation.y,
      //                                     quad_state_msg_.pose.pose.orientation.z);

      Eigen::Quaterniond init_orient(1.0, 0., 0., 0.);
      quad_model_->CalcBaseHeight({true, true, true, true},  // Assuming all feet are in contact
                                  joint_states,
                                  init_orient,
                                  init_pose(2));
      kalman_filter_ = new KalmanFilter(quad_model_,
                                        (Eigen::Translation3d(init_pose) * quad_model_->GetBodyToIMU()).translation(),
                                        init_orient,
                                        kalman_params_);
      RCLCPP_INFO(this->get_logger(),
                  "Kalman filter initialized at: [x %f, y %f,z %f] with [w %f, x %f, y %f, z %f]",
                  init_pose.x(),
                  init_pose.y(),
                  init_pose.z(),
                  init_orient.w(),
                  init_orient.x(),
                  init_orient.y(),
                  init_orient.z());
    }

    if (kalman_filter_ != nullptr) {
      std::array<const Eigen::Vector3d, 4> belly_contact_positions{
          (quad_model_->GetBodyToBellyBottom(0).translation()
           + Eigen::Vector3d(0., 0., laying_on_ground_detection_offset_[0])),
          (quad_model_->GetBodyToBellyBottom(1).translation()
           + Eigen::Vector3d(0., 0., laying_on_ground_detection_offset_[1])),
          (quad_model_->GetBodyToBellyBottom(2).translation()
           + Eigen::Vector3d(0., 0., laying_on_ground_detection_offset_[2])),
          (quad_model_->GetBodyToBellyBottom(3).translation()
           + Eigen::Vector3d(0., 0., laying_on_ground_detection_offset_[3])),
      };

      // Adapt the covariance matrices in the Kalman filter
      if (kalman_filter_->params_.adapt_covariances_) {
        kalman_filter_->AdaptCovariances(quad_state_msg_.foot_contact, feet_positions, contact_forces_map);
      }

      // get contacts for kalman filter; should not be true if swing phase scheduled
      std::array<bool, ModelInterface::N_LEGS> contacts;
      for (int i = 0; i < ModelInterface::N_LEGS; ++i) {
        contacts[i] =
            (quad_state_msg_.foot_contact[i] || use_only_gait_contacts_)
            && (gait_state_.contact[i]
                && (((gait_state_.phase[i] / gait_state_.duty_factor[i]) >= min_stance_percentage_for_contact_)
                    || (gait_state_.duty_factor[i] >= (1.0 - std::numeric_limits<double>::epsilon()))));
      }

      kalman_filter_->Update(
          contacts, feet_positions, no_foot_contact and quad_state_msg_.belly_contact, belly_contact_positions);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Receiving joint_msgs with wrong array sizes. Ignoring this messages");
  }
}

void StateEstimationNode::ImuCallback(const sensor_msgs::msg::Imu &imu_msg) {
  auto current_imu_time = QuadState::TimePoint(
      QuadState::TimePoint::duration(imu_msg.header.stamp.nanosec)
      + std::chrono::duration_cast<QuadState::TimePoint::duration>(std::chrono::seconds(imu_msg.header.stamp.sec)));

  auto filtered_ang_vel =
      angular_vel_filter_.update({imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z});
  auto filtered_linear_acc = linear_acc_filter_.update(
      {imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z});
  auto ang_acc = (filtered_ang_vel - last_angular_velocity_)
                 / std::chrono::duration_cast<std::chrono::duration<double>>(current_imu_time - last_imu_time_).count();
  if (kalman_filter_ == nullptr) {
    RCLCPP_INFO(this->get_logger(),
                "Received first imu message, kalman filter needs joint state as well to initialize");
    quad_state_msg_.header.stamp = imu_msg.header.stamp;  // Set time stamp according to IMU measurement
    quad_state_msg_.pose.pose.orientation = imu_msg.orientation;
    first_orientation_from_raw_imu_set_ = true;
  } else {
    kalman_filter_->Predict(imu_msg);
    // if kalman filter is running we have an orientation and can properly orient all the stuff
    Eigen::Quaterniond orientation;
    kalman_filter_->GetOrientation(orientation);
    if (kalman_params_.imu_measures_g_ == true) {
      Eigen::Vector3d linear_acc = orientation * filtered_linear_acc;
      linear_acc.z() -= quad_model_->GetG();
      eigenToRosMsg(linear_acc, quad_state_msg_.acceleration.linear);
    } else {
      eigenToRosMsg(orientation * filtered_linear_acc, quad_state_msg_.acceleration.linear);
    }

    Eigen::Quaterniond angular_vel_quat(0.0, filtered_ang_vel.x(), filtered_ang_vel.y(), filtered_ang_vel.z());
    Eigen::Quaterniond orientation_der = angular_vel_quat * orientation;
    orientation_der.coeffs() = 0.5 * orientation_der.coeffs();  // derivative of orientation quaternion
    eigenToRosMsg(orientation_der * ang_acc, quad_state_msg_.acceleration.angular);
    eigenToRosMsg(orientation * filtered_ang_vel, quad_state_msg_.twist.twist.angular);
  }
  last_angular_velocity_ = filtered_ang_vel;
  last_imu_time_ = current_imu_time;
}

#ifdef WITH_VICON
void StateEstimationNode::ViconCallback(const vicon_receiver::msg::Position &vicon_msg) {
  auto now = QuadState::TimePoint(std::chrono::nanoseconds(this->get_clock()->now().nanoseconds()));

  Eigen::Vector3d position{vicon_msg.x_trans / 1000., vicon_msg.y_trans / 1000., vicon_msg.z_trans / 1000.};
  Eigen::Quaterniond orientation{vicon_msg.w, vicon_msg.x_rot, vicon_msg.y_rot, vicon_msg.z_rot};

  auto last_position = vicon_pos_filter_.get();
  auto last_orient = vicon_orient_filter_.get();
  vicon_pos_filter_.update(position);
  Eigen::Quaterniond filtered_orient = vicon_orient_filter_.update(orientation);

  // TODO:!
  quad_state_msg_.header.stamp = this->get_clock()->now();
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_vicon_time_).count();

  Eigen::Vector3d angular_vel{2. / dt
                                  * (last_orient.w() * filtered_orient.x() - last_orient.x() * filtered_orient.w()
                                     - last_orient.y() * filtered_orient.z() + last_orient.z() * filtered_orient.y()),
                              2. / dt
                                  * (last_orient.w() * filtered_orient.y() + last_orient.x() * filtered_orient.z()
                                     - last_orient.y() * filtered_orient.w() - last_orient.z() * filtered_orient.x()),
                              2. / dt
                                  * (last_orient.w() * filtered_orient.z() - last_orient.x() * filtered_orient.y()
                                     + last_orient.y() * filtered_orient.x() - last_orient.z() * filtered_orient.w())};
  Eigen::Vector3d linear_vel = (vicon_pos_filter_.get() - last_position) / dt;
  Eigen::Vector3d linear_acc = (linear_vel - last_vicon_linear_vel_) / dt;
  Eigen::Vector3d angular_acc = (angular_vel - last_vicon_angular_vel_) / dt;

  eigenToRosMsg(position, quad_state_msg_.pose.pose.position);
  eigenToRosMsg(orientation, quad_state_msg_.pose.pose.orientation);
  eigenToRosMsg(linear_vel, quad_state_msg_.twist.twist.linear);
  eigenToRosMsg(angular_vel, quad_state_msg_.twist.twist.angular);
  eigenToRosMsg(linear_acc, quad_state_msg_.acceleration.linear);
  eigenToRosMsg(angular_acc, quad_state_msg_.acceleration.angular);

  last_vicon_time_ = now;
  last_vicon_linear_vel_ = linear_vel;
  last_vicon_angular_vel_ = angular_vel;
  quad_state_publisher_->publish(quad_state_msg_);
}
#endif

void StateEstimationNode::KalmanUpdateCallback() {}

void StateEstimationNode::PublishStateCallback() {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
  kalman_filter_->GetState(position, orientation, linear_velocity);
  Eigen::Matrix<double, 6, 6> pose_cov;
  Eigen::Matrix3d lin_vel_cov;
  kalman_filter_->GetCovariances(pose_cov, lin_vel_cov);

  eigenToRosMsg((Eigen::Translation3d(position) * quad_model_->GetBodyToIMU().inverse()).translation(),
                quad_state_msg_.pose.pose.position);  // TODO: wherelese to rotate this?
  eigenToRosMsg(orientation, quad_state_msg_.pose.pose.orientation);
  eigenToRosMsg(linear_velocity, quad_state_msg_.twist.twist.linear);

  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::StorageOptions::RowMajor>>(quad_state_msg_.pose.covariance.data()) =
      pose_cov;
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::StorageOptions::RowMajor>>(quad_state_msg_.twist.covariance.data())
      .block<3, 3>(0, 0) = lin_vel_cov;

  // quad_state_msg_.header.stamp = this->get_clock()->now();  // TODO: which time to set?
  quad_state_publisher_->publish(quad_state_msg_);
}

void StateEstimationNode::PublishStateLfCallback() { quad_state_lf_publisher_->publish(quad_state_msg_); }

void StateEstimationNode::ResetStateEstimationCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                       std_srvs::srv::Trigger::Response::SharedPtr response) {
  (void)request;  // unused since it is empty

  kalman_filter_->Reset();
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Reset of state estimation performed");
}

void StateEstimationNode::ResetStateEstimationCovariancesCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
  (void)request;  // unused sicne it is empty

  kalman_filter_->ResetCovariances();
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Reset of state estimation covariances performed");
}

void StateEstimationNode::ContactThresholdCallback() {
  for (int i = 0; i < ModelInterface::N_LEGS; ++i) {
    contact_detection_force_threshold_[i] =
        std::min(contact_detection_force_filter_[i].update(ground_contact_force_[i]) + threshold_offset_,
                 max_contact_detection_force_filter_[i].update(ground_contact_force_[i])
                     - max_threshold_offset_);  // TODO: use standard deviation instead of comparing to max
  }
}

void StateEstimationNode::GaitStateCallback(const interfaces::msg::GaitState &gait_state_msg) {
  gait_state_ = gait_state_msg;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ModelInterface> quad_model = std::make_shared<QuadModelSymbolic>();
  auto node = std::make_shared<StateEstimationNode>(quad_model);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

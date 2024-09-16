//  Implements symbolic leg driver that works in leg base frame

#include "leg_driver.hpp"

#include <eigen3/Eigen/src/Core/DiagonalMatrix.h>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <common/sequence_containers.hpp>

LegDriver::LegDriver()
    : Node("leg_driver"),
      quad_state_received_(false),
      first_message_received_(false),
      state_(INIT_WAIT),
      msg_repeat_(0) {
  // Parameters from config file
  this->declare_parameter("torque_limit", 16.0);
  this->declare_parameter("update_freq", 400.0);
  this->declare_parameter("max_msg_repeat",
                          20);                 // max amount cycles without new msg until going into damping mode
  this->declare_parameter("kd_damping", 5.0);  // kd value of damping mode
  this->declare_parameter("control_mode",
                          0);  // 0: joint control; 1: joint torque control; 2: cartesian
                               // stiffness control; 3 cartesian joint control
  this->declare_parameter("num_legs",
                          4);  // 0: joint control; 1: joint torque control; 2: cartesian
                               // stiffness control; 3 cartesian joint control
  this->declare_parameter("default_keep_joint_positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("keep_joint_pose_Kp", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->declare_parameter("keep_joint_pose_Kd", rclcpp::ParameterType::PARAMETER_DOUBLE);
  assert(this->get_parameter("default_keep_joint_positions").as_double_array().size() == ModelInterface::NUM_JOINTS);
  default_keep_joint_pos_ =
      to_array<ModelInterface::NUM_JOINTS>(this->get_parameter("default_keep_joint_positions").as_double_array());
  assert(default_keep_joint_pos_.size() == 12);
  init_Kp_ = this->get_parameter("keep_joint_pose_Kp").as_double();
  init_Kd_ = this->get_parameter("keep_joint_pose_Kd").as_double();

  update_freq_ = this->get_parameter("update_freq").get_parameter_value().get<float>();
  max_msg_repeat_ = this->get_parameter("max_msg_repeat").get_parameter_value().get<unsigned int>();
  damping_kd_ = this->get_parameter("kd_damping").get_parameter_value().get<float>();
  torque_limit_ = this->get_parameter("torque_limit").get_parameter_value().get<double>();
  operation_mode_ = static_cast<OperationMode>(this->get_parameter("control_mode").get_parameter_value().get<int>());

  // Publishers
  joint_cmd_goal_pub_ = this->create_publisher<interfaces::msg::JointCmd>("joint_cmd", QOS_RELIABLE_NO_DEPTH);

  // Subscribers
  quad_state_sub_ = this->create_subscription<interfaces::msg::QuadState>(
      "quad_state", QOS_RELIABLE_NO_DEPTH, std::bind(&LegDriver::handle_quad_state_update, this, _1));

  set_keep_joint_pos_srv_ = this->create_service<interfaces::srv::KeepJointPositions>(
      "keep_joint_pose", std::bind(&LegDriver::handle_keep_pose, this, _1, _2));
  set_daming_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "set_damping_mode", std::bind(&LegDriver::handle_damping, this, _1, _2));
  set_energ_daming_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "set_emergency_damping_mode", std::bind(&LegDriver::handle_emerg_damping, this, _1, _2));
  switch_op_mode_srv_ = this->create_service<interfaces::srv::ChangeLegDriverMode>(
      "switch_op_mode", std::bind(&LegDriver::handle_change_mode, this, _1, _2));

  this->registerSubscribersAccordingToOperatioMode();

  update_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::microseconds(static_cast<int>(1e6 / update_freq_)), [this]() {
        switch (state_) {
          case OPERATE:
            do_control();
            break;
          case KEEP_POSE:
            do_keep_pose();
            break;
          case DAMPING:
            do_damping();
            break;
          case EMERGENCY_DAMPING:
            do_emergency_damp();
            break;
          case INIT_WAIT:
            do_init_wait();
            break;
        }
      });
}

void LegDriver::do_control() {
  joint_cmd_goal_ = interfaces::msg::JointCmd();
  joint_cmd_goal_.header.stamp = this->now();

  if (msg_repeat_ >= max_msg_repeat_) {  // watch dog, goto damping
    RCLCPP_WARN(this->get_logger(), "Watch dog triggered damping");
    switchState(DAMPING);
    return;
  }

  switch (operation_mode_) {
    case LegDriver::OperationMode::kCartesianJointControl:
      if (leg_cmd_.ee_pos.size() >= ModelInterface::N_LEGS) {
        for (unsigned int leg = 0; leg < ModelInterface::N_LEGS; leg++) {
          Eigen::Vector3d tau;
          Eigen::Vector3d f_e;
          Eigen::Vector3d f_d;
          Eigen::Vector3d x_d;
          Eigen::Vector3d q_d;
          Eigen::Vector3d v_d;
          Eigen::Vector3d qd_d;
          Eigen::Vector3d q_init_guess;
          x_d[0] = leg_cmd_.ee_pos[leg * 3 + 0];
          x_d[1] = leg_cmd_.ee_pos[leg * 3 + 1];
          x_d[2] = leg_cmd_.ee_pos[leg * 3 + 2];

          v_d[0] = leg_cmd_.ee_vel[leg * 3 + 0];
          v_d[1] = leg_cmd_.ee_vel[leg * 3 + 1];
          v_d[2] = leg_cmd_.ee_vel[leg * 3 + 2];

          f_d[0] = leg_cmd_.ee_force[leg * 3 + 0];
          f_d[1] = leg_cmd_.ee_force[leg * 3 + 1];
          f_d[2] = leg_cmd_.ee_force[leg * 3 + 2];

          q_init_guess << quad_state_.GetJointPositions()[leg][0], quad_state_.GetJointPositions()[leg][1],
              quad_state_.GetJointPositions()[leg][2];
          quad_model_.calcLegInverseKinematicsInBody(leg, x_d, q_init_guess, q_d);
          quad_model_.calcLegDiffKinematicsBodyFrame(leg, quad_state_, f_d, v_d, tau, qd_d);

          for (unsigned int j = 0; j < ModelInterface::N_JOINTS_PER_LEG; j++) {
            tau[j] = std::max(-torque_limit_, std::min(torque_limit_, tau[j]));
            joint_cmd_goal_.position[leg * ModelInterface::N_JOINTS_PER_LEG + j] = q_d[j];
            joint_cmd_goal_.velocity[leg * ModelInterface::N_JOINTS_PER_LEG + j] = qd_d[j];
            joint_cmd_goal_.effort[leg * ModelInterface::N_JOINTS_PER_LEG + j] = tau[j];
          }
        }
        joint_cmd_goal_.kp = leg_cmd_.kp;
        joint_cmd_goal_.kd = leg_cmd_.kd;
      }
      break;
    case LegDriver::OperationMode::kCartesianStiffnessControl:
      if (leg_cmd_.ee_pos.size() >= ModelInterface::N_LEGS) {
        joint_cmd_goal_.kp.fill(0);
        joint_cmd_goal_.kd.fill(0);
        for (unsigned int leg = 0; leg < ModelInterface::N_LEGS; leg++) {
          Eigen::Matrix3d J;
          Eigen::Vector3d tau;
          Eigen::Vector3d f_e;
          Eigen::Vector3d f_d;
          Eigen::Vector3d x_d;
          Eigen::Vector3d x_e;
          Eigen::Vector3d v_d;
          Eigen::Vector3d v_e;
          Eigen::Matrix3d kp = Eigen::MatrixXd::Zero(3, 3);
          Eigen::Matrix3d kd = Eigen::MatrixXd::Zero(3, 3);
          kp.diagonal() << leg_cmd_.kp[leg * ModelInterface::N_JOINTS_PER_LEG + 0],
              leg_cmd_.kp[leg * ModelInterface::N_JOINTS_PER_LEG + 1],
              leg_cmd_.kp[leg * ModelInterface::N_JOINTS_PER_LEG + 2];
          kd.diagonal() << leg_cmd_.kd[leg * ModelInterface::N_JOINTS_PER_LEG + 0],
              leg_cmd_.kd[leg * ModelInterface::N_JOINTS_PER_LEG + 1],
              leg_cmd_.kd[leg * ModelInterface::N_JOINTS_PER_LEG + 2];

          quad_model_.calcJacobianLegBase(
              leg, Eigen::Map<const Eigen::Vector3d>(quad_state_.GetJointPositions()[leg].data()), J);  // Should not
          // metter here if it is not
          // body frame as the Jacobian
          // maps to a velocity
          Eigen::Matrix4d unused;
          quad_model_.calcFwdKinLegBody(
              leg, Eigen::Map<const Eigen::Vector3d>(quad_state_.GetJointPositions()[leg].data()), unused, x_e);

          quad_model_.calcFootForceVelocityBodyFrame(leg, quad_state_, f_e, v_e);

          x_d[0] = leg_cmd_.ee_pos[leg * 3 + 0];
          x_d[1] = leg_cmd_.ee_pos[leg * 3 + 1];
          x_d[2] = leg_cmd_.ee_pos[leg * 3 + 2];

          v_d[0] = leg_cmd_.ee_vel[leg * 3 + 0];
          v_d[1] = leg_cmd_.ee_vel[leg * 3 + 1];
          v_d[2] = leg_cmd_.ee_vel[leg * 3 + 2];

          f_d[0] = leg_cmd_.ee_force[leg * 3 + 0];
          f_d[1] = leg_cmd_.ee_force[leg * 3 + 1];
          f_d[2] = leg_cmd_.ee_force[leg * 3 + 2];

          tau = J.transpose() * (kp * (x_d - x_e) + kd * (v_d - v_e) + f_d);
          for (unsigned int j = 0; j < ModelInterface::N_JOINTS_PER_LEG; j++) {
            tau[j] = std::max(-torque_limit_, std::min(torque_limit_, tau[j]));
            joint_cmd_goal_.effort[leg * ModelInterface::N_JOINTS_PER_LEG + j] = tau[j];
          }
        }
      }
      break;
    case LegDriver::OperationMode::kJointTorqueControl:
      if (leg_joint_cmd_.position.size() >= ModelInterface::NUM_JOINTS) {
        joint_cmd_goal_.kp.fill(0.0);
        joint_cmd_goal_.kd.fill(0.0);
        for (unsigned int i = 0; i < ModelInterface::NUM_JOINTS; i++) {
          double current_pos =
              quad_state_.GetJointPositions()[int(i / ModelInterface::N_LEGS)][i % ModelInterface::N_LEGS];
          double current_vel =
              quad_state_.GetJointVelocities()[int(i / ModelInterface::N_LEGS)][i % ModelInterface::N_LEGS];
          double des_pos = leg_joint_cmd_.position[i];
          double des_vel = leg_joint_cmd_.velocity[i];
          double tau = leg_joint_cmd_.effort[i] + leg_joint_cmd_.kp[i] * (des_pos - current_pos)
                       + leg_joint_cmd_.kd[i] * (des_vel - current_vel);
          tau = std::max(-torque_limit_, std::min(torque_limit_, tau));
          joint_cmd_goal_.effort[i] = tau;
        }
      }
      break;
    case LegDriver::OperationMode::kJointControl:
      if (leg_joint_cmd_.position.size() >= ModelInterface::NUM_JOINTS) {
        joint_cmd_goal_.position = leg_joint_cmd_.position;
        joint_cmd_goal_.velocity = leg_joint_cmd_.velocity;
        joint_cmd_goal_.kp = leg_joint_cmd_.kp;
        joint_cmd_goal_.kd = leg_joint_cmd_.kd;
        joint_cmd_goal_.effort = leg_joint_cmd_.effort;
      } else {
        RCLCPP_ERROR(this->get_logger(), "wrong joint size");
      }
      break;
  }
  msg_repeat_++;

  joint_cmd_goal_pub_->publish(joint_cmd_goal_);
}

void LegDriver::do_emergency_damp() {
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "EMERGENCY DAMPING");
  joint_cmd_goal_.position.fill(0.0);
  joint_cmd_goal_.velocity.fill(0.0);
  joint_cmd_goal_.kp.fill(0.0);
  joint_cmd_goal_.kd.fill(damping_kd_);
  joint_cmd_goal_.effort.fill(0.0);
  joint_cmd_goal_pub_->publish(joint_cmd_goal_);
}

void LegDriver::do_damping() {
  if (first_message_received_
      and quad_state_received_) {  // In case a maessage is recived the driver switches automatically back
    RCLCPP_INFO(this->get_logger(), "Leg cmd message and quadt state received, switching to operation");
    switchState(OPERATE);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No leg cmd messages, sending damping");

    joint_cmd_goal_.position.fill(0.0);
    joint_cmd_goal_.velocity.fill(0.0);
    joint_cmd_goal_.kp.fill(0.0);
    joint_cmd_goal_.kd.fill(damping_kd_);
    joint_cmd_goal_.effort.fill(0.0);
    joint_cmd_goal_pub_->publish(joint_cmd_goal_);
  }
}
void LegDriver::do_init_wait() {
  if (first_message_received_ and quad_state_received_) {
    RCLCPP_INFO(this->get_logger(), "First leg cmd message and quadt state received, switching to operation");
    switchState(OPERATE);
  } else if (first_message_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for quad state");
  } else if (quad_state_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for first leg cmd message");
  } else {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Waiting for first leg cmd message and quad state");
  }
}
void LegDriver::do_keep_pose() {
  if (first_message_received_
      and quad_state_received_) {  // In case a maessage is recived the driver switches automatically back
    RCLCPP_INFO(this->get_logger(), "Leg cmd message and quadt state received, switching to operation");
    switchState(OPERATE);
  } else {
    joint_cmd_goal_.position = keep_joint_pos_;
    joint_cmd_goal_.velocity.fill(0.0);
    joint_cmd_goal_.kp.fill(init_Kp_);
    joint_cmd_goal_.kd.fill(init_Kd_);
    joint_cmd_goal_.effort.fill(0.0);
    joint_cmd_goal_pub_->publish(joint_cmd_goal_);
  }
}

void LegDriver::handle_quad_state_update(const std::shared_ptr<interfaces::msg::QuadState> msg) {
  quad_state_ = *msg;
  quad_state_received_ = true;
}

// sub leg state goal
void LegDriver::handle_leg_joint_cmd_update(const std::shared_ptr<interfaces::msg::JointCmd> msg) {
  first_message_received_ = true;
  leg_joint_cmd_ = *msg;
  msg_repeat_ = 0;
}

void LegDriver::handle_leg_cmd_update(const std::shared_ptr<interfaces::msg::LegCmd> msg) {
  first_message_received_ = true;
  leg_cmd_ = *msg;
  msg_repeat_ = 0;
}

void LegDriver::handle_keep_pose(const std::shared_ptr<interfaces::srv::KeepJointPositions::Request> req,
                                 std::shared_ptr<interfaces::srv::KeepJointPositions::Response> resp) {
  if (req->joint_positions.size() == ModelInterface::NUM_JOINTS) {
    keep_joint_pos_ = to_array<ModelInterface::NUM_JOINTS>(req->joint_positions);
    RCLCPP_INFO_STREAM(this->get_logger(), "Keeping requested pose ");
  } else {
    keep_joint_pos_ = default_keep_joint_pos_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Keeping default pose ");
  }
  resp->success = switchState(KEEP_POSE);
}

void LegDriver::handle_change_mode(const std::shared_ptr<interfaces::srv::ChangeLegDriverMode::Request> req,
                                   const std::shared_ptr<interfaces::srv::ChangeLegDriverMode::Response> res) {
  operation_mode_ = static_cast<OperationMode>(req->target_mode);
  RCLCPP_INFO_STREAM(this->get_logger(), "Switching operation mode to " << operation_mode_);
  registerSubscribersAccordingToOperatioMode();
  if (state_ == INIT_WAIT or state_ == DAMPING
      or state_ == KEEP_POSE) {  // Stay here as it is waiting for messages anyway
    res->success = true;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Switch to damping state and wait for new message in new operation mode");
    res->success = switchState(DAMPING);
  }
  res->mode_in = operation_mode_;
}

void LegDriver::registerSubscribersAccordingToOperatioMode() {
  leg_joint_cmd_sub_.reset();
  leg_cmd_sub_.reset();
  switch (operation_mode_) {
    case OperationMode::kJointControl:
    case OperationMode::kJointTorqueControl:
      leg_joint_cmd_sub_ = this->create_subscription<interfaces::msg::JointCmd>(
          "leg_joint_cmd", QOS_RELIABLE_NO_DEPTH, std::bind(&LegDriver::handle_leg_joint_cmd_update, this, _1));
      break;
    case OperationMode::kCartesianStiffnessControl:
    case OperationMode::kCartesianJointControl:
      leg_cmd_sub_ = this->create_subscription<interfaces::msg::LegCmd>(
          "leg_cmd", QOS_RELIABLE_NO_DEPTH, std::bind(&LegDriver::handle_leg_cmd_update, this, _1));
      break;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Registered subscribers according to operation mode " << operation_mode_);
}

void LegDriver::handle_damping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
  (void)req;
  bool ret = switchState(DAMPING);
  resp->success = ret;
  resp->message = "See log";
}

void LegDriver::handle_emerg_damping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
  (void)req;
  bool ret = switchState(EMERGENCY_DAMPING);
  resp->success = ret;
  resp->message = "See log";
}

bool LegDriver::switchState(LegDriver::State new_state) {
  if (new_state == state_) {
    return true;  // nothing to do
  }
  if (new_state == EMERGENCY_DAMPING) {
    RCLCPP_INFO(this->get_logger(), "Swicht to ENERGENCY_DAMPING");
    state_ = EMERGENCY_DAMPING;
    return true;
  }
  switch (state_) {  // Switch over current state.
    case EMERGENCY_DAMPING: {
      if (new_state == DAMPING) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to DAMPING");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Can't switch from state EMERGENCY_DAMPING to %i", new_state);
        return false;
      }
    } break;
    case INIT_WAIT: {
      if (new_state == OPERATE) {
        state_ = new_state;
        RCLCPP_INFO(this->get_logger(), "Switch to OPERATE");
      } else if (new_state == KEEP_POSE) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to KEEP_POSE");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Can't switch from state INIT_WAIT to %i", new_state);
        return false;
      }
    } break;
    case OPERATE: {
      if (new_state == KEEP_POSE) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to KEEP_POSE");
      } else if (new_state == DAMPING) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to DAMPING");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Can't switch from state OPERATE to %i", new_state);
        return false;
      }
    } break;
    case DAMPING: {
      if (new_state == OPERATE) {
        state_ = new_state;
        RCLCPP_INFO(this->get_logger(), "Switch to OPERATE");
      } else if (new_state == KEEP_POSE) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to KEEP_POSE");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Can't switch from state DAMPING to %i", new_state);
        return false;
      }
    } break;
    case KEEP_POSE: {
      if (new_state == OPERATE) {
        state_ = new_state;
        RCLCPP_INFO(this->get_logger(), "Switch to OPERATE");
      } else if (new_state == DAMPING) {
        state_ = new_state;
        first_message_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Switch to DAMPING");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Can't switch from state KEEP_POSE to %i", new_state);
        return false;
      }
    } break;
  }
  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegDriver>());
  rclcpp::shutdown();
  return 0;
}

#include "abstract_motor_driver.hpp"

#include <iterator>
void AbstractMotorDriver::static_sigint_handler(int sig) { instance->sigint_handler(sig); }

void AbstractMotorDriver::sigint_handler(int sig) {
  RCLCPP_INFO(this->get_logger(), "Disabling Motors and Shutting Down...");
  // Disable motors
  this->disable_motors();
  // Shutdown the node
  rclcpp::shutdown();
  exit(sig);
}

void AbstractMotorDriver::watchdog_callback() {
  if (received_initial_msg) {
    // Check if the watchdog counter has reached the limit
    if (this->watchdog_counter > WATCHDOG_LIMIT) {
      // We haven't received a message in a while. Set Torques to zero and
      // disable motors.
      RCLCPP_WARN(this->get_logger(),
                  "Did not receive a message within watchdog limit! Disabling "
                  "motors...");
      // Disable motors
      this->disable_motors();
      motors_enabled = false;
    } else {
      // Increase the counter
      watchdog_counter++;
    }
  }
}

void AbstractMotorDriver::cmd_callback(const cmdMsg::SharedPtr msg) {
  // std::cout<< " Inside topic's callback function definition" <<std::endl;
  // RCLCPP_INFO(this->get_logger(), "Inside abstract callback");
  if (motors_enabled) {
    // We received atleast one message.
    if (!received_initial_msg) received_initial_msg = true;
    // Reset the watchdog counter
    this->watchdog_counter = 0;
    // Save the command message into local variables
    // First, update the local motor commands that uses the joint names
    for (long unsigned int i = 0; i < this->motor_names.size(); i++) {
      std::string name = this->motor_names[i];
      MotorCommand mcommand;
      mcommand.position = msg->position[i];
      mcommand.velocity = msg->velocity[i];
      mcommand.Kp = msg->kp[i];
      mcommand.Kd = msg->kd[i];
      mcommand.torque = msg->effort[i];
      motor_commands[name] = mcommand;
      // RCLCPP_INFO(this->get_logger(), "kp,kd ", msg->kp[i]);
      // std::cout<<"In abstract driver \n Cmd Motor Kp, kd : " << mcommand.Kp << " " <<
      //						mcommand.Kd << std::endl;
    }
    // Second, Update the class variable that use the motor ids
    for (auto const& [name, cmd] : motor_commands) {
      cmds_to_send[motor_name_to_id_map[name]] = cmd;
    }
    frame_id_to_send = msg->header.frame_id;
    // auto now = this->now();
    // auto latency  = now - msg->header.stamp;
    // RCLCPP_INFO(this->get_logger(), "Latency : %f ms",
    // latency.seconds() * 1000
    // + latency.nanoseconds()/1000000.0);
  } else {
    // Reset Watchdog timer
    this->watchdog_counter = 0;
    RCLCPP_WARN(this->get_logger(), "Received a command message but motors are not enabled!");
  }
}

void AbstractMotorDriver::state_pub_callback() {
  std::map<std::tuple<int, int>, MotorState> motor_states = std::get<0>(state_imu_motors);
  for (auto const& [id_map, state] : motor_states) {
    joint_states[motor_id_to_name_map[id_map]] = state;
  }
  stateMsg msg_to_send;
  msg_to_send.header.frame_id = "motor_driver_" + frame_id_to_send;  // "motor_driver"
  msg_to_send.header.stamp = this->now();
  unsigned int i = 0;
  for (auto motor : motor_names) {
    auto states = joint_states[motor];
    msg_to_send.position[i] = states.position;
    msg_to_send.velocity[i] = states.velocity;
    msg_to_send.effort[i] = states.torque;
    i++;
  }

  ImuState imu_states = std::get<1>(state_imu_motors);
  imuMsg imu_msg;
  imu_msg.header.frame_id = "imu_frame";  // Set the frame ID for the IMU message
  imu_msg.header.stamp = this->now();

  // Access individual IMU state variables
  imu_msg.orientation.x = imu_states.orientation_x;
  imu_msg.orientation.y = imu_states.orientation_y;
  imu_msg.orientation.z = imu_states.orientation_z;
  imu_msg.orientation.w = imu_states.orientation_w;
  imu_msg.angular_velocity.x = imu_states.angular_velocity_x;
  imu_msg.angular_velocity.y = imu_states.angular_velocity_y;
  imu_msg.angular_velocity.z = imu_states.angular_velocity_z;
  imu_msg.linear_acceleration.x = imu_states.linear_acceleration_x;
  imu_msg.linear_acceleration.y = imu_states.linear_acceleration_y;
  imu_msg.linear_acceleration.z = imu_states.linear_acceleration_z;

  joint_state_pub->publish(msg_to_send);
  imu_data_pub->publish(imu_msg);
}

void AbstractMotorDriver::cmd_send_callback() {
  // motor_states = send_command(cmds_to_send);
  state_imu_motors = send_command(cmds_to_send);
}

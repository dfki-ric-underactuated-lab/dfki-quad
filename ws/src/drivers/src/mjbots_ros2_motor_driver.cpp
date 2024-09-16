// Mjbots motor driver Class -> derived from Abstract Motor Driver
// Author: Rohit Kumar(r.kumar@dfki.de)

#include "abstract_motor_driver.hpp"
// Include the public header installed in /usr/local/include
#include "mjbots_api/mjbots_hardware_interface.hpp"
#include "mjbots_api/realtime.h"

class MjbotsMotorDriver : public AbstractMotorDriver {
   private:
    ulab::mjbots::RealtimeParams param;
    ulab::mjbots::MoteusJointConfig motor_configs;  // servo bus config
    std::vector<std::shared_ptr<ulab::mjbots::JointMoteus>> joints;
    std::shared_ptr<ulab::mjbots::MjbotsHardwareInterface> mjbots_interface_;
    ulab::IMUData<float> imu_;

    // For mapping from mjbots api to ros2 interfaces
    std::tuple<int, int> bus_motor_map;
    std::map<std::tuple<int, int>, MotorState> motor_states_;
    MotorState mstate;
    ImuState imu_states;
    MotorCommand mcommand;
    double time_ = 0;
    double n_motors = 0;
    int main_cpu;

   public:
    MjbotsMotorDriver() : AbstractMotorDriver("mjbots_ros2_motor_driver") {
      RCLCPP_INFO(this->get_logger(), "Mjbots Motor Driver Node Started");
      // print_motor_properties_map();
      configure_motors();
    }
    ~MjbotsMotorDriver() = default;

    void configure_motors() override {
      RCLCPP_INFO_STREAM(this->get_logger(), "Configuring Motors...");

      // pi3hat moteus_options can cpu
      param.can_cpu = 3;

      // Configure realtime from within the driver node
      param.main_cpu = 2;
      // ::mjbots::moteus::ConfigureRealtime(main_cpu);

      // Start joint config and making vector of joints
      n_motors = motor_properties_map.size();
      for (auto it = motor_properties_map.begin(); it != motor_properties_map.end(); ++it) {
        motor_configs.name = it->first;
        motor_configs.can_bus = motor_properties_map[it->first].bus_id;
        motor_configs.can_id = motor_properties_map[it->first].motor_id;
        motor_configs.moteus_kpScale = motor_properties_map[it->first].Kp;
        motor_configs.moteus_kdScale = motor_properties_map[it->first].Kd;
        motor_configs.max_torque = motor_properties_map[it->first].max_torque;
        motor_configs.direction = motor_properties_map[it->first].direction;
        motor_configs.zero_offset = motor_properties_map[it->first].zero_offset;
        RCLCPP_INFO_STREAM(this->get_logger(), "Zero offset" << motor_properties_map[it->first].zero_offset);
        joints.emplace_back(std::make_shared<ulab::mjbots::JointMoteus>(motor_configs));
      }

      // Create an interface to mjbots pi3hat
      mjbots_interface_ = std::make_shared<ulab::mjbots::MjbotsHardwareInterface>(
          joints,
          param,
          ::mjbots::pi3hat::Euler(imu_config.yaw, imu_config.pitch, imu_config.roll),
          imu_config.attitude_rate_hz,
          nullptr,
          std::nullopt);
      // imu_ = mjbots_interface_->GetIMUDataSharedPtr();
      RCLCPP_INFO(this->get_logger(), "No error in configuration");
      RCLCPP_INFO_STREAM(this->get_logger(), "Finished configuring Motors...");

      // enable_motors();
      // motors_enabled = true;
    }

    std::map<std::tuple<int, int>, MotorState> enable_motors() override {
      RCLCPP_INFO(this->get_logger(), "Enabling Motors...");
      // imu_read_only_ = mjbots_interface_->GetIMUData();
      while (!motors_enabled) {
        motors_enabled = true;
        mjbots_interface_->EnableMotors();
        mjbots_interface_->ProcessReply();
        // mjbots_interface_->GetIMUDataSharedPtr();

        // For printing which mode the joint is currently in. Enable mode is
        // Position mode= 10
        std::vector<::mjbots::moteus::Mode> modes = mjbots_interface_->GetJointModes();
        for (uint i = 0; i < n_motors; i++) {
          std::cout << "Joint id: " << joints[i]->get_can_id() << " Mode : " << static_cast<int>(modes[i]) << std::endl;
          if (static_cast<int>(modes[i]) != 10) {
            motors_enabled = false;
          }
        }
      }

      if (motors_enabled) {
        for (uint i = 0; i < n_motors; i++) {
          // bus_motor_map =
          // std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id());
          mstate.position = joints[i]->get_position();
          mstate.velocity = joints[i]->get_velocity();
          mstate.torque = joints[i]->get_measured_torque();
          motor_states_[std::make_tuple(joints[i]->get_can_bus(), joints[i]->get_can_id())] = mstate;
        }
      } else {
        for (uint i = 0; i < n_motors; i++) {
          // bus_motor_map =
          // std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id());
          mstate.position = 0;
          mstate.velocity = 0;
          mstate.torque = 0;
          motor_states_[std::make_tuple(joints[i]->get_can_bus(), joints[i]->get_can_id())] = mstate;
        }
      }
      // mstate.orientation = imu_->get_quat();
      return motor_states_;
    }
    void disable_motors() override {
      RCLCPP_INFO(this->get_logger(), "Disabling Motors...");

      while (motors_enabled) {
        motors_enabled = false;
        mjbots_interface_->ProcessReply();
        mjbots_interface_->SetModeStop();
        // For printing which mode the joint is currently in. Enable mode is
        // Position mode= 10
        std::vector<::mjbots::moteus::Mode> modes = mjbots_interface_->GetJointModes();
        for (uint i = 0; i < n_motors; i++) {
          std::cout << "Joint id: " << joints[i]->get_can_id() << " Mode : " << static_cast<int>(modes[i]) << std::endl;
          if (static_cast<int>(modes[i]) != 0) {
            motors_enabled = true;
          }
        }
      }
      // const int n = 10;
      // int i =0;
      // while (i < n)
      // {
      // 	mjbots_interface_->ProcessReply();
      // 	mjbots_interface_->SetModeStop();
      // 	// For printing which mode the joint is currently in. Stop is mode 0
      // 	std::vector<::mjbots::moteus::Mode> modes =
      // mjbots_interface_->GetJointModes(); 	std::cout << "mode : " <<
      // static_cast<int>(modes[0]) << std::endl;
      // 	++i;
      // }
    }

    void set_zero() override {
      RCLCPP_INFO(this->get_logger(), "Setting Zero...");
      for (uint i = 0; i < n_motors; i++) {
        joints[i]->set_joint_position_target(0.0);
        // joints[i]->set_joint_velocity_target(0.2);
      }
      mjbots_interface_->SendCommand();
    }
    // std::tuple<std::map<std::tuple<int, int>, MotorState>, IMUState>
    std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState> send_command(
        std::map<std::tuple<int, int>, MotorCommand> commands) override {
      //  (void)commands;
      //  RCLCPP_INFO_STREAM(this->get_logger(), "Sending Commands...");
      for (uint i = 0; i < n_motors; i++) {
        // bus_motor_map =
        // std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id());
        mcommand = commands[std::make_tuple(joints[i]->get_can_bus(), joints[i]->get_can_id())];
        joints[i]->set_joint_position_target(mcommand.position);
        joints[i]->set_joint_velocity_target(mcommand.velocity);
        joints[i]->set_kp(mcommand.Kp);
        joints[i]->set_kd(mcommand.Kd);
        // std::cout<<"Cmd Motor Kp, kd : " << mcommand.Kp << " " <<
        //				mcommand.Kd << std::endl;
        // std::cout<<"Motor Kp, kd : " << joints[i]->get_kp_scale() << " " <<
        //				joints[i]->get_kd_scale() << std::endl;

        joints[i]->UpdateTorque(mcommand.torque);
      }
      mjbots_interface_->SendCommand();
      // std::this_thread::sleep_for(std::chrono::milliseconds(2));
      for (uint i = 0; i < n_motors; i++) {
        // bus_motor_map =
        // std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id());
        mstate.position = joints[i]->get_position();
        mstate.velocity = joints[i]->get_velocity();
        mstate.torque = joints[i]->get_measured_torque();
        motor_states_[std::make_tuple(joints[i]->get_can_bus(), joints[i]->get_can_id())] = mstate;
      }
      //////addd here/////
      imu_ = mjbots_interface_->GetIMUData();

      // static const Eigen::Quaternionf imu_unrot(0, 1, 0, 0);

      Eigen::Vector3<float> accel =  imu_.get_accel();
      const Eigen::Quaternionf quat = imu_.get_quat();  // attitude quaternion
      const Eigen::Matrix3f rot_mat = imu_.get_rot_mat();     // attitude rot.
      // matrix
      const Eigen::Vector3f ang_vel = imu_.get_ang_rate();  // angular velocity

      // Convert Eigen::Vector3<float> to double values
      imu_states.linear_acceleration_x = static_cast<double>(accel.x());
      imu_states.linear_acceleration_y = static_cast<double>(accel.y());
      imu_states.linear_acceleration_z = static_cast<double>(accel.z());

      // Convert Eigen::Vector3f to double values
      imu_states.angular_velocity_x = static_cast<double>(ang_vel.x());
      imu_states.angular_velocity_y = static_cast<double>(ang_vel.y());
      imu_states.angular_velocity_z = static_cast<double>(ang_vel.z());

      // Convert Eigen::Quaternionf to double values
      imu_states.orientation_x = static_cast<double>(quat.x());
      imu_states.orientation_y = static_cast<double>(quat.y());
      imu_states.orientation_z = static_cast<double>(quat.z());
      imu_states.orientation_w = static_cast<double>(quat.w());

      // std::cout << " IMU STATE : " << quat << std::endl;
      // imu_.PrintIMUData();
      return std::make_tuple(motor_states_, imu_states);
    }

    // std::map<std::tuple<int, int>, MotorState> read_joints_state() override
    // {
    // 	mjbots_interface_->ProcessReply();
    // 	for (uint i = 0; i < n_motors; i++)
    // 	{
    // 		// bus_motor_map =
    // std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id());
    // 		mstate.position = joints[i]->get_position();
    // 		mstate.velocity = joints[i]->get_velocity();
    // 		mstate.torque = joints[i]->get_measured_torque();
    // 		motor_states_[std::make_tuple(joints[i]->get_can_bus(),joints[i]->get_can_id())]
    // = mstate;
    // 	}
    // 	return motor_states_;
    // }
  };

  int main(int argc, char *argv[]) {
    /* Init ROS2 if its not already running */
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
    }
    signal(SIGINT, MjbotsMotorDriver::static_sigint_handler);
    // Mjbots should be executed on a single thread
    auto node = std::make_shared<MjbotsMotorDriver>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Add your node to the executor
    executor->add_node(node);

    // Spin the executor
    executor->spin();

    // rclcpp::spin(std::make_shared<MjbotsMotorDriver>());
    return 0;
  }

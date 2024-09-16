// Test Derived Class for the Abstract Motor Driver to Check compilation

#include "abstract_motor_driver.hpp"

class TestAbstractMotorDriver : public AbstractMotorDriver {
 public:
  TestAbstractMotorDriver() : AbstractMotorDriver("test_abstract_motor_driver") {
    RCLCPP_INFO(this->get_logger(), "Test Abstract Motor Driver Node Started");
    std::cout << " IMU Config " << imu_config.roll << " , " << imu_config.attitude_rate_hz << std::endl;
  }
  ~TestAbstractMotorDriver() = default;

  void configure_motors() override { RCLCPP_INFO(this->get_logger(), "Configuring Motors..."); }
  void set_zero() override { RCLCPP_INFO(this->get_logger(), "Setting Zero..."); }
  std::map<std::tuple<int, int>, MotorState> enable_motors() override {
    RCLCPP_INFO(this->get_logger(), "Enabling Motors...");
    return std::map<std::tuple<int, int>, MotorState>();
  }
  void disable_motors() override { RCLCPP_INFO(this->get_logger(), "Disabling Motors..."); }
  std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState> send_command(
      std::map<std::tuple<int, int>, MotorCommand> commands) override {
    (void)commands;
    RCLCPP_INFO_STREAM(this->get_logger(), "Sending Commands...");
    return std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState>();
  }
};

int main(int argc, char *argv[]) {
  /* Init ROS2 if its not already running */
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  signal(SIGINT, TestAbstractMotorDriver::static_sigint_handler);
  rclcpp::spin(std::make_shared<TestAbstractMotorDriver>());
  return 0;
}

/*An Abstract Motor Driver Class for using QDD motors on the mjbots pi3hat
attached to a Raspberry Pi 4. This class provides basic motor driver functions
and can be inherited for creating ROS2 Driver for either T-Motors (using the C++
 drivers from GitHub) or mjbots qdd100 (using the internally developed C++ API
translation from pi3hat and moteus lib). This abstract class provides basic
functionalities such as motor configuration (motor_id:CAN_id dictionary via YAML
file), basic motor functions declaration: enable/disable/set_zero/send_command
(implementations are left to the derived classes), and uses standard message
types to ensure easy compatibility between T-motors and mjbots.

 The derived classes must implement the following pure virtual functions:
 - configure_motors()
 - enable_motors()
 - disable_motors()
 - set_zero()
 - std::map<std::tuple<int, int>, MotorState> motor_states =
send_command(std::map<std::tuple<int, int>, MotorCommand> cmds_to_send)

Important: The derived classes must not do any pub-sub communication! Instead,
they should edit the internal class variables. The pub-sub are handled using
ROS2 timer callbacks which read/write the internal class variables. The
interface to the external nodes is defined using this base class.

  */

#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "common/custom_qos.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/joint_state.hpp"
#include "interfaces/srv/disable_motors.hpp"
#include "interfaces/srv/enable_motors.hpp"
#include "interfaces/srv/zero_motors.hpp"
#include "sensor_msgs/msg/imu.hpp"

using cmdMsg = interfaces::msg::JointCmd;
using enableSrv = interfaces::srv::EnableMotors;
using disableSrv = interfaces::srv::DisableMotors;
using zeroSrv = interfaces::srv::ZeroMotors;
using stateMsg = interfaces::msg::JointState;
using imuMsg = sensor_msgs::msg::Imu;
// Helper function to print vectors with << operator
template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
  out << "{";
  size_t last = v.size() - 1;
  for (size_t i = 0; i < v.size(); ++i) {
    out << v[i];
    if (i != last) out << ", ";
  }
  out << "}";
  return out;
}

class AbstractMotorDriver : public rclcpp::Node {
 public:
  explicit AbstractMotorDriver(const std::string &node_name) : rclcpp::Node(node_name) {
    // Create a singleton instance of the node
    if (instance != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Motor Driver already exists! Shutting down...");
      rclcpp::shutdown();
    } else {
      instance = this;

      // Load configuration from YAML file
      this->declare_parameter("watchdog_limit", 500);
      this->declare_parameter("cmd_send_frequency", 600.0);
      this->declare_parameter("state_pub_frequency", 600.0);
      this->declare_parameter("motor_names", rclcpp::ParameterValue(std::vector<std::string>{}));
      motor_names = this->get_parameter("motor_names").as_string_array();
      std::cout << motor_names << std::endl;
      for (const std::string &motor : motor_names) {
        // Declare the fields required for each motor
        this->declare_parameter(motor + ".bus_id", rclcpp::ParameterValue(int{}));
        this->declare_parameter(motor + ".motor_id", rclcpp::ParameterValue(int{}));
        this->declare_parameter(motor + ".fixed_gains", rclcpp::ParameterValue(bool{}));
        this->declare_parameter(motor + ".Kp", rclcpp::ParameterValue(double{}));
        this->declare_parameter(motor + ".Kd", rclcpp::ParameterValue(double{}));
        this->declare_parameter(motor + ".max_torque", rclcpp::ParameterValue(double{}));
        this->declare_parameter(motor + ".direction", rclcpp::ParameterValue(int{}));
        this->declare_parameter(motor + ".zero_offset", rclcpp::ParameterValue(double{}));
        // Get the values of each motor and add to the motor properties map
        int bus_id = this->get_parameter(motor + ".bus_id").as_int();
        int motor_id = this->get_parameter(motor + ".motor_id").as_int();
        bool fixed_gains = this->get_parameter(motor + ".fixed_gains").as_bool();
        double Kp = this->get_parameter(motor + ".Kp").as_double();
        double Kd = this->get_parameter(motor + ".Kd").as_double();
        double max_torque = this->get_parameter(motor + ".max_torque").as_double();
        int direction = this->get_parameter(motor + ".direction").as_int();
        double zero_offset = this->get_parameter(motor + ".zero_offset").as_double();
        motor_properties_map[motor] = {bus_id, motor_id, fixed_gains, Kp, Kd, max_torque, direction, zero_offset};

        // Make the motor name to motor id map
        motor_name_to_id_map[motor] = {bus_id, motor_id};
        // Make the motor id to motor name map
        motor_id_to_name_map[{bus_id, motor_id}] = motor;
      }

      // Declare the mounting degrees and imu rate in Hz
      this->declare_parameter("imu_mounting_deg.roll", 0.0);
      this->declare_parameter("imu_mounting_deg.pitch", 0.0);
      this->declare_parameter("imu_mounting_deg.yaw", 0.0);
      this->declare_parameter("attitude_rate_hz", 400.0);

      //  Get Values of mounting Degrees of pi3hat connected with Raspberry Pi 4
      double roll = this->get_parameter("imu_mounting_deg.roll").as_double();
      double pitch = this->get_parameter("imu_mounting_deg.pitch").as_double();
      double yaw = this->get_parameter("imu_mounting_deg.yaw").as_double();
      double attitude_rate_hz = this->get_parameter("attitude_rate_hz").as_double();
      imu_config = {roll, pitch, yaw, attitude_rate_hz};

      this->cmd_send_frequency = this->get_parameter("cmd_send_frequency").as_double();
      this->state_pub_frequency = this->get_parameter("state_pub_frequency").as_double();
      // Number of Messages Allowed to miss before disabling the motors.
      // E.g. Running at 100Hz, 50 messages allow for 0.5 seconds of missed
      // messages.
      this->WATCHDOG_LIMIT = this->get_parameter("watchdog_limit").as_int();
      this->watchdog_counter = 0;
      watchdog_timer = rclcpp::create_timer(this,
                                            this->get_clock(),
                                            std::chrono::duration<float>(1.0 / cmd_send_frequency),
                                            std::bind(&AbstractMotorDriver::watchdog_callback, this));

      // Create the Subscriber
      this->command_sub =
          this->create_subscription<cmdMsg>("joint_cmd",
                                            QOS_RELIABLE_NO_DEPTH,
                                            std::bind(&AbstractMotorDriver::cmd_callback, this, std::placeholders::_1));

      // Create the Publishers for States
      this->joint_state_pub = this->create_publisher<stateMsg>("joint_states", QOS_RELIABLE_NO_DEPTH);
      state_pub_timer = rclcpp::create_timer(this,
                                             this->get_clock(),
                                             std::chrono::duration<float>(1.0 / state_pub_frequency),
                                             std::bind(&AbstractMotorDriver::state_pub_callback, this));

      // Create Cmd Send Timer
      cmd_send_timer = rclcpp::create_timer(this,
                                            this->get_clock(),
                                            std::chrono::duration<float>(1.0 / cmd_send_frequency),
                                            std::bind(&AbstractMotorDriver::cmd_send_callback, this));

      // Create the Publisher for IMUdata
      this->imu_data_pub = this->create_publisher<imuMsg>("imu_measurement", QOS_RELIABLE_NO_DEPTH);
    }
  }

  // Public Methods

  // Destructor
  // In derived classes, delete the singleton along with other operations such
  // as disabling motors, closing the CAN communication, etc...
  virtual ~AbstractMotorDriver() {
    // Delete the instance. Hence no singleton exists anymore
    instance = nullptr;
  }

  // Internal Motor State/Command Variables
  // We use a dictionary internally to store the motor states and commands.
  // The key is the motor_id and the value is the MotorState/MotorCommand
  // struct.
  struct MotorState {
    double position;
    double velocity;
    double torque;
  };

  struct ImuState {
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;
    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
  };

  struct MotorCommand {
    double position;
    double velocity;
    double torque;
    double Kp;
    double Kd;
  };

  struct MotorProperties {
    int bus_id;
    int motor_id;
    bool fixed_gains;
    double Kp;
    double Kd;
    double max_torque;
    int direction;
    double zero_offset;
  };

  struct ImuConfig {
    double roll;
    double pitch;
    double yaw;
    double attitude_rate_hz;
  };

  static void static_sigint_handler(int sig);

  /************** Pure Virtual Member Functions. **************/
  // These functions must be implemented in the derived class
  // These should not directly interact with the pub-sub part of the ROS2 node.
  // Instead, they should edit the internal class variables. The pub-sub are
  // handled using ROS2 timer callbacks which read/write the internal class
  // variables.

  // Motor configuration
  virtual void configure_motors() = 0;

  // Motor functions
  virtual std::map<std::tuple<int, int>, MotorState> enable_motors() = 0;
  virtual void disable_motors() = 0;
  virtual void set_zero() = 0;

  virtual std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState> send_command(
      std::map<std::tuple<int, int>, MotorCommand> commands) = 0;
  // virtual std::map<std::tuple<int, int>, MotorState> read_joints_state() = 0;

  void print_motor_properties_map() {
    std::cout << "\n Motor properties size: " << motor_properties_map.size() << std::endl;
    for (auto it = motor_properties_map.begin(); it != motor_properties_map.end(); ++it) {
      std::cout << " Motor name: " << it->first << " Bus id : " << it->second.bus_id
                << " Motor id: " << it->second.motor_id << std::endl;
    }
  }
  // Another map is used to see the motor properties such as fixed gains, Kp,
  // Kd, etc... Maps/Dictionaries to help in conversion between names and motor
  // and bus IDs. The tuple is (bus_id, motor_id)
  std::map<std::string, MotorProperties> motor_properties_map;
  ImuConfig imu_config;
  bool motors_enabled = true;
  // std::map<std::string, imu_param> imu_param;
 private:
  bool received_initial_msg = false;

  int watchdog_counter = 0;
  int64_t WATCHDOG_LIMIT;

  std::vector<std::string> motor_names;

  void watchdog_callback();
  void cmd_send_callback();
  void cmd_callback(cmdMsg::SharedPtr msg);

  void sigint_handler(int sig);
  rclcpp::TimerBase::SharedPtr watchdog_timer, cmd_send_timer;
  rclcpp::TimerBase::SharedPtr state_pub_timer;
  rclcpp::Subscription<cmdMsg>::SharedPtr command_sub;
  rclcpp::Publisher<stateMsg>::SharedPtr joint_state_pub;
  rclcpp::Publisher<imuMsg>::SharedPtr imu_data_pub;

  // C++ STL Does not have bidirectional maps. Hence we use two maps to convert
  // between joint names and motor IDs.
  std::map<std::string, std::tuple<int, int>> motor_name_to_id_map;

  // These structures are for the ROS2 messages which have joint names.
  std::map<std::string, MotorState> joint_states;
  std::map<std::string, MotorCommand> motor_commands;
  // These structures are for the low-level motor drivers which use motor IDs.
  std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState> state_imu_motors;
  std::map<std::tuple<int, int>, MotorCommand> cmds_to_send;

  // OLD .. Only for joint states. Should be kept same later. Modified to be
  // adapted to mjbots above
  // // C++ STL Does not have bidirectional maps. Hence we use two maps to
  // convert between joint names and motor IDs. std::map<std::string,
  // std::tuple<int, int>> motor_name_to_id_map; std::map<std::tuple<int, int>,
  // std::string> motor_id_to_name_map;
  // // These structures are for the ROS2 messages which have joint names.
  // std::map<std::string, MotorState> joint_states;
  // std::map<std::string, MotorCommand> motor_commands;
  // // These structures are for the low-level motor drivers which use motor
  // IDs. std::map<std::tuple<int, int>, MotorState> motor_states;
  // std::map<std::tuple<int, int>, MotorCommand> cmds_to_send;

  std::string frame_id_to_send;

 protected:
  double cmd_send_frequency, state_pub_frequency;
  virtual void state_pub_callback();
  std::map<std::tuple<int, int>, std::string> motor_id_to_name_map;
  /********** Static members ************************/
  inline static AbstractMotorDriver *instance;
 
};

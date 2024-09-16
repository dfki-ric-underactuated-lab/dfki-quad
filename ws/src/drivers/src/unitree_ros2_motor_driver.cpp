#include <chrono>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "abstract_motor_driver.hpp"
#include "common/motor_crc.h"
#include "interfaces/msg/battery_state.hpp"
#include "interfaces/msg/contact_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/motor_state.hpp"

#define INFO_IMU 0         // Set 1 to info IMU states
#define INFO_MOTOR 0       // Set 1 to info motor states
#define INFO_FOOT_FORCE 0  // Set 1 to info foot force states
#define INFO_BATTERY 0     // Set 1 to info battery states

#define HIGH_FREQ 1  // Set 1 to subscribe to low states with high frequencies (500Hz)

using std::placeholders::_1;

class UnitreeMotorDriver : public AbstractMotorDriver {
 private:
  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr suber;

  unitree_go::msg::IMUState imu;          // Unitree go2 IMU message
  unitree_go::msg::MotorState motor[12];  // Unitree go2 motor state message
  int16_t foot_force[4];                  // External contact force value (int)
  int16_t foot_force_est[4];              // Estimated  external contact force value (int)
  float battery_voltage;                  // Battery voltage
  float battery_current;                  // Battery current

  rclcpp::TimerBase::SharedPtr timer_;                              // ROS2 timer
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;  // ROS2 Publisher
  rclcpp::Publisher<interfaces::msg::BatteryState>::SharedPtr battery_state_pub;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr robot_state_request_pub;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr robot_mode_request_pub;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr robot_state_respone_sub;

  std::deque<unitree_api::msg::Response> robot_state_responses_;

  unitree_go::msg::LowCmd cmd_msg;  // Unitree go2 lowcmd message

  // For mapping from unitree api to ros2 interfaces
  std::tuple<int, int> bus_motor_map;
  std::map<std::tuple<int, int>, MotorState> motor_states_;
  MotorState mstate;
  ImuState imu_states;
  MotorCommand mcommand;
  interfaces::msg::ContactState contact_state;
  double time_ = 0;
  double n_motors = 12;
  int main_cpu;
  bool hand_back_control_to_unitree_;
  //  unitree::robot::go2::RobotStateClient rsc;

  rclcpp::Publisher<interfaces::msg::ContactState>::SharedPtr contact_state_pub_;

 public:
  UnitreeMotorDriver() : AbstractMotorDriver("unitree_ros2_motor_driver") {
    RCLCPP_INFO(this->get_logger(), "Unitree Go2 Motor Driver Node Started");
    print_motor_properties_map();
    auto topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }

    this->declare_parameter("hand_back_control_to_unitree", rclcpp::ParameterType::PARAMETER_BOOL);
    hand_back_control_to_unitree_ = this->get_parameter("hand_back_control_to_unitree").as_bool();

    // The suber  callback function is bind to low_state_suber::topic_callback
    suber = this->create_subscription<unitree_go::msg::LowState>(
        topic_name, QOS_BEST_EFFORT_NO_DEPTH, std::bind(&UnitreeMotorDriver::topic_callback, this, _1));

    // configure_motors();
    contact_state_pub_ = this->create_publisher<interfaces::msg::ContactState>("contact_state", QOS_RELIABLE_NO_DEPTH);

    robot_state_request_pub =
        this->create_publisher<unitree_api::msg::Request>("/api/robot_state/request", QOS_RELIABLE_NO_DEPTH);

    robot_mode_request_pub = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    robot_state_respone_sub = this->create_subscription<unitree_api::msg::Response>(
        "/api/robot_state/response", 10, [this](const unitree_api::msg::Response& msg) {
          robot_state_responses_.push_back(msg);
        });

    // the cmd_puber is set to subscribe "/lowcmd" topic
    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", QOS_RELIABLE_NO_DEPTH);
    battery_state_pub = this->create_publisher<interfaces::msg::BatteryState>("/battery_state", QOS_RELIABLE_NO_DEPTH);

    RCLCPP_INFO(this->get_logger(), "Setting up rsc");
    //    rsc.SetTimeout(10.0f);
    //    rsc.Init();

    // Initialize lowcmd
    init_cmd();
  }
  ~UnitreeMotorDriver() = default;
  void init_cmd() {
    //    rsc.ServiceSwitch("sport_mode", 0);

    //    cmd_msg.head[0] = 0xFE;
    //    cmd_msg.head[1] = 0xEF;
    //    cmd_msg.level_flag = 0xFF;
    //    cmd_msg.gpio = 0;

    // try to send it into dmaping mode:
    {
      RCLCPP_INFO(this->get_logger(), "Try to set sports mode into damping");
      unitree_api::msg::Request req;
      req.header.identity.api_id = 1001;
      std::packaged_task<bool()> delay([this, req]() {
        robot_mode_request_pub->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        robot_mode_request_pub->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        robot_mode_request_pub->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return true;
      });
      auto fut = delay.get_future();
      std::thread t(std::move(delay));
      auto fut_res = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
      t.join();
    }

    // try to disable the sport mode:
    RCLCPP_INFO(this->get_logger(), "Disable sports mode");
    unitree_api::msg::Request req;  // The content of this message is reverse engineered and nowhere documented
    req.header.identity.id = 0;
    req.header.lease.id = 0;
    req.header.identity.api_id = 1001;
    req.header.policy.priority = 0;
    req.header.policy.noreply = false;
    req.parameter = "{\"name\":\"sport_mode\",\"switch\":0}";
    req.binary = {0};
    // await response
    std::packaged_task<bool()> wait_response([this, req]() {
      while (true) {
        RCLCPP_INFO(this->get_logger(), "Sending request");
        robot_state_request_pub->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (robot_state_responses_.size() > 0) {
          RCLCPP_INFO(this->get_logger(), "Received answer");
          auto resp = robot_state_responses_.front();
          robot_state_responses_.pop_front();
          if ((resp.header.identity.api_id == 1001) and (resp.header.status.code == 0)
              and (resp.data.compare("{\"name\":\"sport_mode\",\"status\":1}") == 0)) {
            RCLCPP_INFO(this->get_logger(), "Answer ok");
            return true;
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Answer is not ok (%li, %i, %s)",
                        resp.header.identity.api_id,
                        resp.header.status.code,
                        resp.data.c_str());
            return false;
          }
        }
      }
    });
    auto fut = wait_response.get_future();
    std::thread t(std::move(wait_response));
    auto fut_res = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    t.join();
    if (fut.get()) {
      RCLCPP_INFO(this->get_logger(), "Sport mode has been disabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error while disabling sport mode, ending");
      exit(-1);
    }

    for (int i = 0; i < 20; i++) {
      cmd_msg.motor_cmd[i].mode = 0x01;  // Set toque mode, 0x00 is passive mode
      cmd_msg.motor_cmd[i].q = PosStopF;
      cmd_msg.motor_cmd[i].kp = 0;
      cmd_msg.motor_cmd[i].dq = VelStopF;
      cmd_msg.motor_cmd[i].kd = 0;
      cmd_msg.motor_cmd[i].tau = 0;
    }
    cmd_puber->publish(cmd_msg);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / this->cmd_send_frequency),
                                     std::bind(&UnitreeMotorDriver::timer_callback, this));
    motors_enabled = true;
  }
  void timer_callback() {
    get_crc(cmd_msg);             // Check motor cmd crc
    cmd_puber->publish(cmd_msg);  // Publish lowcmd message
  }
  void topic_callback(unitree_go::msg::LowState::SharedPtr data) {
    if (INFO_IMU) {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)

      RCLCPP_INFO(
          this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu.quaternion[0],
                  imu.quaternion[1],
                  imu.quaternion[2],
                  imu.quaternion[3]);
      RCLCPP_INFO(this->get_logger(),
                  "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu.gyroscope[0],
                  imu.gyroscope[1],
                  imu.gyroscope[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu.accelerometer[0],
                  imu.accelerometer[1],
                  imu.accelerometer[2]);
    }

    imu = data->imu_state;

    if (INFO_MOTOR) {
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque

      for (int i = 0; i < 12; i++) {
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i,
                    motor[i].q,
                    motor[i].dq,
                    motor[i].ddq,
                    motor[i].tau_est);
      }
    }

    for (int i = 0; i < 12; i++) {
      motor[i] = data->motor_state[i];
    }

    if (INFO_FOOT_FORCE) {
      // Info foot force value (int not true value)
      for (int i = 0; i < 4; i++) {
        foot_force[i] = data->foot_force[i];
        foot_force_est[i] = data->foot_force_est[i];
      }

      RCLCPP_INFO(this->get_logger(),
                  "Foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force[0],
                  foot_force[1],
                  foot_force[2],
                  foot_force[3]);
      RCLCPP_INFO(this->get_logger(),
                  "Estimated foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force_est[0],
                  foot_force_est[1],
                  foot_force_est[2],
                  foot_force_est[3]);
    }

    // Mapping from https://support.unitree.com/home/en/developer
    contact_state.ground_contact_force[0] = data->foot_force[1];
    contact_state.ground_contact_force[1] = data->foot_force[0];
    contact_state.ground_contact_force[2] = data->foot_force[3];
    contact_state.ground_contact_force[3] = data->foot_force[2];

    if (INFO_BATTERY) {
      // Info battery states
      // battery current
      // battery voltage

      RCLCPP_INFO(this->get_logger(), "Battery state -- current: %f; voltage: %f", battery_current, battery_voltage);
    }

    battery_current = data->power_a;
    battery_voltage = data->power_v;

    interfaces::msg::BatteryState battery_state_msg;
    battery_state_msg.current = battery_current;
    battery_state_msg.voltage = battery_voltage;
    battery_state_pub->publish(battery_state_msg);
  }

  void configure_motors() override {
    RCLCPP_INFO_STREAM(this->get_logger(), "Configuring Motors...");
    RCLCPP_INFO(this->get_logger(), "No error in configuration");
  }

  std::map<std::tuple<int, int>, MotorState> enable_motors() override {
    RCLCPP_INFO(this->get_logger(), "Enabling Motors...");
  }
  void disable_motors() override {
    RCLCPP_INFO(this->get_logger(), "Disabling Motors...");
    timer_.reset();

    for (int i = 0; i < 20; i++) {
      cmd_msg.motor_cmd[i].mode = 0x00;  // Set toque mode, 0x00 is passive mode
      cmd_msg.motor_cmd[i].q = PosStopF;
      cmd_msg.motor_cmd[i].kp = 0;
      cmd_msg.motor_cmd[i].dq = VelStopF;
      cmd_msg.motor_cmd[i].kd = 0;
      cmd_msg.motor_cmd[i].tau = 0;
    }

    if (hand_back_control_to_unitree_) {
      RCLCPP_INFO(this->get_logger(), "Enable sports mode");
      unitree_api::msg::Request req;  // The content of this message is reverse engineered and nowhere documented
      req.header.identity.id = 0;
      req.header.lease.id = 0;
      req.header.identity.api_id = 1001;
      req.header.policy.priority = 0;
      req.header.policy.noreply = false;
      req.parameter = "{\"name\":\"sport_mode\",\"switch\":1}";
      req.binary = {0};
      robot_state_request_pub->publish(req);
      robot_state_request_pub->publish(req);
      robot_state_request_pub->publish(req);
      motors_enabled = false;
    }
  }

  void set_zero() override { RCLCPP_INFO(this->get_logger(), "Setting Zero..."); }

  std::tuple<std::map<std::tuple<int, int>, MotorState>, ImuState> send_command(
      std::map<std::tuple<int, int>, MotorCommand> commands) override {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Reading Commands...");
    for (uint i = 0; i < n_motors; i++) {
      auto motor_id = std::make_tuple(1, i);
      mcommand = commands[motor_id];
      cmd_msg.motor_cmd[i].q =
          mcommand.position * motor_properties_map[motor_id_to_name_map[motor_id]].direction;  // Set to// stop
      // position(rad)
      cmd_msg.motor_cmd[i].kp = mcommand.Kp;
      cmd_msg.motor_cmd[i].dq = mcommand.velocity;  // Set to stop angular velocity(rad/s)
      cmd_msg.motor_cmd[i].kd = mcommand.Kd;
      cmd_msg.motor_cmd[i].tau = std::clamp(mcommand.torque,
                                            -motor_properties_map[motor_id_to_name_map[motor_id]].max_torque,
                                            motor_properties_map[motor_id_to_name_map[motor_id]].max_torque);
    }
    // joint index
    // constexpr int FR_0 = 0;
    // constexpr int FR_1 = 1;
    // constexpr int FR_2 = 2;

    // constexpr int FL_0 = 3;
    // constexpr int FL_1 = 4;
    // constexpr int FL_2 = 5;

    // constexpr int RR_0 = 6;
    // constexpr int RR_1 = 7;
    // constexpr int RR_2 = 8;

    // constexpr int RL_0 = 9;
    // constexpr int RL_1 = 10;
    // constexpr int RL_2 = 11;
    // here i acts as motorid for the motors.
    for (uint i = 0; i < n_motors; i++) {
      mstate.position = motor[i].q;
      mstate.velocity = motor[i].dq;
      mstate.torque = motor[i].tau_est;
      motor_states_[std::make_tuple(1, i)] = mstate;
      // RCLCPP_INFO(this->get_logger(),
      //               "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
      //               i,
      //               mstate.position,
      //               mstate.velocity,
      //               motor[i].ddq,
      //               mstate.torque);
    }

    // Convert Eigen::Vector3<float> to double values
    imu_states.linear_acceleration_x = static_cast<double>(imu.accelerometer[0]);
    imu_states.linear_acceleration_y = static_cast<double>(imu.accelerometer[1]);
    imu_states.linear_acceleration_z = static_cast<double>(imu.accelerometer[2]);

    // Convert Eigen::Vector3f to double values
    imu_states.angular_velocity_x = static_cast<double>(imu.gyroscope[0]);
    imu_states.angular_velocity_y = static_cast<double>(imu.gyroscope[1]);
    imu_states.angular_velocity_z = static_cast<double>(imu.gyroscope[2]);

    // Convert Eigen::Quaternionf to double values
    imu_states.orientation_x = static_cast<double>(imu.quaternion[1]);
    imu_states.orientation_y = static_cast<double>(imu.quaternion[2]);
    imu_states.orientation_z = static_cast<double>(imu.quaternion[3]);
    imu_states.orientation_w = static_cast<double>(imu.quaternion[0]);
    // std::cout << " IMU STATE : " << quat << std::endl;
    // imu_.PrintIMUData();
    return std::make_tuple(motor_states_, imu_states);
  }

  void state_pub_callback() override {
    AbstractMotorDriver::state_pub_callback();
    contact_state_pub_->publish(contact_state);
  }
};

int main(int argc, char* argv[]) {
  /* Init ROS2 if its not already running */
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  //  unitree::robot::ChannelFactory::Instance()->Init(0, "enp0s31f6");
  signal(SIGINT, UnitreeMotorDriver::static_sigint_handler);
  rclcpp::spin(std::make_shared<UnitreeMotorDriver>());  // Run ROS2 node which is make share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}

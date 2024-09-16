#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "common/custom_qos.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "common/quad_model_symbolic.hpp"
#include "common/quad_model_symbolic.hpp"
#include "common/quad_state.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/srv/change_leg_driver_mode.hpp"
#include "interfaces/srv/keep_joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/*
Leg driver node provides an abstraction for higher level controllers that
reason about contact forces and locatiosn rather than just joint states.
*/

class LegDriver : public rclcpp::Node {
 public:
  enum OperationMode {
    kJointControl = 0,           // PD control in Motor driver
    kJointTorqueControl,         // PD control in leg driver
    kCartesianStiffnessControl,  // Kp, Kd are used as Cartesian gains
    kCartesianJointControl,      // Kp, Kd are used as Motor gains
  };
  enum State {
    INIT_WAIT,         // Initial state in which the driver does nothing until a message or service arrives
    OPERATE,           // Operation mode in which it performs the control as specified in OperationMode
    KEEP_POSE,         // Keeps a predefined pose (e.g. stand up)
    DAMPING,           // Damping mode in which is switches e.g. automatically when no message arrives
    EMERGENCY_DAMPING  // Damping mode from which you can only return via service
  };

  LegDriver();
  void handle_quad_state_update(const std::shared_ptr<interfaces::msg::QuadState> msg);
  void handle_leg_joint_cmd_update(const std::shared_ptr<interfaces::msg::JointCmd> msg);
  void handle_leg_cmd_update(const std::shared_ptr<interfaces::msg::LegCmd> msg);
  void handle_keep_pose(const std::shared_ptr<interfaces::srv::KeepJointPositions::Request> req,
                        std::shared_ptr<interfaces::srv::KeepJointPositions::Response> resp);
  void handle_damping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_emerg_damping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_change_mode(const std::shared_ptr<interfaces::srv::ChangeLegDriverMode::Request> req,
                          const std::shared_ptr<interfaces::srv::ChangeLegDriverMode::Response> res);

  // main update functions, depending on state
  void do_control();
  void do_damping();
  void do_init_wait();
  void do_keep_pose();
  void do_emergency_damp();

 private:
  bool switchState(State new_state);
  void registerSubscribersAccordingToOperatioMode();

  // -- publishers --
  rclcpp::Publisher<interfaces::msg::JointCmd>::SharedPtr joint_cmd_goal_pub_;

  // -- subscribers --
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_sub_;
  rclcpp::Subscription<interfaces::msg::JointCmd>::SharedPtr leg_joint_cmd_sub_;
  rclcpp::Subscription<interfaces::msg::LegCmd>::SharedPtr leg_cmd_sub_;
  rclcpp::Service<interfaces::srv::KeepJointPositions>::SharedPtr set_keep_joint_pos_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_daming_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_energ_daming_mode_srv_;
  rclcpp::Service<interfaces::srv::ChangeLegDriverMode>::SharedPtr switch_op_mode_srv_;

  float update_freq_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  // rclcpp::TimerBase::SharedPtr joint_cmd_goal_pub_timer_;
  // rclcpp::TimerBase::SharedPtr quad_state_sub_timer_;
  // rclcpp::TimerBase::SharedPtr leg_joint_cmd_sub_timer_;

  QuadState quad_state_;
  interfaces::msg::JointCmd joint_cmd_goal_;
  interfaces::msg::JointCmd leg_joint_cmd_;
  interfaces::msg::LegCmd leg_cmd_;

  // quad model
  QuadModelSymbolic quad_model_;
  bool quad_state_received_;
  bool first_message_received_;

  OperationMode operation_mode_;
  State state_;

  // params
  float damping_kd_;
  double torque_limit_;
  std::array<double, ModelInterface::NUM_JOINTS> default_keep_joint_pos_;
  std::array<double, ModelInterface::NUM_JOINTS> keep_joint_pos_;
  double init_Kp_;
  double init_Kd_;

  // counter to go back to damping mode without new messages
  unsigned int msg_repeat_;
  unsigned int max_msg_repeat_;
};

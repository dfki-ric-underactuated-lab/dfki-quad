#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "common/custom_qos.hpp"
#include "common/eigen_msg_conversions.hpp"
#include "common/quad_model_symbolic.hpp"
#include "common/quad_state.hpp"
#include "common/sequence_containers.hpp"
#include "interfaces/msg/controller_info.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/position_sequence.hpp"
#include "interfaces/msg/quad_control_target.hpp"
#include "interfaces/msg/quad_model.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/msg/vector_sequence.hpp"
#include "interfaces/msg/wbc_return.hpp"
#include "interfaces/msg/wbc_target.hpp"
#include "interfaces/msg/mpc_diagnostics.hpp"
#include "interfaces/srv/change_leg_driver_mode.hpp"
#include "mit_controller/adaptive_gait_sequencer.hpp"
#include "mit_controller/gait_sequence_to_msg.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/inverse_dynamics.hpp"
#include "mit_controller/mit_controller_params.hpp"
#include "mit_controller/mpc.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/simple_gait_sequencer.hpp"
#include "mit_controller/swing_leg_controller.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wbc_arc_opt.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/least_squares_model_adaptation.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"

class MITController : public rclcpp::Node {
 public:
  enum LEGControlMode {
    CARTESIAN_JOINT_CONTROL = 3,
    CARTESIAN_STIFFNESS_CONTROL = 2,
    JOINT_TORQUE_CONTROL = 1,
    JOINT_CONTROL = 0
  };

 private:
  enum LegStatus { SWING, STANCE, EARLY_CONTACT, LATE_CONTACT, LOST_CONTACT };

  // Parameters:
  LEGControlMode leg_control_mode_;
  Eigen::Vector3d cartesian_joint_control_swing_Kp_;
  Eigen::Vector3d cartesian_joint_control_swing_Kd_;
  Eigen::Vector3d cartesian_joint_control_stance_Kp_;
  Eigen::Vector3d cartesian_joint_control_stance_Kd_;
  Eigen::Vector3d cartesian_stiffness_control_swing_Kp_;
  Eigen::Vector3d cartesian_stiffness_control_swing_Kd_;
  Eigen::Vector3d cartesian_stiffness_control_stance_Kp_;
  Eigen::Vector3d cartesian_stiffness_control_stance_Kd_;
  Eigen::Vector3d joint_control_swing_Kp_;
  Eigen::Vector3d joint_control_swing_Kd_;
  Eigen::Vector3d joint_control_stance_Kp_;
  Eigen::Vector3d joint_control_stance_Kd_;
  bool early_contact_detection_;
  bool late_contact_detection_;
  bool lost_contact_detection_;
  bool late_contact_reschedule_swing_phase_;
  bool use_model_adaptation_;
  Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1> state_weights_stand_;
  Eigen::Matrix<double, MPC::STATE_SIZE - 1, 1> state_weights_move_;

  // ROS related members
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_subscription_;
  rclcpp::Subscription<interfaces::msg::QuadControlTarget>::SharedPtr quad_control_target_subscription_;
  rclcpp::Client<interfaces::srv::ChangeLegDriverMode>::SharedPtr change_leg_driver_mode_client_;
  rclcpp::Publisher<interfaces::msg::LegCmd>::SharedPtr leg_cmd_publisher_;
  rclcpp::Publisher<interfaces::msg::JointCmd>::SharedPtr leg_joint_cmd_publisher_;

  interfaces::msg::LegCmd leg_cmd_;
  interfaces::msg::JointCmd leg_joint_cmd_;
  interfaces::msg::ControllerInfo controller_heartbeat_;

  rclcpp::TimerBase::SharedPtr mpc_loop_timer_;
  rclcpp::TimerBase::SharedPtr slc_loop_timer_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;  // SLC runs with a higher frequency
  rclcpp::TimerBase::SharedPtr
      model_adaptation_loop_timer_;  // Potentially, the model adaptation runs with a lower frequency
  rclcpp::TimerBase::SharedPtr heartbeat_loop_timer_;

  rclcpp::Publisher<interfaces::msg::VectorSequence>::SharedPtr swing_leg_trajs_publisher_;
  rclcpp::Publisher<interfaces::msg::GaitState>::SharedPtr gait_state_publisher_;
  rclcpp::Publisher<interfaces::msg::PositionSequence>::SharedPtr open_loop_publisher_;
  rclcpp::Publisher<interfaces::msg::MPCDiagnostics>::SharedPtr solve_time_publisher_;
  rclcpp::Publisher<interfaces::msg::WBCReturn>::SharedPtr wbc_solve_time_publisher_;
  rclcpp::Publisher<interfaces::msg::WBCTarget>::SharedPtr wbc_target_publisher_;
  rclcpp::Publisher<interfaces::msg::GaitSequence>::SharedPtr gait_sequence_publisher_;
  rclcpp::Publisher<interfaces::msg::QuadModel>::SharedPtr quad_model_publisher_;
  rclcpp::Publisher<interfaces::msg::ControllerInfo>::SharedPtr controller_heartbeat_publisher_;

  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> on_setparam_callback_handler_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> parameter_event_callback_handle_;

  // Controller related members
  // std::shared_ptr<MPCInterface> mpc_;
  std::unique_ptr<MPCInterface> mpc_;
  std::unique_ptr<GaitSequencerInterface> gs_;
  std::unique_ptr<SwingLegControllerInterface> slc_;
  typedef std::conditional<USE_WBC,
                           WBCInterface<JointTorqueVelocityPositionCommands>,
                           WBCInterface<CartesianCommands>>::type WBCType;
  std::unique_ptr<WBCType> wbc_;
  // std::shared_ptr<ModelAdaptationInterface> ma_;
  std::unique_ptr<ModelAdaptationInterface> ma_;
  Target target_;
  GaitSequence gait_sequence_;
  bool gs_updated_;
  WrenchSequence wrench_sequence_;
  MPCPrediction mpc_prediction_;
  FeetTargets feet_targets_;
  std::array<double, ModelInterface::N_LEGS> feet_swing_progress_;
  std::array<SwingLegControllerInterface::LegState, ModelInterface::N_LEGS> feet_swing_states_;
  GaitSequence::Mode last_gait_sequence_mode_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> last_feet_pos_targets_;
  std::array<LegStatus, ModelInterface::N_LEGS> feet_status_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> early_contact_hold_position_;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> slip_hold_in_body_;

  // For sync
  std::mutex quad_state_lock_;
  std::mutex gs_wrench_sequence_lock_;
  std::mutex targets_lock_;
  std::mutex gait_sequencer_lock_;
  std::mutex mpc_lock_;
  std::mutex slc_lock_;  // TODO: instead of this locks, maybe schedule the change to the repsective callback group
  std::mutex wbc_lock_;

  // For multithreading
  rclcpp::CallbackGroup::SharedPtr mpc_call_back_group_;
  rclcpp::CallbackGroup::SharedPtr slc_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_loop_call_back_group_;
  rclcpp::CallbackGroup::SharedPtr model_adaptation_callback_group_;

  // State and model
  bool first_quad_state_received_;
  QuadState quad_state_;
  QuadModelSymbolic quad_model_;

  std::unique_ptr<GaitSequencerInterface> GetGaitSequencerFromParams() const;

 public:
  MITController(const std::string& nodeName);
  void QuadStateUpdateCallback(interfaces::msg::QuadState::SharedPtr quad_state_msg);
  void QuadControlTargetUpdateCallback(interfaces::msg::QuadControlTarget::SharedPtr quad_target_msg);

  void MPCLoopCallback();
  void SLCLoopCallback();
  void ControlLoopCallback();
  void ModelAdaptationCallback();
  void HartbeatCallback();
};

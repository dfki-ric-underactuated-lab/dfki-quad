#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "common/custom_qos.hpp"
#include "common/eigen_msg_conversions.hpp"
#include "common/quad_model_symbolic.hpp"
#include "contact_detection.hpp"
#include "interfaces/msg/contact_state.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/joint_state.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "kalman_filter.hpp"
#include "sensor_msgs/msg/imu.hpp"
#ifdef WITH_VICON
#include "vicon_receiver/msg/position.hpp"
#endif

class StateEstimationNode : public rclcpp::Node {
 private:
  const unsigned int FILTER_WINDOW = 5;
  const unsigned int IMU_FILTER_WINDOW = 5;
  const unsigned int VICON_FILTER_WINDOW = 5;

  // Members
  std::shared_ptr<ModelInterface> &quad_model_;
  interfaces::msg::JointState joint_state_;
  interfaces::msg::JointCmd joint_cmd_;
  interfaces::msg::GaitState gait_state_;
  StateInterface::TimePoint last_imu_time_;
  StateInterface::TimePoint last_joint_time_;
  Eigen::Vector3d last_angular_velocity_;
  std::array<double, ModelInterface::NUM_JOINTS> last_joint_velocity_;
  MovingAverage<std::array<double, ModelInterface::NUM_JOINTS>> joint_vel_filter_;
  MovingAverage<std::array<double, ModelInterface::NUM_JOINTS>> joint_acc_filter_;
  MovingAverage<std::array<double, ModelInterface::NUM_JOINTS>> joint_tau_filter_;
  MovingAverage<Eigen::Vector3d> angular_vel_filter_;
  MovingAverage<Eigen::Vector3d> linear_acc_filter_;
  MovingAverage<Eigen::Vector3d> vicon_pos_filter_;
  MovingAverage<Eigen::Quaterniond> vicon_orient_filter_;
  QuadState::TimePoint last_vicon_time_;
  Eigen::Vector3d last_vicon_angular_vel_;
  Eigen::Vector3d last_vicon_linear_vel_;

  // ROS 2 Stuff
  rclcpp::SubscriptionBase::SharedPtr imu_subscription_;
  rclcpp::SubscriptionBase::SharedPtr joint_states_subscription_;
  rclcpp::SubscriptionBase::SharedPtr joint_cmd_subscription_;
  rclcpp::SubscriptionBase::SharedPtr vicon_receiver_;
  rclcpp::Publisher<interfaces::msg::QuadState>::SharedPtr quad_state_publisher_;
  rclcpp::Publisher<interfaces::msg::QuadState>::SharedPtr quad_state_lf_publisher_;
  rclcpp::SubscriptionBase::SharedPtr contact_state_subscription_;
  rclcpp::SubscriptionBase::SharedPtr gait_state_subscription_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_state_estimation_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_state_estimation_convariances_service_;

  rclcpp::TimerBase::SharedPtr kalman_update_step_timer_;
  rclcpp::TimerBase::SharedPtr quad_state_publish_timer_;
  rclcpp::TimerBase::SharedPtr quad_state_lf_publish_timer_;
  rclcpp::TimerBase::SharedPtr contact_threshold_timer_;

  // Parameters
  bool first_orientation_from_raw_imu_set_;
  std::array<double, ModelInterface::N_LEGS> contact_detection_force_threshold_;
  std::array<double, ModelInterface::N_LEGS> laying_on_ground_detection_offset_;

  std::array<MovingMin<double>, ModelInterface::N_LEGS> contact_detection_force_filter_;
  std::array<MovingMax<double>, ModelInterface::N_LEGS> max_contact_detection_force_filter_;
  std::array<double, ModelInterface::N_LEGS> ground_contact_force_;
  double threshold_offset_;
  double max_threshold_offset_;
  double min_stance_percentage_for_contact_;
  bool use_only_gait_contacts_;
  bool replace_kalman_filter_by_vicon_;

  // Sub parts
  KalmanFilter::Params kalman_params_;
  KalmanFilter *kalman_filter_;  // is a pointer in order to exchange and lazy initialise it
  ContactDetection *contact_detection_;

  // Output Msg
  interfaces::msg::QuadState quad_state_msg_;

 public:
  explicit StateEstimationNode(std::shared_ptr<ModelInterface> &quadModel);

  // Callbacks
  void JointStatesCallback(const interfaces::msg::JointState &joint_state_msg);
  void ImuCallback(const sensor_msgs::msg::Imu &imu_msg);
#ifdef WITH_VICON
  void ViconCallback(const vicon_receiver::msg::Position &vicon_msg);
#endif
  void KalmanUpdateCallback();
  void PublishStateCallback();
  void PublishStateLfCallback();
  void ResetStateEstimationCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response);
  void ResetStateEstimationCovariancesCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                               std_srvs::srv::Trigger::Response::SharedPtr response);
  void ContactThresholdCallback();
  void GaitStateCallback(const interfaces::msg::GaitState &gait_state_msg);
};

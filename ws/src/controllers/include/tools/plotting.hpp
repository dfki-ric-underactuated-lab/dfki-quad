#pragma once

#include <matplot/matplot.h>

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common/custom_qos.hpp"
#include "common/filters.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "interfaces/msg/quad_control_target.hpp"
#include "interfaces/msg/quad_state.hpp"

#define BUFFER_SIZE 200

class GaitPlottingNode : public rclcpp::Node {
 public:
  GaitPlottingNode(const std::string& nodeName);

 private:
  void GaitStateCallback(interfaces::msg::GaitState::SharedPtr msg);
  void QuadStateCallback(interfaces::msg::QuadState::SharedPtr msg);
  void TargetCallback(interfaces::msg::QuadControlTarget::SharedPtr msg);
  void PlottingCallback();

  rclcpp::Subscription<interfaces::msg::GaitState>::SharedPtr gait_state_subscription_;
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_subscription_;
  rclcpp::Subscription<interfaces::msg::QuadControlTarget>::SharedPtr target_subscription_;
  interfaces::msg::GaitState gait_state_;
  interfaces::msg::QuadState quad_state_;
  interfaces::msg::QuadControlTarget target_state_;
  double plotting_freq_;
  rclcpp::TimerBase::SharedPtr plotting_timer_;
  matplot::figure_handle figure_;
  std::vector<matplot::axes_handle> ax_;
  std::array<std::array<double, BUFFER_SIZE>, 4> contact_buffer_;
  unsigned int buffer_index_;
  MovingAverage<double> cot_filter_;
  std::array<double, BUFFER_SIZE> cot_buffer_;
  std::array<double, BUFFER_SIZE> v_buffer_;
  std::array<double, BUFFER_SIZE> v_des_buffer_;
  std::array<std::array<double, BUFFER_SIZE>, 4> duty_buffer_;
  std::array<std::array<double, BUFFER_SIZE>, 4> phase_buffer_;
};
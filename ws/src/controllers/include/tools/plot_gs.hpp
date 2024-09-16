#pragma once

#include <matplot/matplot.h>

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common/custom_qos.hpp"
#include "common/filters.hpp"
#include "interfaces/msg/gait_sequence.hpp"
#include "mit_controller/mit_controller_params.hpp"

class GaitSequencePlottingNode : public rclcpp::Node {
 public:
  GaitSequencePlottingNode(const std::string& nodeName);

 private:
  void GaitSequenceCallback(interfaces::msg::GaitSequence::SharedPtr msg);
  void PlottingCallback();

  rclcpp::Subscription<interfaces::msg::GaitSequence>::SharedPtr gait_sequence_subscription_;
  interfaces::msg::GaitSequence gait_sequence_;
  double plotting_freq_;
  rclcpp::TimerBase::SharedPtr plotting_timer_;
  matplot::figure_handle figure_;
  std::vector<matplot::axes_handle> ax_;
  unsigned int buffer_index_;
  bool updated_;
};
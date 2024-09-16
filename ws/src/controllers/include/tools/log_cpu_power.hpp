# pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>


class CPUPowerLoggingNode : public rclcpp::Node {
 public:
  CPUPowerLoggingNode(const std::string& nodeName, uint64_t prev_energy_, std::chrono::steady_clock::time_point prev_time_);

 private:
  void TimerCallback();
  double GetCPUPower();

  rclcpp::TimerBase::SharedPtr logging_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cpu_power_publisher_;

  uint64_t prev_energy_;
  std::chrono::steady_clock::time_point prev_time_;
};
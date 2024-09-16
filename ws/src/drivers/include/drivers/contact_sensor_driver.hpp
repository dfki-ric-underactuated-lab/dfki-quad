#pragma once

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>

#include "common/custom_qos.hpp"
#include "interfaces/msg/contact_state.hpp"

class ContactSensorDriver : public rclcpp::Node {
 public:
  ContactSensorDriver();
  virtual ~ContactSensorDriver();

  void Run();

 private:
  rclcpp::Publisher<interfaces::msg::ContactState>::SharedPtr contact_state_publisher_;
  interfaces::msg::ContactState contact_state_msg_;
  LibSerial::SerialPort serial_port_;
  int serial_read_time_out_ms_;
};
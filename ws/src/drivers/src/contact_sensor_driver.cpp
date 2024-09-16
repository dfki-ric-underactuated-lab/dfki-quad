#include "contact_sensor_driver.hpp"
ContactSensorDriver::ContactSensorDriver() : rclcpp::Node("contact_sensor_driver") {
  this->declare_parameter("serial_port_name", rclcpp::ParameterType::PARAMETER_STRING);
  this->declare_parameter("serial_read_time_out_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
  serial_read_time_out_ms_ = this->get_parameter("serial_read_time_out_ms").as_int();
  contact_state_publisher_ =
      this->create_publisher<interfaces::msg::ContactState>("/contact_state", QOS_RELIABLE_NO_DEPTH);
  try {
    serial_port_.Open(this->get_parameter("serial_port_name").as_string(), std::ios_base::in);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  } catch (const LibSerial::OpenFailed&) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    rclcpp::shutdown();
    exit(-1);
  }
}

void ContactSensorDriver::Run() {
  std::string token;
  std::string line;
  unsigned int cnt = 0;
  while (rclcpp::ok()) {
    try {
      // Read a line from the serial port
      serial_port_.ReadLine(line, '\n', serial_read_time_out_ms_);
      // Parse the received line
      std::istringstream ss(line);
      cnt = 0;
      while (std::getline(ss, token, ';')) {
        try {
          contact_state_msg_.ground_contact_force[cnt] = std::stoi(token);
        } catch (const std::invalid_argument& e) {
          RCLCPP_ERROR(this->get_logger(), "Value %i, invalid number format: %s", cnt, token.c_str());
        } catch (const std::out_of_range& e) {
          RCLCPP_ERROR(this->get_logger(), "Value %i, number out of range: %s", cnt, token.c_str());
        }
        cnt++;
      }
      if (cnt != 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of values: %i", cnt - 1);
      } else {
        contact_state_msg_.header.stamp = this->get_clock()->now();
        contact_state_publisher_->publish(contact_state_msg_);
      }
    } catch (const LibSerial::ReadTimeout&) {
      // Handle read timeout if necessary
      RCLCPP_ERROR(this->get_logger(), "Serial read timeout occurred");
    }
  }
}

ContactSensorDriver::~ContactSensorDriver() { serial_port_.Close(); }

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactSensorDriver>();
  node->Run();
  rclcpp::shutdown();
  return 0;
}

#include "tools/log_cpu_power.hpp"

CPUPowerLoggingNode::CPUPowerLoggingNode(const std::string &nodeName,
                                         uint64_t prev_energy_,
                                         std::chrono::steady_clock::time_point prev_time_)
    : Node(nodeName) {
  cpu_power_publisher_ = this->create_publisher<std_msgs::msg::Float64>("cpu_power_consumption", 10);
  this->declare_parameter("logging_interval", rclcpp::ParameterType::PARAMETER_DOUBLE);
  double logging_interval = this->get_parameter("logging_interval").as_double();
  logging_timer_ = rclcpp::create_timer(this,
                                        this->get_clock(),
                                        std::chrono::duration<double>(logging_interval),
                                        std::bind(&CPUPowerLoggingNode::TimerCallback, this));
  this->prev_energy_ = prev_energy_;
  this->prev_time_ = prev_time_;
}

void CPUPowerLoggingNode::TimerCallback() {
  double power = GetCPUPower();
  if (power >= 0.0) {
    auto message = std_msgs::msg::Float64();
    message.data = power;
    cpu_power_publisher_->publish(message);
    //    RCLCPP_INFO(this->get_logger(), "Published CPU Power: %.2f W", power);
  }
}

double CPUPowerLoggingNode::GetCPUPower() {
  double cpu_power_consumption = -1.0;
#if defined(__x86_64__)
  try {
    std::ifstream cpu_energy_file("/sys/class/powercap/intel-rapl:0/energy_uj");
    if (cpu_energy_file.is_open()) {
      std::string line;
      std::getline(cpu_energy_file, line);

      uint64_t cpu_energy = std::stoull(line);

      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time_).count();

      if (duration > 0) {
        cpu_power_consumption = static_cast<double>(cpu_energy - prev_energy_) / duration;
      }

      prev_energy_ = cpu_energy;
      prev_time_ = now;

      cpu_energy_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not open energy_uj file.");
      rclcpp::shutdown();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading power consumption: %s", e.what());
  }
#elif defined(__aarch64__)
  try {
    std::array<char, 128> buffer;
    std::string result;
    // const char* command = "sudo tegrastats --interval 1 | head -n1 | awk \'{for (I=1;I<NF;I++) if ($I == \"GPU\")
    // {print $(I+3)}} | sed \"s/[mW\\/]/ /g\" | cut -d \" \" -f1"; const char* command = "sudo tegrastats --interval 1
    // | head -n1 | grep -P \"CPU \\d+\" -o | cut -d \" \" -f2";
    const char *command =
        "sudo tegrastats --interval 1 | head -n1 | grep -P \"VDD_CPU_GPU_CV \\d+\" -o | cut -d \" \" -f2";
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
    if (!pipe) {
      RCLCPP_ERROR(this->get_logger(), "Failed to run tegrastats");
      return 0.0;
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    cpu_power_consumption = (std::stod(result) / 1000);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading power consumption: %s", e.what());
  }
#endif
  return cpu_power_consumption;
}

// #if defined(__aarch64__)
//  std::string parse_cpu_power(const std::string& tegrastats_output) {
//   std::regex cpu_power_regex(R"(CPU (\d+))");
//   std::smatch match;
//   if (std::regex_search(tegrastats_output, match, cpu_power_regex)) {
//     std::cout << match.str(1);
//     return match.str(1) + " µW";
//   }
//   return "";
// }
// #endif

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      std::make_shared<CPUPowerLoggingNode>("cpu_power_logging_node", 0, std::chrono::steady_clock::now());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <array>
#include <common/quaternion_operations.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include "common/custom_qos.hpp"
#include "common/quad_state.hpp"
#include "interfaces/msg/simulation_disturbance.hpp"
#include "interfaces/srv/disturb_sim.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class DisturbanceNode : public rclcpp::Node {
 public:
  DisturbanceNode() : rclcpp::Node("disturbance_node"), orientation_(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)) {
    quad_state_sub_ = this->create_subscription<interfaces::msg::QuadState>(
        "quad_state", QOS_RELIABLE_NO_DEPTH, std::bind(&DisturbanceNode::quad_state_callback, this, _1));

    dist_service_ = create_service<interfaces::srv::DisturbSim>(
        "disturb_simulation",
        std::bind(&DisturbanceNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    sim_dist_pub_ =
        this->create_publisher<interfaces::msg::SimulationDisturbance>("simulation_disturbance", QOS_RELIABLE_NO_DEPTH);
  }

  void quad_state_callback(const std::shared_ptr<interfaces::msg::QuadState> msg) {
    Eigen::Quaterniond orientation;
    orientation.w() = msg->pose.pose.orientation.w;
    orientation.x() = msg->pose.pose.orientation.x;
    orientation.y() = msg->pose.pose.orientation.y;
    orientation.z() = msg->pose.pose.orientation.z;
    orientation_ = yaw_quaternion_from_quaternion(orientation);
  }

  void service_callback(const std::shared_ptr<interfaces::srv::DisturbSim::Request> request,
                        std::shared_ptr<interfaces::srv::DisturbSim::Response> response) {
    // Publish the first message immediately
    auto dist = interfaces::msg::SimulationDisturbance();
    Eigen::Vector3d f, t;
    f(0) = request->force[0];
    f(1) = request->force[1];
    f(2) = request->force[2];
    t(0) = request->tau[0];
    t(1) = request->tau[1];
    t(2) = request->tau[2];
    f = orientation_ * f;
    t = orientation_ * t;
    dist.force[0] = f(0);
    dist.force[1] = f(1);
    dist.force[2] = f(2);
    dist.tau[0] = t(0);
    dist.tau[1] = t(1);
    dist.tau[2] = t(2);

    sim_dist_pub_->publish(dist);

    this->get_clock()->sleep_for(rclcpp::Duration(std::chrono::nanoseconds(static_cast<int64_t>(request->time * 1e9))));
    send_zero_force();

    response->success = true;
  }

  void send_zero_force() {
    interfaces::msg::SimulationDisturbance dist;
    sim_dist_pub_->publish(dist);
  }

 private:
  Eigen::Quaterniond orientation_;
  rclcpp::Service<interfaces::srv::DisturbSim>::SharedPtr dist_service_;
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_sub_;
  rclcpp::Publisher<interfaces::msg::SimulationDisturbance>::SharedPtr sim_dist_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<DisturbanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
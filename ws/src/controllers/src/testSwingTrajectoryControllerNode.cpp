#include <array>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "SwingTrajectory.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "common/custom_qos.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SwingTrajectoryControllerNode : public rclcpp::Node {
 public:
  SwingTrajectoryControllerNode() : Node("swing_trajectory_controller_node") {
    update_freq_ = 500;

    // Publishers
    leg_cmd_pub_ = this->create_publisher<interfaces::msg::LegCmd>("leg_cmd", QOS_RELIABLE_NO_DEPTH);

    // Timers
    update_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::microseconds(static_cast<int>(1e6 / update_freq_)),
                                         std::bind(&SwingTrajectoryControllerNode::do_swing_control, this));

    pInit_ << 0, 0, 0;
    pFinal_ << 0, 0, 0;
    height_ = 0.05;
    footSwingTrajectory_.setInitialPosition(pInit_);
    footSwingTrajectory_.setFinalPosition(pFinal_);
    footSwingTrajectory_.setHeight(height_);
    dt_ = 1.0 / update_freq_;
    swingTime_ = 2.0;
    idleTime_ = 2.0;
    iterCounter_ = 0;
    stancePhase = false;
  }
  // main update function, do back and forth
  void do_swing_control() {
    leg_state_goal_ = interfaces::msg::LegCmd();
    leg_state_goal_.header.stamp = now();
    double timer = dt_ * iterCounter_;
    double phase = timer / swingTime_;
    if (timer <= swingTime_ && !stancePhase)  // swing phase start
    {
      footSwingTrajectory_.computeSwingTrajectoryBezier(phase, swingTime_);
      std::cout << " Foot position : " << footSwingTrajectory_.getPosition();
    } else if (stancePhase && timer < (idleTime_ + swingTime_))  // start of stance phase
    {
      footSwingTrajectory_.computeSwingTrajectoryBezier(1.0, idleTime_);
      stancePhase = true;
    } else if (timer == (idleTime_ + swingTime_)) {  // Reverse the trajectory
      if (footSwingTrajectory_.getPosition() == pFinal_) {
        footSwingTrajectory_.setInitialPosition(pFinal_);
        footSwingTrajectory_.setFinalPosition(pInit_);
      }
      if (footSwingTrajectory_.getPosition() == pInit_) {
        footSwingTrajectory_.setInitialPosition(pInit_);
        footSwingTrajectory_.setFinalPosition(pFinal_);
      }
      stancePhase = false;
      iterCounter_ = 0;
    }
    iterCounter_++;
    geometry_msgs::msg::Point pos;
    geometry_msgs::msg::Vector3 vel;
    pos.x = footSwingTrajectory_.getPosition()(0);
    pos.y = footSwingTrajectory_.getPosition()(1);
    pos.z = footSwingTrajectory_.getPosition()(2);
    vel.x = footSwingTrajectory_.getVelocity()(0);
    vel.y = footSwingTrajectory_.getVelocity()(1);
    vel.z = footSwingTrajectory_.getVelocity()(2);
    leg_state_goal_.ee_pos[0] = pos.x;
    leg_state_goal_.ee_pos[1] = pos.x;
    leg_state_goal_.ee_pos[2] = pos.x;
    leg_state_goal_.ee_vel[0] = vel.x;
    leg_state_goal_.ee_vel[1] = vel.x;
    leg_state_goal_.ee_vel[2] = vel.x;
    leg_state_goal_.kp[0] = 1.;
    leg_state_goal_.kp[1] = 1.;
    leg_state_goal_.kp[2] = 1.;
    leg_state_goal_.kd[0] = 1.;
    leg_state_goal_.kd[1] = 1.;
    leg_state_goal_.kd[2] = 1.;

    leg_cmd_pub_->publish(leg_state_goal_);
  }

 private:
  // -- publishers --
  rclcpp::Publisher<interfaces::msg::LegCmd>::SharedPtr leg_cmd_pub_;

  float update_freq_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  interfaces::msg::LegCmd leg_state_goal_;

  Vec3 pInit_, pFinal_;
  double height_, swingTime_, idleTime_, dt_, iterCounter_;  // swing from init to final and stay there for sometime
  FootSwingTrajectory footSwingTrajectory_;
  bool stancePhase;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwingTrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
#include <chrono>

#include "common/custom_qos.hpp"
#include "common/quad_model_symbolic.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_identification/SysIDTrajectory.hpp"

class SysIDLegControllerNode : public rclcpp::Node {
 public:
  SysIDLegControllerNode() : Node("sysid_leg_controller_node") {
    // parameters
    this->declare_parameter<int>("leg_no", 0);
    this->declare_parameter<double>("max_segments", 1.0);
    this->declare_parameter<std::vector<double>>("ellipsoid_position", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("ellipsoid_params", {1.0, 1.0, 1.0});
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("kd", 1.0);
    this->declare_parameter<double>("max_segment_time", 2.0);
    this->declare_parameter<double>("min_segment_time", 0.1);
    this->declare_parameter<double>("num_velocities", 10.0);

    // quad state subscriber (to get first ee position)
    quad_state_subscription_ = this->create_subscription<interfaces::msg::QuadState>(
        "quad_state",
        QOS_RELIABLE_NO_DEPTH,
        std::bind(&SysIDLegControllerNode::QuadStateUpdateCallback, this, std::placeholders::_1));
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    update_freq_ = 1000.0;
    dt_ = 1.0 / update_freq_;
    iter_counter_ = 0;
    segment_counter_ = 0;
    motion_counter_ = 0;
    max_segment_time_ = this->get_parameter("max_segment_time").as_double();
    min_segment_time_ = this->get_parameter("min_segment_time").as_double();
    segment_time_ = max_segment_time_;
    num_velocities_ = this->get_parameter("num_velocities").as_double();
    segment_time_step_ = (max_segment_time_ - min_segment_time_) / num_velocities_;
    legno_ = this->get_parameter("leg_no").as_int();
    max_segments_ = this->get_parameter("max_segments").as_double();
    // Publishers
    leg_cmd_pub_ = this->create_publisher<interfaces::msg::LegCmd>("leg_cmd", QOS_RELIABLE_NO_DEPTH);

    // Timers
    update_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::microseconds(static_cast<int>(1e6 / update_freq_)),
                                         std::bind(&SysIDLegControllerNode::perform_trajectory, this));
    // Trajectory Initialization
    Eigen::Vector<double, 12> ellipsoid_pos =
        Eigen::Map<const Eigen::Vector<double, 12>>(this->get_parameter("ellipsoid_position").as_double_array().data());
    foot_trajectory_ =
        std::make_unique<SysIdTrajectory>(ellipsoid_pos.block<3, 1>(legno_ * 3, 0),
                                          this->get_parameter("ellipsoid_params").as_double_array().data()[0],
                                          this->get_parameter("ellipsoid_params").as_double_array().data()[1],
                                          this->get_parameter("ellipsoid_params").as_double_array().data()[2]);
    foot_trajectory_->setSegmentTime(segment_time_);
    RCLCPP_INFO(this->get_logger(),
                "Ellipsoid Position: [%f, %f, %f]",
                foot_trajectory_->getEllipsoidPosition().x(),
                foot_trajectory_->getEllipsoidPosition().y(),
                foot_trajectory_->getEllipsoidPosition().z());
    RCLCPP_INFO(this->get_logger(),
                "Ellipsoid Parameters: a:%f, b:%f, c:%f",
                foot_trajectory_->getEllipsoidParams()[0],
                foot_trajectory_->getEllipsoidParams()[1],
                foot_trajectory_->getEllipsoidParams()[2]);
  }
  void QuadStateUpdateCallback(interfaces::msg::QuadState::SharedPtr quad_state_msg) {
    // TODO: get first EE position
    if (!first_quad_state_received_) {
      first_quad_state_received_ = true;
      Eigen::Vector<double, 12> joint_positions;
      joint_positions = Eigen::Map<Eigen::Vector<double, 12>>(quad_state_msg->joint_state.position.data());
      std::cout << "used joint positions " << joint_positions.block<3, 1>(legno_ * 3, 0).transpose() << std::endl;
      Eigen::Vector3d EE_pos_bf, EE_zero_pos_bf;
      EE_start_pos_ = quad_model_.GetFootPositionInBodyFrame(legno_, joint_positions.block<3, 1>(legno_ * 3, 0));
      std::cout << "Start EE position " << EE_start_pos_.transpose() << std::endl;
    }
  }
  void perform_trajectory() {
    if (first_quad_state_received_) {
      leg_state_goal_.header.stamp = now();
      // do trajectory
      double timer = dt_ * iter_counter_;
      if (segment_counter_ == 0) {  // go to start position on ellipsoidal shell
        Eigen::Vector3d EE_curr_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d EE_curr_vel = Eigen::Vector3d::Zero();
        double start_time = 2.0;
        if (timer <= start_time) {  // motion in process
          EE_curr_pos = Interpolate::cubicBezier(
              EE_start_pos_, foot_trajectory_->getInitialEEPositionCartesian(), timer / start_time);
          EE_curr_vel = Interpolate::cubicBezierFirstDerivative(
              EE_start_pos_, foot_trajectory_->getInitialEEPositionCartesian(), timer / start_time);
          for (int i = 0; i < 3; i++) {
            leg_state_goal_.ee_pos[legno_ * 3 + i] = EE_curr_pos(i);
            leg_state_goal_.ee_vel[legno_ * 3 + i] = EE_curr_vel(i);
            leg_state_goal_.kp[legno_ * 3 + i] = kp_;
            leg_state_goal_.kd[legno_ * 3 + i] = kd_;
          }
          iter_counter_++;
        } else {  // motion finished
          iter_counter_ = 0;
          segment_counter_++;
        }
      } else if (segment_counter_ <= max_segments_) {  // experiment in process
        if (timer == 0) {                              // start next segment
          std::cout << "Exp Progress: " << (segment_counter_ / max_segments_) * 100.0 << "%" << std::endl;
          foot_trajectory_->sampleNextEEPosition();
          foot_trajectory_->computeEllipsoidalTrajectory(timer);
          for (int i = 0; i < 3; i++) {
            leg_state_goal_.ee_pos[legno_ * 3 + i] = foot_trajectory_->getCurrentEEPositionCartesian()(i);
            leg_state_goal_.ee_vel[legno_ * 3 + i] = foot_trajectory_->getCurrentEEVelocityCartesian()(i);
            leg_state_goal_.kp[legno_ * 3 + i] = kp_;
            leg_state_goal_.kd[legno_ * 3 + i] = kd_;
          }
          iter_counter_++;
        } else if (timer <= foot_trajectory_->getSegmentTime()) {  // motion in process
          foot_trajectory_->computeEllipsoidalTrajectory(timer);
          for (int i = 0; i < 3; i++) {
            leg_state_goal_.ee_pos[legno_ * 3 + i] = foot_trajectory_->getCurrentEEPositionCartesian()(i);
            leg_state_goal_.ee_vel[legno_ * 3 + i] = foot_trajectory_->getCurrentEEVelocityCartesian()(i);
            leg_state_goal_.kp[legno_ * 3 + i] = kp_;
            leg_state_goal_.kd[legno_ * 3 + i] = kd_;
          }
          iter_counter_++;
        } else {  // motion finished
          iter_counter_ = 0;
          segment_counter_++;
          motion_counter_++;
          if (motion_counter_ >= max_segments_ / num_velocities_) {
            segment_time_ = segment_time_ - segment_time_step_;
            foot_trajectory_->setSegmentTime(segment_time_);
            motion_counter_ = 0;
          }
        }
      } else {  // experiment finished! shut donw node
        RCLCPP_INFO(this->get_logger(), "Finished! Shutting Down.");
        rclcpp::shutdown();
      }
      leg_cmd_pub_->publish(leg_state_goal_);
    }
  }

 private:
  double update_freq_, dt_, iter_counter_, segment_counter_, max_segments_, kp_, kd_, min_segment_time_,
      max_segment_time_, segment_time_step_, num_velocities_, motion_counter_, segment_time_;
  bool first_quad_state_received_;
  int legno_;
  Eigen::Vector3d EE_start_pos_;
  // quad model
  QuadModelSymbolic quad_model_;
  // subscribers
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_subscription_;
  // publishers
  rclcpp::Publisher<interfaces::msg::LegCmd>::SharedPtr leg_cmd_pub_;
  // timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  interfaces::msg::LegCmd leg_state_goal_;

  std::unique_ptr<SysIdTrajectory> foot_trajectory_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SysIDLegControllerNode>());
  rclcpp::shutdown();
  return 0;
}
#include "tools/plotting.hpp"

#include <array>
#include <eigen3/Eigen/Dense>
#include <vector>

GaitPlottingNode::GaitPlottingNode(const std::string& nodeName)
    : Node(nodeName), buffer_index_(0), cot_filter_(MovingAverage<double>(20)) {
  gait_state_subscription_ = this->create_subscription<interfaces::msg::GaitState>(
      "gait_state",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&GaitPlottingNode::GaitStateCallback, this, std::placeholders::_1));
  quad_state_subscription_ = this->create_subscription<interfaces::msg::QuadState>(
      "quad_state",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&GaitPlottingNode::QuadStateCallback, this, std::placeholders::_1));
  target_subscription_ = this->create_subscription<interfaces::msg::QuadControlTarget>(
      "quad_control_target",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&GaitPlottingNode::TargetCallback, this, std::placeholders::_1));
  plotting_freq_ = 5.0;
  plotting_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::duration<float>(1.0 / plotting_freq_),
                                         std::bind(&GaitPlottingNode::PlottingCallback, this));

  // init figure
  figure_ = matplot::figure(true);
  matplot::hold(false);
  ax_.push_back(matplot::subplot(figure_, 2, 1, 0));
  ax_.push_back(matplot::subplot(figure_, 2, 1, 1));
  ax_[0]->xlabel("Timestep");
  ax_[0]->ylabel("Leg index");
  ax_[0]->y2label("Phase (green) & duty factor (blue)");

  ax_[1]->xlabel("Timestep");
  ax_[1]->ylabel("Velocity (blue) & Velocity target (green)");
  ax_[1]->y2label("Cost of transport (red)");
}

void GaitPlottingNode::GaitStateCallback(interfaces::msg::GaitState::SharedPtr msg) {
  gait_state_ = *msg;
  for (int leg = 0; leg < 4; ++leg) {
    contact_buffer_[leg][buffer_index_] = 255 - 125 * ((double)gait_state_.contact[leg]);
    duty_buffer_[leg][buffer_index_] = leg + 0.5 + (1.0 - gait_state_.duty_factor[leg]);
    phase_buffer_[leg][buffer_index_] = leg + 0.5 + (1.0 - gait_state_.phase[leg]);
  }
  cot_buffer_[buffer_index_] = cot_filter_.get();
  Eigen::Vector2d v = {quad_state_.twist.twist.linear.x, quad_state_.twist.twist.linear.y};
  v_buffer_[buffer_index_] = v.norm();
  Eigen::Vector2d v_des = {target_state_.body_x_dot, target_state_.body_y_dot};

  v_des_buffer_[buffer_index_] = v_des.norm();

  buffer_index_ = ++buffer_index_ % BUFFER_SIZE;
}

void GaitPlottingNode::QuadStateCallback(interfaces::msg::QuadState::SharedPtr msg) {
  quad_state_ = *msg;
  double cot = 0.0;  // FIXME: update
  for (int i = 0; i < 12; ++i) {
    cot += (quad_state_.joint_state.effort[i]) * (quad_state_.joint_state.velocity[i]);
  }
  Eigen::Vector2d v = {quad_state_.twist.twist.linear.x, quad_state_.twist.twist.linear.y};
  cot /= 15.019 * 9.81 * v.norm();
  cot_filter_.update(cot);
}

void GaitPlottingNode::TargetCallback(interfaces::msg::QuadControlTarget::SharedPtr msg) { target_state_ = *msg; }

void GaitPlottingNode::PlottingCallback() {
  std::array<std::array<double, BUFFER_SIZE>, 4> seq;
  std::array<std::array<double, BUFFER_SIZE>, 4> duty;
  std::array<std::array<double, BUFFER_SIZE>, 4> phase;
  std::array<double, BUFFER_SIZE> cot;
  std::array<double, BUFFER_SIZE> v;
  std::array<double, BUFFER_SIZE> v_des;
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    cot[i] = cot_buffer_[(i + 1 + buffer_index_) % BUFFER_SIZE];
    v[i] = v_buffer_[(i + 1 + buffer_index_) % BUFFER_SIZE];
    v_des[i] = v_des_buffer_[(i + 1 + buffer_index_) % BUFFER_SIZE];
    for (int leg = 0; leg < 4; ++leg) {
      seq[leg][i] = contact_buffer_[leg][(i + 1 + buffer_index_) % BUFFER_SIZE];
      duty[leg][i] = duty_buffer_[leg][(i + 1 + buffer_index_) % BUFFER_SIZE];
      phase[leg][i] = phase_buffer_[leg][(i + 1 + buffer_index_) % BUFFER_SIZE];
    }
  }
  // auto ax1 = matplot::subplot(figure_, 2, 1, 0);
  matplot::image(ax_[0], seq, seq, seq);  // use rgb image to ensure color even when all states the same
  matplot::hold(ax_[0], true);
  for (int i = 0; i < 4; ++i) {
    matplot::plot(ax_[0], duty[i])->color("blue").line_width(3).display_name("");
    matplot::plot(ax_[0], phase[i])->color("green").line_width(3).display_name("");
  }
  auto lgd = matplot::legend(ax_[0], {"", "Duty factor", "Phase", "", "", "", "", "", ""});
  lgd->location(matplot::legend::general_alignment::bottomleft);
  lgd->box(false);
  lgd->inside(false);
  // lgd->horizontal(true);
  // ax_[0]->plot(cot)->use_y2(true).color("red").line_width(3);
  matplot::hold(ax_[0], false);

  // auto ax2 = matplot::subplot(figure_, 2, 1, 1);
  ax_[1]->plot(v)->color("blue").line_width(3).display_name("Velocity");
  ax_[1]->ylim({0.0, 0.6});
  matplot::hold(ax_[1], true);
  ax_[1]->plot(v_des)->color("green").line_width(3).display_name("Target velocity");
  ax_[1]->plot(cot)->use_y2(true).color("red").line_width(3).display_name("Cost of transport");
  matplot::hold(ax_[1], false);
  auto lgd2 = matplot::legend(true);
  lgd2->location(matplot::legend::general_alignment::topleft);
  lgd2->box(false);
  lgd2->inside(false);
  // lgd2->horizontal(true);
  figure_->draw();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<GaitPlottingNode>("gait_plotting_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include "tools/plot_gs.hpp"

#include <array>
#include <eigen3/Eigen/Dense>
#include <vector>

#define SIZE 21

GaitSequencePlottingNode::GaitSequencePlottingNode(const std::string& nodeName) : Node(nodeName) {
  gait_sequence_subscription_ = this->create_subscription<interfaces::msg::GaitSequence>(
      "gait_sequence",
      QOS_RELIABLE_NO_DEPTH,
      std::bind(&GaitSequencePlottingNode::GaitSequenceCallback, this, std::placeholders::_1));
  plotting_freq_ = 5.0;
  plotting_timer_ = rclcpp::create_timer(this,
                                         this->get_clock(),
                                         std::chrono::duration<float>(1.0 / plotting_freq_),
                                         std::bind(&GaitSequencePlottingNode::PlottingCallback, this));

  // init figure
  figure_ = matplot::figure(true);
  matplot::hold(false);
  ax_.push_back(matplot::subplot(figure_, 2, 1, 0));
  ax_.push_back(matplot::subplot(figure_, 2, 1, 1));
  ax_[0]->xlabel("Timestep");
  ax_[0]->ylabel("Leg index");
  ax_[0]->y2label("Phase (green) & duty factor (blue)");

  ax_[1]->xlabel("Timestep");
  // ax_[1]->ylabel("Velocity (blue) & Velocity target (green)");
  // ax_[1]->y2label("Cost of transport (red)");
}

void GaitSequencePlottingNode::GaitSequenceCallback(interfaces::msg::GaitSequence::SharedPtr msg) {
  gait_sequence_ = *msg;
  updated_ = true;
}

void GaitSequencePlottingNode::PlottingCallback() {
  if (updated_) {
    updated_ = false;
  } else {
    return;  // only plot new data
  }
  std::array<std::array<double, SIZE>, 4> seq;
  std::array<std::array<float, SIZE>, 4> time_seq;
  assert(gait_sequence_.contact_sequence_0.size() >= SIZE);

  // std::array<std::array<double, BUFFER_SIZE>, 4> duty;
  // std::array<std::array<double, BUFFER_SIZE>, 4> phase;
  // std::array<double, BUFFER_SIZE> cot;
  // std::array<double, BUFFER_SIZE> v;
  // std::array<double, BUFFER_SIZE> v_des;
  for (int i = 0; i < SIZE; ++i) {
    seq[0][i] = (1.0 - (double)gait_sequence_.contact_sequence_0[i]) * 255;
    seq[1][i] = (1.0 - (double)gait_sequence_.contact_sequence_1[i]) * 255;
    seq[2][i] = (1.0 - (double)gait_sequence_.contact_sequence_2[i]) * 255;
    seq[3][i] = (1.0 - (double)gait_sequence_.contact_sequence_3[i]) * 255;
  }
  time_seq[0] = (gait_sequence_.swing_time_sequence_0);
  time_seq[1] = (gait_sequence_.swing_time_sequence_1);
  time_seq[2] = (gait_sequence_.swing_time_sequence_2);
  time_seq[3] = (gait_sequence_.swing_time_sequence_3);

  // auto ax1 = matplot::subplot(figure_, 2, 1, 0);
  matplot::image(ax_[0], seq, seq, seq);  // use rgb image to ensure color even when all states the same
  // matplot::hold(ax_[0], true);
  // for (int i = 0; i < 4; ++i) {
  //   matplot::plot(ax_[0], duty[i])->color("blue").line_width(3).display_name("");
  //   matplot::plot(ax_[0], phase[i])->color("green").line_width(3).display_name("");
  // }
  // auto lgd = matplot::legend(ax_[0], {"", "Duty factor", "Phase", "", "", "", "", "", ""});
  // lgd->location(matplot::legend::general_alignment::bottomleft);
  // lgd->box(false);
  // lgd->inside(false);
  // // lgd->horizontal(true);
  // // ax_[0]->plot(cot)->use_y2(true).color("red").line_width(3);
  // matplot::hold(ax_[0], false);

  // auto ax2 = matplot::subplot(figure_, 2, 1, 1);
  ax_[1]->plot(time_seq);
  // ax_[1]->ylim({0.0, 0.6});
  // matplot::hold(ax_[1], true);
  // ax_[1]->plot(v_des)->color("green").line_width(3).display_name("Target velocity");
  // ax_[1]->plot(cot)->use_y2(true).color("red").line_width(3).display_name("Cost of transport");
  // matplot::hold(ax_[1], false);
  // auto lgd2 = matplot::legend(true);
  // lgd2->location(matplot::legend::general_alignment::topleft);
  // lgd2->box(false);
  // lgd2->inside(false);
  // // lgd2->horizontal(true);
  figure_->draw();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<GaitSequencePlottingNode>("gait_sequence_plotting_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
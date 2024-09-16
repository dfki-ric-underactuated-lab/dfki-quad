#include <array>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "common/custom_qos.hpp"
#include "common/quad_model_symbolic.hpp"
#include "common/quad_state.hpp"
#include "common/sequence_containers.hpp"
#include "csv.hpp"
#include "interfaces/msg/joint_cmd.hpp"
#include "interfaces/msg/joint_state.hpp"
#include "interfaces/msg/leg_cmd.hpp"
#include "interfaces/msg/quad_state.hpp"
#include "interfaces/msg/trajectory_status.hpp"
#include "rclcpp/rclcpp.hpp"

#define RAD2DEG 180.0 / M_PI
#define DEG2RAD M_PI / 180.
#define NUM_LEGS 4
#define NUM_JOINTS_PER_LEG 3
#define NUM_JOINTS NUM_LEGS* NUM_JOINTS_PER_LEG

using std::placeholders::_1;

struct ReplayState {
  enum State {
    kInitPose,
    kTrajectory,
    kExertion,
    kFlight,
    kLand,
    kDone,
  };
  State state = kInitPose;
};

class TrajectoryReplay : public rclcpp::Node {
 public:
  TrajectoryReplay() : Node("trajectory_replay") {
    // declare parameters
    this->declare_parameter("csv_name", "");
    this->declare_parameter("update_freq", 400.0);
    this->declare_parameter("state_machine", true);
    this->declare_parameter("publish_state", true);
    this->declare_parameter("cartesian_control", false);
    // use cartesian gains and not joint level gains (has to match controller in
    // leg_driver)
    this->declare_parameter("cartesian_stiffness", true);
    this->declare_parameter("xy_correction",
                            false);  // correct x and y coordinates of legs
    this->declare_parameter("repeat_n_times", 1);
    this->declare_parameter("repeat_from_timestep", 0);

    this->declare_parameter("kd_damping", 1.0);

    this->declare_parameter("kp_abad_init", 1.0);
    this->declare_parameter("kd_abad_init", 1.0);
    this->declare_parameter("kp_hip_init", 1.0);
    this->declare_parameter("kd_hip_init", 1.0);
    this->declare_parameter("kp_knee_init", 1.0);
    this->declare_parameter("kd_knee_init", 1.0);
    this->declare_parameter("kp_abad", 1.0);
    this->declare_parameter("kd_abad", 1.0);
    this->declare_parameter("kp_hip_front", 1.0);
    this->declare_parameter("kd_hip_front", 1.0);
    this->declare_parameter("kp_knee_front", 1.0);
    this->declare_parameter("kd_knee_front", 1.0);
    this->declare_parameter("kp_hip_back", 1.0);
    this->declare_parameter("kd_hip_back", 1.0);
    this->declare_parameter("kp_knee_back", 1.0);
    this->declare_parameter("kd_knee_back", 1.0);

    this->declare_parameter("kp_x_front_init", 1.0);
    this->declare_parameter("kp_y_front_init", 1.0);
    this->declare_parameter("kp_z_front_init", 1.0);
    this->declare_parameter("kp_x_back_init", 1.0);
    this->declare_parameter("kp_y_back_init", 1.0);
    this->declare_parameter("kp_z_back_init", 1.0);
    this->declare_parameter("kp_x_front_trajectory", 1.0);
    this->declare_parameter("kp_y_front_trajectory", 1.0);
    this->declare_parameter("kp_z_front_trajectory", 1.0);
    this->declare_parameter("kp_x_back_trajectory", 1.0);
    this->declare_parameter("kp_y_back_trajectory", 1.0);
    this->declare_parameter("kp_z_back_trajectory", 1.0);

    this->declare_parameter("kd_x_front_init", 1.0);
    this->declare_parameter("kd_y_front_init", 1.0);
    this->declare_parameter("kd_z_front_init", 1.0);
    this->declare_parameter("kd_x_back_init", 1.0);
    this->declare_parameter("kd_y_back_init", 1.0);
    this->declare_parameter("kd_z_back_init", 1.0);
    this->declare_parameter("kd_x_front_trajectory", 1.0);
    this->declare_parameter("kd_y_front_trajectory", 1.0);
    this->declare_parameter("kd_z_front_trajectory", 1.0);
    this->declare_parameter("kd_x_back_trajectory", 1.0);
    this->declare_parameter("kd_y_back_trajectory", 1.0);
    this->declare_parameter("kd_z_back_trajectory", 1.0);

    this->declare_parameter("initial_interpolation", false);
    this->declare_parameter("wait",
                            true);  // this parameter is used to start trajectory after init phase

    std::string default_urdf_path = ament_index_cpp::get_package_share_directory("common");
    default_urdf_path += "/model/urdf/quad.urdf";
    this->declare_parameter("urdf_path", default_urdf_path);

    std::string urdf_path = this->get_parameter("urdf_path").get_parameter_value().get<std::string>();

    // read parameters
    csv_path_ = ament_index_cpp::get_package_share_directory("controllers") + "/trajectories/"
                + this->get_parameter("csv_name").as_string();
    state_machine_ = this->get_parameter("state_machine").get_parameter_value().get<bool>();
    cartesian_ = this->get_parameter("cartesian_control").get_parameter_value().get<bool>();
    cartesian_stiffness_ = this->get_parameter("cartesian_stiffness").get_parameter_value().get<bool>();
    publish_state_ = this->get_parameter("publish_state").get_parameter_value().get<bool>();
    initial_interpolation_ = this->get_parameter("initial_interpolation").get_parameter_value().get<bool>();
    correct_xy_ = this->get_parameter("xy_correction").get_parameter_value().get<bool>();

    req_num_of_times_ = this->get_parameter("repeat_n_times").get_parameter_value().get<int>();
    repeat_from_timestep_ = this->get_parameter("repeat_from_timestep").get_parameter_value().get<int>();

    update_freq_ = this->get_parameter("update_freq").get_parameter_value().get<float>();
    kd_damping_ = this->get_parameter("kd_damping").get_parameter_value().get<float>();
    kp_abad_init_ = this->get_parameter("kp_abad_init").get_parameter_value().get<float>();
    kd_abad_init_ = this->get_parameter("kd_abad_init").get_parameter_value().get<float>();
    kp_hip_init_ = this->get_parameter("kp_hip_init").get_parameter_value().get<float>();
    kd_hip_init_ = this->get_parameter("kd_hip_init").get_parameter_value().get<float>();
    kp_knee_init_ = this->get_parameter("kp_knee_init").get_parameter_value().get<float>();
    kd_knee_init_ = this->get_parameter("kd_knee_init").get_parameter_value().get<float>();
    kp_abad_ = this->get_parameter("kp_abad").get_parameter_value().get<float>();
    kd_abad_ = this->get_parameter("kd_abad").get_parameter_value().get<float>();
    kp_hip_front_ = this->get_parameter("kp_hip_front").get_parameter_value().get<float>();
    kd_hip_front_ = this->get_parameter("kd_hip_front").get_parameter_value().get<float>();
    kp_knee_front_ = this->get_parameter("kp_knee_front").get_parameter_value().get<float>();
    kd_knee_front_ = this->get_parameter("kd_knee_front").get_parameter_value().get<float>();
    kp_hip_back_ = this->get_parameter("kp_hip_back").get_parameter_value().get<float>();
    kd_hip_back_ = this->get_parameter("kd_hip_back").get_parameter_value().get<float>();
    kp_knee_back_ = this->get_parameter("kp_knee_back").get_parameter_value().get<float>();
    kd_knee_back_ = this->get_parameter("kd_knee_back").get_parameter_value().get<float>();

    kp_x_front_init_ = this->get_parameter("kp_x_front_init").get_parameter_value().get<float>();
    kp_y_front_init_ = this->get_parameter("kp_y_front_init").get_parameter_value().get<float>();
    kp_z_front_init_ = this->get_parameter("kp_z_front_init").get_parameter_value().get<float>();
    kp_x_back_init_ = this->get_parameter("kp_x_back_init").get_parameter_value().get<float>();
    kp_y_back_init_ = this->get_parameter("kp_y_back_init").get_parameter_value().get<float>();
    kp_z_back_init_ = this->get_parameter("kp_z_back_init").get_parameter_value().get<float>();
    kp_x_front_trajectory_ = this->get_parameter("kp_x_front_trajectory").get_parameter_value().get<float>();
    kp_y_front_trajectory_ = this->get_parameter("kp_y_front_trajectory").get_parameter_value().get<float>();
    kp_z_front_trajectory_ = this->get_parameter("kp_z_front_trajectory").get_parameter_value().get<float>();
    kp_x_back_trajectory_ = this->get_parameter("kp_x_back_trajectory").get_parameter_value().get<float>();
    kp_y_back_trajectory_ = this->get_parameter("kp_y_back_trajectory").get_parameter_value().get<float>();
    kp_z_back_trajectory_ = this->get_parameter("kp_z_back_trajectory").get_parameter_value().get<float>();

    kd_x_front_init_ = this->get_parameter("kd_x_front_init").get_parameter_value().get<float>();
    kd_y_front_init_ = this->get_parameter("kd_y_front_init").get_parameter_value().get<float>();
    kd_z_front_init_ = this->get_parameter("kd_z_front_init").get_parameter_value().get<float>();
    kd_x_back_init_ = this->get_parameter("kd_x_back_init").get_parameter_value().get<float>();
    kd_y_back_init_ = this->get_parameter("kd_y_back_init").get_parameter_value().get<float>();
    kd_z_back_init_ = this->get_parameter("kd_z_back_init").get_parameter_value().get<float>();
    kd_x_front_trajectory_ = this->get_parameter("kd_x_front_trajectory").get_parameter_value().get<float>();
    kd_y_front_trajectory_ = this->get_parameter("kd_y_front_trajectory").get_parameter_value().get<float>();
    kd_z_front_trajectory_ = this->get_parameter("kd_z_front_trajectory").get_parameter_value().get<float>();
    kd_x_back_trajectory_ = this->get_parameter("kd_x_back_trajectory").get_parameter_value().get<float>();
    kd_y_back_trajectory_ = this->get_parameter("kd_y_back_trajectory").get_parameter_value().get<float>();
    kd_z_back_trajectory_ = this->get_parameter("kd_z_back_trajectory").get_parameter_value().get<float>();

    // read trajectory
    if (cartesian_) {
      read_cartesian_Trajectory();
    } else if (state_machine_) {
      // contact information is required separate states
      read_Trajectory_contact();
    } else {
      read_Trajectory();
    }

    // initialize params
    replay_state_.state = ReplayState::State::kInitPose;

    // map kp and kd values to joint indices
    kp_map_[0] = kp_abad_;
    kp_map_[1] = kp_hip_front_;
    kp_map_[2] = kp_knee_front_;
    kp_map_[3] = kp_abad_;
    kp_map_[4] = kp_hip_front_;
    kp_map_[5] = kp_knee_front_;
    kp_map_[6] = kp_abad_;
    kp_map_[7] = kp_hip_back_;
    kp_map_[8] = kp_knee_back_;
    kp_map_[9] = kp_abad_;
    kp_map_[10] = kp_hip_back_;
    kp_map_[11] = kp_knee_back_;

    kd_map_[0] = kd_abad_;
    kd_map_[1] = kd_hip_front_;
    kd_map_[2] = kd_knee_front_;
    kd_map_[3] = kd_abad_;
    kd_map_[4] = kd_hip_front_;
    kd_map_[5] = kd_knee_front_;
    kd_map_[6] = kd_abad_;
    kd_map_[7] = kd_hip_back_;
    kd_map_[8] = kd_knee_back_;
    kd_map_[9] = kd_abad_;
    kd_map_[10] = kd_hip_back_;
    kd_map_[11] = kd_knee_back_;

    kp_init_map_[0] = kp_abad_init_;
    kp_init_map_[1] = kp_hip_init_;
    kp_init_map_[2] = kp_knee_init_;
    kp_init_map_[3] = kp_abad_init_;
    kp_init_map_[4] = kp_hip_init_;
    kp_init_map_[5] = kp_knee_init_;
    kp_init_map_[6] = kp_abad_init_;
    kp_init_map_[7] = kp_hip_init_;
    kp_init_map_[8] = kp_knee_init_;
    kp_init_map_[9] = kp_abad_init_;
    kp_init_map_[10] = kp_hip_init_;
    kp_init_map_[11] = kp_knee_init_;

    kd_init_map_[0] = kd_abad_init_;
    kd_init_map_[1] = kd_hip_init_;
    kd_init_map_[2] = kd_knee_init_;
    kd_init_map_[3] = kd_abad_init_;
    kd_init_map_[4] = kd_hip_init_;
    kd_init_map_[5] = kd_knee_init_;
    kd_init_map_[6] = kd_abad_init_;
    kd_init_map_[7] = kd_hip_init_;
    kd_init_map_[8] = kd_knee_init_;
    kd_init_map_[9] = kd_abad_init_;
    kd_init_map_[10] = kd_hip_init_;
    kd_init_map_[11] = kd_knee_init_;

    cartesian_kp_map_[0] = kp_x_front_trajectory_;
    cartesian_kp_map_[1] = kp_y_front_trajectory_;
    cartesian_kp_map_[2] = kp_z_front_trajectory_;
    cartesian_kp_map_[3] = kp_x_front_trajectory_;
    cartesian_kp_map_[4] = kp_y_front_trajectory_;
    cartesian_kp_map_[5] = kp_z_front_trajectory_;
    cartesian_kp_map_[6] = kp_x_back_trajectory_;
    cartesian_kp_map_[7] = kp_y_back_trajectory_;
    cartesian_kp_map_[8] = kp_z_back_trajectory_;
    cartesian_kp_map_[9] = kp_x_back_trajectory_;
    cartesian_kp_map_[10] = kp_y_back_trajectory_;
    cartesian_kp_map_[11] = kp_z_back_trajectory_;

    cartesian_kd_map_[0] = kd_x_front_trajectory_;
    cartesian_kd_map_[1] = kd_y_front_trajectory_;
    cartesian_kd_map_[2] = kd_z_front_trajectory_;
    cartesian_kd_map_[3] = kd_x_front_trajectory_;
    cartesian_kd_map_[4] = kd_y_front_trajectory_;
    cartesian_kd_map_[5] = kd_z_front_trajectory_;
    cartesian_kd_map_[6] = kd_x_back_trajectory_;
    cartesian_kd_map_[7] = kd_y_back_trajectory_;
    cartesian_kd_map_[8] = kd_z_back_trajectory_;
    cartesian_kd_map_[9] = kd_x_back_trajectory_;
    cartesian_kd_map_[10] = kd_y_back_trajectory_;
    cartesian_kd_map_[11] = kd_z_back_trajectory_;

    cartesian_kp_init_map_[0] = kp_x_front_init_;
    cartesian_kp_init_map_[1] = kp_y_front_init_;
    cartesian_kp_init_map_[2] = kp_z_front_init_;
    cartesian_kp_init_map_[3] = kp_x_front_init_;
    cartesian_kp_init_map_[4] = kp_y_front_init_;
    cartesian_kp_init_map_[5] = kp_z_front_init_;
    cartesian_kp_init_map_[6] = kp_x_back_init_;
    cartesian_kp_init_map_[7] = kp_y_back_init_;
    cartesian_kp_init_map_[8] = kp_z_back_init_;
    cartesian_kp_init_map_[9] = kp_x_back_init_;
    cartesian_kp_init_map_[10] = kp_y_back_init_;
    cartesian_kp_init_map_[11] = kp_z_back_init_;

    cartesian_kd_init_map_[0] = kd_x_front_init_;
    cartesian_kd_init_map_[1] = kd_y_front_init_;
    cartesian_kd_init_map_[2] = kd_z_front_init_;
    cartesian_kd_init_map_[3] = kd_x_front_init_;
    cartesian_kd_init_map_[4] = kd_y_front_init_;
    cartesian_kd_init_map_[5] = kd_z_front_init_;
    cartesian_kd_init_map_[6] = kd_x_back_init_;
    cartesian_kd_init_map_[7] = kd_y_back_init_;
    cartesian_kd_init_map_[8] = kd_z_back_init_;
    cartesian_kd_init_map_[9] = kd_x_back_init_;
    cartesian_kd_init_map_[10] = kd_y_back_init_;
    cartesian_kd_init_map_[11] = kd_z_back_init_;

    time_trajectory_ = 0.;
    interpolate_time_ = 0.;
    temp_time_ = 0.;
    num_of_times_ = 0;
    quad_state_received_ = false;

    // -- publishers --
    if (cartesian_) {
      leg_cmd_pub_ = this->create_publisher<interfaces::msg::LegCmd>("leg_cmd", QOS_RELIABLE_NO_DEPTH);
    } else {
      joint_cmd_pub_ = this->create_publisher<interfaces::msg::JointCmd>("leg_joint_cmd", QOS_RELIABLE_NO_DEPTH);
    }
    if (publish_state_) {
      state_pub_ = this->create_publisher<interfaces::msg::TrajectoryStatus>("trajectory_state", QOS_RELIABLE_NO_DEPTH);
    }

    // -- subscribers --
    // currently only required when we want to interpolate position in init
    // state
    if (initial_interpolation_) {
      quad_state_sub_ = this->create_subscription<interfaces::msg::QuadState>(
          "quad_state", QOS_RELIABLE_NO_DEPTH, std::bind(&TrajectoryReplay::quad_state_callback, this, _1));
    }

    // -- timers --
    if (cartesian_) {
      replay_timer_ = rclcpp::create_timer(this,
                                           this->get_clock(),
                                           std::chrono::duration<float>(1.0 / update_freq_),
                                           std::bind(&TrajectoryReplay::calc_leg_states_goal_, this));
    } else {
      replay_timer_ = rclcpp::create_timer(this,
                                           this->get_clock(),
                                           std::chrono::duration<float>(1.0 / update_freq_),
                                           std::bind(&TrajectoryReplay::calc_joint_states_goal_, this));
    }
  }

  void read_Trajectory() {
    // Clear contents of the vector
    y_vec.clear();
    yd_vec.clear();
    Tau_vec.clear();
    try {
      io::CSVReader<36> in(csv_path_);

      in.read_header(io::ignore_extra_column,
                     "q_fl1",
                     "qd_fl1",
                     "Tau_fl1",
                     "q_fl2",
                     "qd_fl2",
                     "Tau_fl2",
                     "q_fl3",
                     "qd_fl3",
                     "Tau_fl3",  // front left leg
                     "q_fr1",
                     "qd_fr1",
                     "Tau_fr1",
                     "q_fr2",
                     "qd_fr2",
                     "Tau_fr2",
                     "q_fr3",
                     "qd_fr3",
                     "Tau_fr3",  // front right leg
                     "q_bl1",
                     "qd_bl1",
                     "Tau_bl1",
                     "q_bl2",
                     "qd_bl2",
                     "Tau_bl2",
                     "q_bl3",
                     "qd_bl3",
                     "Tau_bl3",  // back left leg
                     "q_br1",
                     "qd_br1",
                     "Tau_br1",
                     "q_br2",
                     "qd_br2",
                     "Tau_br2",
                     "q_br3",
                     "qd_br3",
                     "Tau_br3"  // back right leg
      );

      double q_fl1, qd_fl1, Tau_fl1, q_fl2, qd_fl2, Tau_fl2, q_fl3, qd_fl3,
          Tau_fl3;  // front left leg
      double q_fr1, qd_fr1, Tau_fr1, q_fr2, qd_fr2, Tau_fr2, q_fr3, qd_fr3,
          Tau_fr3;  // front right leg
      double q_bl1, qd_bl1, Tau_bl1, q_bl2, qd_bl2, Tau_bl2, q_bl3, qd_bl3,
          Tau_bl3;  // back left leg
      double q_br1, qd_br1, Tau_br1, q_br2, qd_br2, Tau_br2, q_br3, qd_br3,
          Tau_br3;  // back right leg

      while (in.read_row(q_fl1,
                         qd_fl1,
                         Tau_fl1,
                         q_fl2,
                         qd_fl2,
                         Tau_fl2,
                         q_fl3,
                         qd_fl3,
                         Tau_fl3,  // front left leg
                         q_fr1,
                         qd_fr1,
                         Tau_fr1,
                         q_fr2,
                         qd_fr2,
                         Tau_fr2,
                         q_fr3,
                         qd_fr3,
                         Tau_fr3,  // front right leg
                         q_bl1,
                         qd_bl1,
                         Tau_bl1,
                         q_bl2,
                         qd_bl2,
                         Tau_bl2,
                         q_bl3,
                         qd_bl3,
                         Tau_bl3,  // back left leg
                         q_br1,
                         qd_br1,
                         Tau_br1,
                         q_br2,
                         qd_br2,
                         Tau_br2,
                         q_br3,
                         qd_br3,
                         Tau_br3  // back right leg
                         )) {
        // extract joint coordinates
        Eigen::VectorXd y = Eigen::VectorXd::Zero(12);
        y(0) = q_fl1, y(1) = q_fl2, y(2) = q_fl3, y(3) = q_fr1, y(4) = q_fr2, y(5) = q_fr3;
        y(6) = q_bl1, y(7) = q_bl2, y(8) = q_bl3, y(9) = q_br1, y(10) = q_br2, y(11) = q_br3;

        Eigen::VectorXd yd = Eigen::VectorXd::Zero(12);
        yd(0) = qd_fl1, yd(1) = qd_fl2, yd(2) = qd_fl3, yd(3) = qd_fr1, yd(4) = qd_fr2, yd(5) = qd_fr3;
        yd(6) = qd_bl1, yd(7) = qd_bl2, yd(8) = qd_bl3, yd(9) = qd_br1, yd(10) = qd_br2, yd(11) = qd_br3;

        Eigen::VectorXd Tau = Eigen::VectorXd::Zero(12);
        Tau(0) = Tau_fl1, Tau(1) = Tau_fl2, Tau(2) = Tau_fl3, Tau(3) = Tau_fr1, Tau(4) = Tau_fr2, Tau(5) = Tau_fr3;
        Tau(6) = Tau_bl1, Tau(7) = Tau_bl2, Tau(8) = Tau_bl3, Tau(9) = Tau_br1, Tau(10) = Tau_br2, Tau(11) = Tau_br3;

        y_vec.push_back(y);
        yd_vec.push_back(yd);
        Tau_vec.push_back(Tau);
      }
      time_trajectory_ = y_vec.size();
      y_vec_initPose = y_vec[0];
      std::cout << "CSV file read, total rows: " << time_trajectory_ << std::endl;
    } catch (const io::error::can_not_open_file& e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Warning: cannot open CSV file " + csv_path_);
      return;
    }
  }

  void read_Trajectory_contact() {
    // Clear contents of the vector
    y_vec.clear();
    yd_vec.clear();
    Tau_vec.clear();
    try {
      io::CSVReader<37> in(csv_path_);

      in.read_header(io::ignore_extra_column,
                     "q_fl1",
                     "qd_fl1",
                     "Tau_fl1",
                     "q_fl2",
                     "qd_fl2",
                     "Tau_fl2",
                     "q_fl3",
                     "qd_fl3",
                     "Tau_fl3",  // front left leg
                     "q_fr1",
                     "qd_fr1",
                     "Tau_fr1",
                     "q_fr2",
                     "qd_fr2",
                     "Tau_fr2",
                     "q_fr3",
                     "qd_fr3",
                     "Tau_fr3",  // front right leg
                     "q_bl1",
                     "qd_bl1",
                     "Tau_bl1",
                     "q_bl2",
                     "qd_bl2",
                     "Tau_bl2",
                     "q_bl3",
                     "qd_bl3",
                     "Tau_bl3",  // back left leg
                     "q_br1",
                     "qd_br1",
                     "Tau_br1",
                     "q_br2",
                     "qd_br2",
                     "Tau_br2",
                     "q_br3",
                     "qd_br3",
                     "Tau_br3",  // back right leg
                     "contactPhases");

      double q_fl1, qd_fl1, Tau_fl1, q_fl2, qd_fl2, Tau_fl2, q_fl3, qd_fl3,
          Tau_fl3;  // front left leg
      double q_fr1, qd_fr1, Tau_fr1, q_fr2, qd_fr2, Tau_fr2, q_fr3, qd_fr3,
          Tau_fr3;  // front right leg
      double q_bl1, qd_bl1, Tau_bl1, q_bl2, qd_bl2, Tau_bl2, q_bl3, qd_bl3,
          Tau_bl3;  // back left leg
      double q_br1, qd_br1, Tau_br1, q_br2, qd_br2, Tau_br2, q_br3, qd_br3,
          Tau_br3;  // back right leg
      int contactPhases;
      while (in.read_row(q_fl1,
                         qd_fl1,
                         Tau_fl1,
                         q_fl2,
                         qd_fl2,
                         Tau_fl2,
                         q_fl3,
                         qd_fl3,
                         Tau_fl3,  // front left leg
                         q_fr1,
                         qd_fr1,
                         Tau_fr1,
                         q_fr2,
                         qd_fr2,
                         Tau_fr2,
                         q_fr3,
                         qd_fr3,
                         Tau_fr3,  // front right leg
                         q_bl1,
                         qd_bl1,
                         Tau_bl1,
                         q_bl2,
                         qd_bl2,
                         Tau_bl2,
                         q_bl3,
                         qd_bl3,
                         Tau_bl3,  // back left leg
                         q_br1,
                         qd_br1,
                         Tau_br1,
                         q_br2,
                         qd_br2,
                         Tau_br2,
                         q_br3,
                         qd_br3,
                         Tau_br3,  // back right leg
                         contactPhases)) {
        // extract joint coordinates
        Eigen::VectorXd y = Eigen::VectorXd::Zero(12);
        y(0) = q_fl1, y(1) = q_fl2, y(2) = q_fl3, y(3) = q_fr1, y(4) = q_fr2, y(5) = q_fr3;
        y(6) = q_bl1, y(7) = q_bl2, y(8) = q_bl3, y(9) = q_br1, y(10) = q_br2, y(11) = q_br3;

        Eigen::VectorXd yd = Eigen::VectorXd::Zero(12);
        yd(0) = qd_fl1, yd(1) = qd_fl2, yd(2) = qd_fl3, yd(3) = qd_fr1, yd(4) = qd_fr2, yd(5) = qd_fr3;
        yd(6) = qd_bl1, yd(7) = qd_bl2, yd(8) = qd_bl3, yd(9) = qd_br1, yd(10) = qd_br2, yd(11) = qd_br3;

        Eigen::VectorXd Tau = Eigen::VectorXd::Zero(12);
        Tau(0) = Tau_fl1, Tau(1) = Tau_fl2, Tau(2) = Tau_fl3, Tau(3) = Tau_fr1, Tau(4) = Tau_fr2, Tau(5) = Tau_fr3;
        Tau(6) = Tau_bl1, Tau(7) = Tau_bl2, Tau(8) = Tau_bl3, Tau(9) = Tau_br1, Tau(10) = Tau_br2, Tau(11) = Tau_br3;

        y_vec.push_back(y);
        yd_vec.push_back(yd);
        Tau_vec.push_back(Tau);
        contactPhases_vec.push_back(contactPhases);
      }
      time_trajectory_ = y_vec.size();
      y_vec_initPose = y_vec[0];

      int i = 0;
      while (contactPhases_vec[i] && i < time_trajectory_) {
        yExertion.push_back(y_vec[i]);
        ydExertion.push_back(yd_vec[i]);
        TauExertion.push_back(Tau_vec[i]);
        contactExertion.push_back(contactPhases_vec[i]);
        i++;
      }
      while (!contactPhases_vec[i] && i < time_trajectory_) {
        yFlight.push_back(y_vec[i]);
        ydFlight.push_back(yd_vec[i]);
        TauFlight.push_back(Tau_vec[i]);
        contactFlight.push_back(contactPhases_vec[i]);
        i++;
      }
      while (i < time_trajectory_) {
        yLand.push_back(y_vec[i]);
        ydLand.push_back(yd_vec[i]);
        TauLand.push_back(Tau_vec[i]);
        contactLand.push_back(contactPhases_vec[i]);
        i++;
      }
    } catch (const io::error::can_not_open_file& e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Warning: cannot open CSV file");
      return;
    }
  }

  void read_cartesian_Trajectory() {
    // Clear contents of the vector
    y_vec.clear();
    yd_vec.clear();
    Tau_vec.clear();
    try {
      io::CSVReader<36> in(csv_path_);

      in.read_header(io::ignore_extra_column,
                     "x_fl",
                     "xd_fl",
                     "Fx_fl",
                     "y_fl",
                     "yd_fl",
                     "Fy_fl",
                     "z_fl",
                     "zd_fl",
                     "Fz_fl",  // front left leg
                     "x_fr",
                     "xd_fr",
                     "Fx_fr",
                     "y_fr",
                     "yd_fr",
                     "Fy_fr",
                     "z_fr",
                     "zd_fr",
                     "Fz_fr",  // front right leg
                     "x_bl",
                     "xd_bl",
                     "Fx_bl",
                     "y_bl",
                     "yd_bl",
                     "Fy_bl",
                     "z_bl",
                     "zd_bl",
                     "Fz_bl",  // back left leg
                     "x_br",
                     "xd_br",
                     "Fx_br",
                     "y_br",
                     "yd_br",
                     "Fy_br",
                     "z_br",
                     "zd_br",
                     "Fz_br"  // back right leg
      );

      double x_fl, xd_fl, Fx_fl, y_fl, yd_fl, Fy_fl, z_fl, zd_fl,
          Fz_fl;  // front left leg
      double x_fr, xd_fr, Fx_fr, y_fr, yd_fr, Fy_fr, z_fr, zd_fr,
          Fz_fr;  // front right leg
      double x_bl, xd_bl, Fx_bl, y_bl, yd_bl, Fy_bl, z_bl, zd_bl,
          Fz_bl;  // back left leg
      double x_br, xd_br, Fx_br, y_br, yd_br, Fy_br, z_br, zd_br,
          Fz_br;  // back right leg

      while (in.read_row(x_fl,
                         xd_fl,
                         Fx_fl,
                         y_fl,
                         yd_fl,
                         Fy_fl,
                         z_fl,
                         zd_fl,
                         Fz_fl,  // front left leg
                         x_fr,
                         xd_fr,
                         Fx_fr,
                         y_fr,
                         yd_fr,
                         Fy_fr,
                         z_fr,
                         zd_fr,
                         Fz_fr,  // front right leg
                         x_bl,
                         xd_bl,
                         Fx_bl,
                         y_bl,
                         yd_bl,
                         Fy_bl,
                         z_bl,
                         zd_bl,
                         Fz_bl,  // back left leg
                         x_br,
                         xd_br,
                         Fx_br,
                         y_br,
                         yd_br,
                         Fy_br,
                         z_br,
                         zd_br,
                         Fz_br  // back right leg
                         )) {
        // extract joint coordinates
        Eigen::Matrix<double, 4, 3> x;
        x(0, 0) = x_fl, x(0, 1) = y_fl, x(0, 2) = z_fl, x(1, 0) = x_fr, x(1, 1) = y_fr, x(1, 2) = z_fr;
        x(2, 0) = x_bl, x(2, 1) = y_bl, x(2, 2) = z_bl, x(3, 0) = x_br, x(3, 1) = y_br, x(3, 2) = z_br;

        Eigen::Matrix<double, 4, 3> v;
        v(0, 0) = xd_fl, v(0, 1) = yd_fl, v(0, 2) = zd_fl, v(1, 0) = xd_fr, v(1, 1) = yd_fr, v(1, 2) = zd_fr;
        v(2, 0) = xd_bl, v(2, 1) = yd_bl, v(2, 2) = zd_bl, v(3, 0) = xd_br, v(3, 1) = yd_br, v(3, 2) = zd_br;

        Eigen::Matrix<double, 4, 3> F;
        F(0, 0) = Fx_fl, F(0, 1) = Fy_fl, F(0, 2) = Fz_fl, F(1, 0) = Fx_fr, F(1, 1) = Fy_fr, F(1, 2) = Fz_fr;
        F(2, 0) = Fx_bl, F(2, 1) = Fy_bl, F(2, 2) = Fz_bl, F(3, 0) = Fx_br, F(3, 1) = Fy_br, F(3, 2) = Fz_br;

        x_vec.push_back(x);
        v_vec.push_back(v);
        F_vec.push_back(F);
      }
      time_trajectory_ = x_vec.size();
      x_vec_initPose = x_vec[0];
      std::cout << "CSV file read, total rows: " << time_trajectory_ << std::endl;
    } catch (const io::error::can_not_open_file& e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Warning: cannot open CSV file " + csv_path_);
      return;
    }
  }

  void DoControl_ZeroVelocity() {
    joint_state_goal_ = interfaces::msg::JointCmd();
    joint_state_goal_.header.stamp = now();
    for (int i = 0; i < NUM_JOINTS; i++) {
      joint_state_goal_.position[i] = 0.;
      joint_state_goal_.velocity[i] = 0.;
      joint_state_goal_.effort[i] = 0.;
      joint_state_goal_.kp[i] = 0.;
      joint_state_goal_.kd[i] = kd_damping_;
    }
  }

  void DoControl_CartesianZeroVelocity() {
    leg_state_goal_ = interfaces::msg::LegCmd();
    leg_state_goal_.header.stamp = now();

    leg_state_goal_.ee_pos.fill(0.0);
    leg_state_goal_.ee_vel.fill(0.0);
    leg_state_goal_.ee_force.fill(0.0);

    leg_state_goal_.kp.fill(0.0);
    leg_state_goal_.kd.fill(kd_damping_);
  }

  void DoControl_Trajectory_state_machine(Eigen::VectorXd y, Eigen::VectorXd yd, Eigen::VectorXd Tau) {
    // std::vector<QC::Joint> out_joints;
    joint_state_goal_ = interfaces::msg::JointCmd();
    joint_state_goal_.header.stamp = now();

    using State = ReplayState::State;

    switch (replay_state_.state) {
      /////////// Interpolation phase
      case State::kInitPose: {
        if (initial_interpolation_ && !interpolation_started_) {
          lerp_init_position_ = to_vector(quad_state_msg_.joint_state.position);
          interpolation_started_ = true;
        }
        bool all_done = true;
        for (int i = 0; i < NUM_JOINTS; i++) {
          double theta;  // current joint position (only for interpolation)
          if (initial_interpolation_ && !wait_) {
            theta = quad_state_msg_.joint_state.position[i];
            // double error = y[i] - theta;
            // double v_des = std::min(std::max(error, -5.0), 5.0);  // 0.5
            // rad/s joint_state_goal_.position.push_back(theta + v_des * (1.0 /
            // update_freq_)); joint_state_goal_.velocity.push_back(v_des);

            joint_state_goal_.position[i] = naive_lerp(lerp_init_position_[i], y[i], interpolate_time_);
            joint_state_goal_.velocity[i] = 0.0;

          } else {
            joint_state_goal_.position[i] = y[i];
            joint_state_goal_.velocity[i] = 0.;
          }
          joint_state_goal_.effort[i] = 0.;
          joint_state_goal_.kp[i] = kp_init_map_[i];
          joint_state_goal_.kd[i] = kd_init_map_[i];

          // check wether interpolation is done (5 deg difference allowed)
          if (initial_interpolation_ && std::abs(y_vec_initPose[i] - theta) > 5 * DEG2RAD) {
            all_done = false;
          }
        }
        if ((all_done || interpolate_time_ == 1.0f) && !wait_) {
          // go to waiting state with commanded position instead of
          // interpolation
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Use 'ros2 param set /trajectory_replay wait "
                             "false' to resume trajectory.");
          wait_ = true;
        }

        interpolate_time_ = interpolate_time_ + 0.0025;
        interpolate_time_ = std::min(interpolate_time_, 1.0f);
        // wait for parameter change to start trajectory
        // if (all_done &&
        // !this->get_parameter("wait").get_parameter_value().get<bool>()) {
        if (!this->get_parameter("wait").get_parameter_value().get<bool>()) {
          this->set_parameter(rclcpp::Parameter("wait", true));
          wait_ = false;
          interpolate_time_ = 0.0;
          temp_time_ = 0;
          replay_state_.state = State::kExertion;
          break;
        }
        break;
      }

      //////////////// Exertion Phase
      case State::kExertion: {
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_state_goal_.position[i] = y[i];
          joint_state_goal_.velocity[i] = yd[i];
          joint_state_goal_.kp[i] = kp_map_[i];
          joint_state_goal_.kd[i] = kd_map_[i];
          joint_state_goal_.effort[i] = Tau[i];
        }
        temp_time_++;

        if (temp_time_ == yExertion.size()) {
          // go to next state
          temp_time_ = 0;
          replay_state_.state = State::kFlight;
          break;
        }
        break;
      }

      //////// FLIGHT PHASE
      case State::kFlight: {
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_state_goal_.position[i] = y[i];
          joint_state_goal_.velocity[i] = yd[i];
          joint_state_goal_.kp[i] = kp_map_[i];
          joint_state_goal_.kd[i] = kd_map_[i];
          joint_state_goal_.effort[i] = Tau[i];
        }
        temp_time_++;

        if (temp_time_ == yFlight.size()) {
          // go to next state
          temp_time_ = 0;
          replay_state_.state = State::kLand;
          break;
        }
        break;
      }

      /////////////// LANDING PHASE
      case State::kLand: {
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_state_goal_.position[i] = y[i];
          joint_state_goal_.velocity[i] = yd[i];
          joint_state_goal_.kp[i] = kp_map_[i];
          joint_state_goal_.kd[i] = kd_map_[i];
          joint_state_goal_.effort[i] = Tau[i];
        }
        temp_time_++;

        if (temp_time_ == yLand.size()) {
          temp_time_ = 0;
          num_of_times_ += 1;
          if (num_of_times_ < req_num_of_times_) {
            interpolate_time_ = 0.0;
            temp_time_ = repeat_from_timestep_;
            replay_state_.state = State::kExertion;
            break;
          } else {
            replay_state_.state = State::kDone;
            break;
          }
        }
        break;
      }

      case State::kDone: {
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_state_goal_.position[i] = y[i];
          joint_state_goal_.velocity[i] = 0.;
          joint_state_goal_.kp[i] = kp_init_map_[i];
          joint_state_goal_.kd[i] = kd_init_map_[i];
          joint_state_goal_.effort[i] = 0.;
        }
        interpolate_time_ = interpolate_time_ + 0.0025;
        if (interpolate_time_ > 0.5) {
          DoControl_ZeroVelocity();
          break;
        }
        break;
      }
      default: {
        DoControl_ZeroVelocity();
        break;
      }
    }
  }

  void DoControl_Trajectory(Eigen::VectorXd y, Eigen::VectorXd yd, Eigen::VectorXd Tau) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "In DoControl_Trajectory");

    using State = ReplayState::State;

    joint_state_goal_ = interfaces::msg::JointCmd();
    joint_state_goal_.header.stamp = now();

    switch (replay_state_.state) {
      case State::kInitPose: {
        if (initial_interpolation_ && !interpolation_started_) {
          lerp_init_position_ = to_vector(quad_state_msg_.joint_state.position);
          interpolation_started_ = true;
        }
        bool all_done = true;
        for (int i = 0; i < NUM_JOINTS; i++) {
          double theta;  // current joint position (only for interpolation)
          if (initial_interpolation_ && !wait_) {
            theta = quad_state_msg_.joint_state.position[i];
            // double error = y[i] - theta;
            // double v_des = std::min(std::max(error, -5.0), 5.0);  // 0.5
            // rad/s joint_state_goal_.position.push_back(theta + v_des * (1.0 /
            // update_freq_)); joint_state_goal_.velocity.push_back(v_des);

            joint_state_goal_.position[i] = naive_lerp(lerp_init_position_[i], y[i], interpolate_time_);
            joint_state_goal_.velocity[i] = 0.0;

          } else {
            joint_state_goal_.position[i] = y[i];
            joint_state_goal_.velocity[i] = 0.;
          }
          joint_state_goal_.effort[i] = 0.;
          joint_state_goal_.kp[i] = kp_init_map_[i];
          joint_state_goal_.kd[i] = kd_init_map_[i];

          // check wether interpolation is done (5 deg difference allowed)
          if (initial_interpolation_ && std::abs(y_vec_initPose[i] - theta) > 5 * DEG2RAD) {
            all_done = false;
          }
        }
        if ((all_done || interpolate_time_ == 1.0f) && !wait_) {
          // go to waiting state with commanded position instead of
          // interpolation
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Use 'ros2 param set /trajectory_replay wait "
                             "false' to resume trajectory.");
          wait_ = true;
        }

        interpolate_time_ = interpolate_time_ + 0.0025;
        interpolate_time_ = std::min(interpolate_time_, 1.0f);
        // if (all_done &&
        // !this->get_parameter("wait").get_parameter_value().get<bool>()) {
        if (!this->get_parameter("wait").get_parameter_value().get<bool>()) {
          this->set_parameter(rclcpp::Parameter("wait", true));
          wait_ = false;
          interpolate_time_ = 0.0;
          temp_time_ = 0;
          replay_state_.state = State::kTrajectory;
          break;
        }
        break;
      }
      case State::kExertion:
      case State::kFlight:
      case State::kLand:
      case State::kTrajectory: {
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_state_goal_.position[i] = y[i];
          joint_state_goal_.velocity[i] = yd[i];
          joint_state_goal_.kp[i] = kp_map_[i];
          joint_state_goal_.kd[i] = kd_map_[i];
          joint_state_goal_.effort[i] = Tau[i];
        }
        temp_time_++;
        if (temp_time_ == y_vec.size()) {
          num_of_times_ += 1;
          if (num_of_times_ < req_num_of_times_) {
            // publish end of traj for plotting
            if (publish_state_) {
              traj_state_msg_.header.stamp = now();
              traj_state_msg_.state = (uint8_t)State::kDone;
              state_pub_->publish(traj_state_msg_);
            }
            temp_time_ = repeat_from_timestep_;
            replay_state_.state = State::kTrajectory;
          } else {
            replay_state_.state = State::kDone;
          }
        }
        break;
      }
      case State::kDone: {
        temp_time_ = 0;
        DoControl_ZeroVelocity();
        break;
      }
    }
  }

  void DoControl_CartesianTrajectory(Eigen::Matrix<double, 4, 3> x,
                                     Eigen::Matrix<double, 4, 3> v,
                                     Eigen::Matrix<double, 4, 3> F) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "In DoControl_Trajectory");

    using State = ReplayState::State;

    leg_state_goal_ = interfaces::msg::LegCmd();
    leg_state_goal_.header.stamp = now();

    if (correct_xy_) {
      for (unsigned int i = 0; i < NUM_LEGS; i++) {
        for (unsigned int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
          x(i, j) += xy_correction_[i](j);
        }
      }
    }

    switch (replay_state_.state) {
      case State::kInitPose: {
        if (initial_interpolation_ && !interpolation_started_) {
          // save init position for lerp
          lerp_init_position_.clear();
          for (int i = 0; i < NUM_LEGS; i++) {
            Eigen::Vector3d x_e;
            quad_model_.calcBodyToFootFKBodyFrame(
                i,
                Eigen::Map<const Eigen::Vector3d>(quad_state_.GetJointPositions()[i].data()),
                quad_state_.GetOrientationInWorld(),
                x_e);
            for (int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
              lerp_init_position_.push_back(x_e(j));
            }
          }
          interpolation_started_ = true;
        }
        bool all_done = true;
        for (int i = 0; i < NUM_LEGS; i++) {
          Eigen::Vector3d x_e;
          Eigen::Vector3d frame_correction;
          geometry_msgs::msg::Point pos;
          geometry_msgs::msg::Vector3 vel;
          geometry_msgs::msg::Vector3 force;

          if (initial_interpolation_ && !wait_) {
            quad_model_.calcBodyToFootFKBodyFrame(
                i,
                Eigen::Map<const Eigen::Vector3d>(quad_state_.GetJointPositions()[i].data()),
                quad_state_.GetOrientationInWorld(),
                x_e);
            pos.x = naive_lerp(lerp_init_position_[i * NUM_JOINTS_PER_LEG + 0], x(i, 0), interpolate_time_);
            pos.y = naive_lerp(lerp_init_position_[i * NUM_JOINTS_PER_LEG + 1], x(i, 1), interpolate_time_);
            pos.z = naive_lerp(lerp_init_position_[i * NUM_JOINTS_PER_LEG + 2], x(i, 2), interpolate_time_);
            //  vel and force is already 0
          } else {
            pos.x = x(i, 0);
            pos.y = x(i, 1);
            pos.z = x(i, 2);
            //  vel and force is already 0
          }
          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 0] = pos.x;
          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 1] = pos.y;
          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 2] = pos.z;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 0] = vel.x;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 1] = vel.y;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 2] = vel.z;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 0] = force.x;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 1] = force.y;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 2] = force.z;
          for (int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
            if (cartesian_stiffness_) {
              leg_state_goal_.kp[i * NUM_JOINTS_PER_LEG + j] = cartesian_kp_init_map_[i * NUM_JOINTS_PER_LEG + j];
              leg_state_goal_.kd[i * NUM_JOINTS_PER_LEG + j] = cartesian_kd_init_map_[i * NUM_JOINTS_PER_LEG + j];
            } else {
              leg_state_goal_.kp[i * NUM_JOINTS_PER_LEG + j] = kp_init_map_[i * NUM_JOINTS_PER_LEG + j];
              leg_state_goal_.kd[i * NUM_JOINTS_PER_LEG + j] = kd_init_map_[i * NUM_JOINTS_PER_LEG + j];
            }
          }

          // check wether interpolation is done (5 deg difference allowed)
          if (initial_interpolation_
              && (std::abs(x_vec_initPose(i, 0) - x_e[0]) > 0.02 || std::abs(x_vec_initPose(i, 1) - x_e[1]) > 0.02
                  || std::abs(x_vec_initPose(i, 2) - x_e[2]) > 0.02)) {
            all_done = false;
          }
        }
        if ((all_done || interpolate_time_ == 1.0f) && !wait_) {
          // go to waiting state with commanded position instead of
          // interpolation
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Use 'ros2 param set /trajectory_replay wait "
                             "false' to resume trajectory.");
          wait_ = true;
        }

        interpolate_time_ = interpolate_time_ + 0.0025;
        interpolate_time_ = std::min(interpolate_time_, 1.0f);
        // if (all_done &&
        // !this->get_parameter("wait").get_parameter_value().get<bool>()) {
        if (!this->get_parameter("wait").get_parameter_value().get<bool>()) {
          this->set_parameter(rclcpp::Parameter("wait", true));
          wait_ = false;
          interpolate_time_ = 0.0;
          temp_time_ = 0;
          replay_state_.state = State::kTrajectory;
          break;
        }
        break;
      }
      case State::kExertion:
      case State::kFlight:
      case State::kLand:
      case State::kTrajectory: {
        for (int i = 0; i < NUM_LEGS; i++) {
          geometry_msgs::msg::Point pos;
          geometry_msgs::msg::Vector3 vel;
          geometry_msgs::msg::Vector3 force;
          pos.x = x(i, 0);
          pos.y = x(i, 1);
          pos.z = x(i, 2);
          vel.x = v(i, 0);
          vel.y = v(i, 1);
          vel.z = v(i, 2);
          force.x = F(i, 0);
          force.y = F(i, 1);
          force.z = F(i, 2);

          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 0] = pos.x;
          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 1] = pos.y;
          leg_state_goal_.ee_pos[i * ModelInterface::N_JOINTS_PER_LEG + 2] = pos.z;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 0] = vel.x;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 1] = vel.y;
          leg_state_goal_.ee_vel[i * ModelInterface::N_JOINTS_PER_LEG + 2] = vel.z;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 0] = force.x;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 1] = force.y;
          leg_state_goal_.ee_force[i * ModelInterface::N_JOINTS_PER_LEG + 2] = force.z;
          for (int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
            if (cartesian_stiffness_) {
              leg_state_goal_.kp[i * NUM_JOINTS_PER_LEG + j] = cartesian_kp_init_map_[i * NUM_JOINTS_PER_LEG + j];
              leg_state_goal_.kd[i * NUM_JOINTS_PER_LEG + j] = cartesian_kd_init_map_[i * NUM_JOINTS_PER_LEG + j];
            } else {
              leg_state_goal_.kp[i * NUM_JOINTS_PER_LEG + j] = kp_map_[i * NUM_JOINTS_PER_LEG + j];
              leg_state_goal_.kd[i * NUM_JOINTS_PER_LEG + j] = kd_map_[i * NUM_JOINTS_PER_LEG + j];
            }
          }
        }
        temp_time_++;
        if (temp_time_ == y_vec.size()) {
          num_of_times_++;
          if (num_of_times_ < req_num_of_times_) {
            // publish end of traj for plotting
            if (publish_state_) {
              traj_state_msg_.header.stamp = now();
              traj_state_msg_.state = (uint8_t)State::kDone;
              state_pub_->publish(traj_state_msg_);
            }
            temp_time_ = repeat_from_timestep_;
            replay_state_.state = State::kTrajectory;
          } else {
            replay_state_.state = State::kDone;
          }
        }
        break;
      }
      case State::kDone: {
        temp_time_ = 0;
        DoControl_CartesianZeroVelocity();
        break;
      }
    }
  }

  void quad_state_callback(const std::shared_ptr<interfaces::msg::QuadState> msg) {
    quad_state_msg_ = *msg;
    if (quad_state_msg_.joint_state.position.size() > 0) {
      quad_state_received_ = true;
      quad_state_.UpdateFromMsg(
          quad_state_msg_);  // TODO: This is shit -> change the quad_state_msg_ users to also use quad_state
    } else {
      quad_state_received_ = false;
    }
  }

  void calc_joint_states_goal_() {
    // if (this->get_parameter("enabled").get_parameter_value().get<bool>() &&
    //     quad_state_received_)
    if (!initial_interpolation_ || quad_state_received_)

    {
      if (!state_machine_) {  // just trajectory replay
        DoControl_Trajectory(y_vec[temp_time_], yd_vec[temp_time_], Tau_vec[temp_time_]);
      } else {  // state machine
        switch (replay_state_.state) {
          case ReplayState::State::kExertion: {
            DoControl_Trajectory_state_machine(yExertion[temp_time_], ydExertion[temp_time_], TauExertion[temp_time_]);
            break;
          }
          case ReplayState::State::kFlight: {
            DoControl_Trajectory_state_machine(yFlight[temp_time_], ydFlight[temp_time_], TauFlight[temp_time_]);
            break;
          }
          case ReplayState::State::kLand: {
            DoControl_Trajectory_state_machine(yLand[temp_time_], ydLand[temp_time_], TauLand[temp_time_]);
            break;
          }
          default:
            DoControl_Trajectory_state_machine(y_vec[0], yd_vec[0], Tau_vec[0]);
            break;
        }
      }

      //  publish
      joint_cmd_pub_->publish(joint_state_goal_);
      if (publish_state_) {
        traj_state_msg_.header.stamp = now();
        traj_state_msg_.state = (uint8_t)replay_state_.state;
        state_pub_->publish(traj_state_msg_);
      }
    }
  }

  void calc_leg_states_goal_() {
    if (!initial_interpolation_ || quad_state_received_)

    {
      DoControl_CartesianTrajectory(x_vec[temp_time_], v_vec[temp_time_], F_vec[temp_time_]);
      //  publish
      leg_cmd_pub_->publish(leg_state_goal_);
      if (publish_state_) {
        traj_state_msg_.header.stamp = now();
        traj_state_msg_.state = (uint8_t)replay_state_.state;
        state_pub_->publish(traj_state_msg_);
      }
    }
  }

  float naive_lerp(float a, float b, float t) { return a + t * (b - a); }

 private:
  // -- subscriptions --
  rclcpp::Subscription<interfaces::msg::QuadState>::SharedPtr quad_state_sub_;

  // -- publishers --
  rclcpp::Publisher<interfaces::msg::JointCmd>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<interfaces::msg::LegCmd>::SharedPtr leg_cmd_pub_;
  rclcpp::Publisher<interfaces::msg::TrajectoryStatus>::SharedPtr state_pub_;

  // -- timers --
  rclcpp::TimerBase::SharedPtr replay_timer_;

  // messages
  interfaces::msg::JointCmd joint_state_goal_;
  interfaces::msg::LegCmd leg_state_goal_;
  interfaces::msg::QuadState quad_state_msg_;
  interfaces::msg::TrajectoryStatus traj_state_msg_;

  // config parameters
  float update_freq_;
  std::string csv_path_;
  bool state_machine_;
  bool initial_interpolation_;
  float kd_damping_;
  bool publish_state_;
  bool correct_xy_;

  float kp_abad_init_;
  float kp_hip_init_;
  float kp_knee_init_;
  float kp_abad_;
  float kp_hip_front_;
  float kp_knee_front_;
  float kp_hip_back_;
  float kp_knee_back_;

  float kd_abad_init_;
  float kd_hip_init_;
  float kd_knee_init_;
  float kd_abad_;
  float kd_hip_front_;
  float kd_knee_front_;
  float kd_hip_back_;
  float kd_knee_back_;

  float kp_x_front_init_;
  float kp_y_front_init_;
  float kp_z_front_init_;
  float kp_x_back_init_;
  float kp_y_back_init_;
  float kp_z_back_init_;
  float kp_x_front_trajectory_;
  float kp_y_front_trajectory_;
  float kp_z_front_trajectory_;
  float kp_x_back_trajectory_;
  float kp_y_back_trajectory_;
  float kp_z_back_trajectory_;

  float kd_x_front_init_;
  float kd_y_front_init_;
  float kd_z_front_init_;
  float kd_x_back_init_;
  float kd_y_back_init_;
  float kd_z_back_init_;
  float kd_x_front_trajectory_;
  float kd_y_front_trajectory_;
  float kd_z_front_trajectory_;
  float kd_x_back_trajectory_;
  float kd_y_back_trajectory_;
  float kd_z_back_trajectory_;

  // params
  ReplayState replay_state_;

  std::map<int, double> kp_map_;
  std::map<int, double> kd_map_;

  std::map<int, double> kp_init_map_;
  std::map<int, double> kd_init_map_;

  std::map<int, double> cartesian_kp_map_;
  std::map<int, double> cartesian_kd_map_;

  std::map<int, double> cartesian_kp_init_map_;
  std::map<int, double> cartesian_kd_init_map_;

  QuadModelSymbolic quad_model_;
  QuadState quad_state_;
  std::array<std::string, 4> leg_names_ = {"fl", "fr", "bl", "br"};

  double time_trajectory_;
  float interpolate_time_;
  double temp_time_;
  int num_of_times_ = 0;
  int req_num_of_times_;
  int repeat_from_timestep_ = 0;
  bool quad_state_received_;
  bool interpolation_started_ = false;
  std::vector<double> lerp_init_position_;
  bool cartesian_ = false;
  bool cartesian_stiffness_ = true;

  bool wait_ = false;

  // traj vectors
  Eigen::VectorXd y_vec_initPose;
  std::vector<Eigen::VectorXd> y_vec;
  std::vector<Eigen::VectorXd> yd_vec;
  std::vector<Eigen::VectorXd> Tau_vec;
  std::vector<int> contactPhases_vec;

  std::vector<Eigen::VectorXd> yExertion;
  std::vector<Eigen::VectorXd> ydExertion;
  std::vector<Eigen::VectorXd> TauExertion;
  std::vector<int> contactExertion;

  std::vector<Eigen::VectorXd> yFlight;
  std::vector<Eigen::VectorXd> ydFlight;
  std::vector<Eigen::VectorXd> TauFlight;
  std::vector<int> contactFlight;

  std::vector<Eigen::VectorXd> yLand;
  std::vector<Eigen::VectorXd> ydLand;
  std::vector<Eigen::VectorXd> TauLand;
  std::vector<int> contactLand;

  // for cartesian trajectories
  Eigen::Matrix<double, 4, 3> x_vec_initPose;
  std::vector<Eigen::Matrix<double, 4, 3>> x_vec;
  std::vector<Eigen::Matrix<double, 4, 3>> v_vec;
  std::vector<Eigen::Matrix<double, 4, 3>> F_vec;

  std::array<Eigen::Vector3d, 4> xy_correction_ = {
      Eigen::Vector3d(0.167, 0.1538, 0.0),
      Eigen::Vector3d(0.167, -0.1538, 0.0),
      Eigen::Vector3d(-0.197, 0.1538, 0.0),
      Eigen::Vector3d(-0.197, -0.1538, 0.0),
  };
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<TrajectoryReplay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

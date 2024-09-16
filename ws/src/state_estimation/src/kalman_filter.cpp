#include "kalman_filter.hpp"

#include <iostream>
#include <utility>

KalmanFilter::KalmanFilter(const std::shared_ptr<const ModelInterface> &quadModel,
                           const Eigen::Vector3d &initial_robot_pose,
                           const Eigen::Quaterniond &initial_orientation,
                           Params params)
    : quad_model_(quadModel),
      first_imu_msg_received_(false),
      contact_forces_prev_(0, 0, 0, 0),
      foot_step_measurement_std_(0),
      params_(std::move(params)) {
  foot_step_measurement_std_ = params_.foot_step_measurement_std_;
  // Set noise params
  inekf::NoiseParams noise_params;
  noise_params.setGyroscopeNoise(params_.gyroscope_std_);
  noise_params.setAccelerometerNoise(params_.accelerometer_std_);
  noise_params.setGyroscopeBiasNoise(params_.gyroscope_bias_std_);
  noise_params.setAccelerometerBiasNoise(params_.accelerometer_bias_std_);
  noise_params.setContactNoise(params_.contact_std_);
  filter_.setNoiseParams(noise_params);

  // Set initial state
  Eigen::Matrix<double, 15, 15> P_init = Eigen::Matrix<double, 15, 15>::Zero();
  P_init.block<3, 3>(0, 0).diagonal().setConstant(pow(params_.prior_base_orientation_std_, 2));
  P_init.block<3, 3>(3, 3).diagonal().setConstant(pow(params_.prior_base_velocity_std_, 2));
  P_init.block<3, 3>(6, 6).diagonal().setConstant(pow(params_.prior_base_position_std_, 2));
  P_init.block<3, 3>(9, 9).diagonal().setConstant(pow(params_.prior_gyroscope_bias_std_, 2));
  P_init.block<3, 3>(12, 12).diagonal().setConstant(pow(params_.prior_accelerometer_bias_std_, 2));
  init_state_.setRotation(initial_orientation.toRotationMatrix());
  init_state_.setVelocity(Eigen::Vector3d::Zero());                     // initial velocity assumed to be zero
  init_state_.setPosition(initial_robot_pose);                          // initial position is at x/y=0, but a
                                                                        // defined height
  init_state_.setGyroscopeBias(params_.prior_gyroscope_bias_);          // init gyroscope bias
  init_state_.setAccelerometerBias(params_.prior_accelerometer_bias_);  // init accelerometer bias
  init_state_.setP(P_init);
  filter_.setState(init_state_);
}

void KalmanFilter::Predict(const sensor_msgs::msg::Imu &imu_msg) {
  if (first_imu_msg_received_) {
    auto current_t =
        std::chrono::seconds(imu_msg.header.stamp.sec)
        + std::chrono::duration<typeof(imu_msg.header.stamp.nanosec), std::nano>(imu_msg.header.stamp.nanosec);
    auto last_t = std::chrono::seconds(last_imu_msg_.header.stamp.sec)
                  + std::chrono::duration<typeof(last_imu_msg_.header.stamp.nanosec), std::nano>(
                      last_imu_msg_.header.stamp.nanosec);
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_t - last_t).count();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d{
        last_imu_msg_.linear_acceleration.x, last_imu_msg_.linear_acceleration.y, last_imu_msg_.linear_acceleration.z};

    if (!params_.imu_measures_g_) {
      linear_acceleration += filter_.getState().getRotation().inverse() * Eigen::Vector3d{0., 0., 9.81};
    }

    filter_.Propagate({last_imu_msg_.angular_velocity.x,
                       last_imu_msg_.angular_velocity.y,
                       last_imu_msg_.angular_velocity.z,
                       linear_acceleration.x(),
                       linear_acceleration.y(),
                       linear_acceleration.z()},
                      dt);
  } else {
    first_imu_msg_received_ = true;
    std::cout << "First IMU msg received, filter starting kalman filter predictions" << std::endl;
  }
  last_imu_msg_ = imu_msg;
}

void KalmanFilter::Update(const SequenceView<bool, ModelInterface::N_LEGS> &foot_contacts,
                          const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &foot_positions,
                          bool belly_contact,
                          const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &belly_contact_point) {
  static std::vector<std::pair<int, bool>> foot_contacts_pairs(ModelInterface::N_LEGS * 2);
  static inekf::vectorKinematics measured_kinematics;
  measured_kinematics.clear();
  measured_kinematics.reserve(ModelInterface::N_LEGS * 2);

  for (int i = 0; i < ModelInterface::N_LEGS; i++) {
    foot_contacts_pairs[i] = {i, foot_contacts[i]};
    measured_kinematics.push_back(
        inekf::Kinematics(i,
                          Eigen::Affine3d(Eigen::Translation3d(foot_positions[i])).matrix(),
                          Eigen::Matrix<double, 6, 6>::Identity() * pow(foot_step_measurement_std_, 2)));
  }

  for (int i = 0; i < ModelInterface::N_LEGS; i++) {
    foot_contacts_pairs[ModelInterface::N_LEGS + i] = {ModelInterface::N_LEGS + i, belly_contact};
    measured_kinematics.push_back(inekf::Kinematics(
        ModelInterface::N_LEGS + i,
        Eigen::Affine3d(Eigen::Translation3d(belly_contact_point[i])).matrix(),
        Eigen::Matrix<double, 6, 6>::Identity() * pow(params_.belly_contact_point_measurement_std_, 2)));
  }

  filter_.setContacts(foot_contacts_pairs);
  filter_.CorrectKinematics(measured_kinematics);
  if (params_.planar_ground_) {
    for (int i = 0; i < ModelInterface::N_LEGS; i++) {
      if (foot_contacts[i]) {
        filter_.CorrectContactPosition(
            i,
            {0.0, 0.0, 0.0},
            Eigen::Matrix3d::Identity() * params_.foot_on_plane_std_ * params_.foot_on_plane_std_,
            {0, 0, 1});
      }
    }
  }
  if (belly_contact) {
    for (int i = 0; i < ModelInterface::N_LEGS; i++) {
      filter_.CorrectContactPosition(
          ModelInterface::N_LEGS + i,
          {0.0, 0.0, 0.0},
          Eigen::Matrix3d::Identity() * params_.foot_on_plane_std_ * params_.foot_on_plane_std_,
          {0, 0, 1});
    }
  }
}

void KalmanFilter::GetState(Eigen::Vector3d &position,
                            Eigen::Quaterniond &orientation,
                            Eigen::Vector3d &linear_velocity) const {
  auto state = filter_.getState();
  position = state.getPosition();
  orientation = state.getRotation();
  linear_velocity = state.getVelocity();
}

void KalmanFilter::GetOrientation(Eigen::Quaterniond &orientation) const {
  orientation = filter_.getState().getRotation();
}

void KalmanFilter::GetCovariances(Eigen::Matrix<double, 6, 6> &pose_covariance,
                                  Eigen::Matrix3d &linear_velocity_covariance) {
  auto state = filter_.getState();
  // Needs to be rearanged, as ROS2 messsage specifies this standard TODO: ugly that we hav now a ROS coupling here
  pose_covariance.setZero();
  pose_covariance.block<3, 3>(0, 0) = state.getP().block<3, 3>(6, 6);
  pose_covariance.block<3, 3>(3, 3) = state.getP().block<3, 3>(0, 0);
  linear_velocity_covariance.setZero();
  linear_velocity_covariance = state.getP().block<3, 3>(3, 3);
}

// TODO: Covariance adaptation for Kalman filter
void KalmanFilter::AdaptCovariances(
    const SequenceView<bool, ModelInterface::N_LEGS> &foot_contacts,
    const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &foot_positions,
    const std::array<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> &contact_forces_map) {
  bool verbose = true;
  if (verbose) std::cout << std::endl << "\033[1;34mfoot_contacts \033[0m " << std::endl;

  // Initialize temporary variables
  double sum_of_contact_forces = 0.0, no_foot_contacts_curr = 0.0, no_foot_contacts_curr_and_prev = 0.0;
  Eigen::Vector3d sum_of_foot_position_diff(0, 0, 0), stddev_foot_positions(0, 0, 0);
  std::vector<Eigen::Vector3d> abs_foot_position_diff;

  // Get the transformation from world to IMU frame from current filter state
  Eigen::Matrix4d transf_world_to_IMU;
  transf_world_to_IMU.setIdentity();
  transf_world_to_IMU.block<3, 3>(0, 0) = filter_.getState().getRotation();
  transf_world_to_IMU.block<3, 1>(0, 3) = filter_.getState().getPosition();

  // Iterate over the foot forces and positions and sum them up
  for (int i = 0; i < ModelInterface::N_LEGS; i++) {
    if (verbose) std::cout << "foot " << i << ", contact: " << foot_contacts[i] << std::endl;

    // Check if foot is currently in contact
    if (foot_contacts[i]) {
      // Sum up the absolute difference between two consecutive foot force vectors (z component) for the feet in contact
      sum_of_contact_forces += std::abs(contact_forces_map[i][2] - contact_forces_prev_[i]);

      // Check if foot was previously in contact (if foot position is part of the state)
      if (filter_.getContacts().find(i)->second) {
        // Transform previous foot position (obtained from filter state) from world to IMU coordinate frame
        Eigen::Vector4d prev_foot_positions_world_frame, prev_foot_positions_IMU_frame;
        prev_foot_positions_world_frame << filter_.getState().getVector(
            filter_.getEstimatedContactPositions().find(i)->second),
            1.0;
        prev_foot_positions_IMU_frame = transf_world_to_IMU.inverse() * prev_foot_positions_world_frame;

        // Sum up the absolute difference between current foot position (measured from kinematics) and previous foot
        // position (from filter state) in IMU coordinate frame
        abs_foot_position_diff.push_back((foot_positions[i] - prev_foot_positions_IMU_frame.head(3)).cwiseAbs());
        sum_of_foot_position_diff += abs_foot_position_diff.back();
      }
      no_foot_contacts_curr++;
    }
  }

  // Average the z foot force difference for the legs in contact between two consecutive time steps
  double delta_f_z = sum_of_contact_forces / no_foot_contacts_curr;

  // Calculate standard deviation of the foot position differences
  no_foot_contacts_curr_and_prev = abs_foot_position_diff.size();
  Eigen::Vector3d mean_foot_positions = sum_of_foot_position_diff / no_foot_contacts_curr_and_prev;
  while (!abs_foot_position_diff.empty()) {
    stddev_foot_positions += Eigen::pow((abs_foot_position_diff.back() - mean_foot_positions).array(), 2).matrix();
    abs_foot_position_diff.pop_back();
  }
  stddev_foot_positions = (Eigen::sqrt((stddev_foot_positions / no_foot_contacts_curr_and_prev).array())).matrix();

  // TODO: Some parameters to tune
  double sigma_0 = params_.foot_step_measurement_std_;  // covariance of leg updates
  double alpha_1 = 0.5;
  double alpha_2 = sigma_0;  // alpha_2(1e-5, 1e-5, 1e-4)

  // Adapt the covariance R for the foot position measurement (leg kinematics)
  double sigma_r_adapted = std::sqrt(
      std::pow(sigma_0, 2) + std::pow(alpha_1 * stddev_foot_positions.mean() + (1 - alpha_1) * alpha_2 * delta_f_z, 2));

  // If adaptive covariance is nan, keep the default covariance
  if (std::isnan(sigma_r_adapted)) {
    sigma_r_adapted = sigma_0;
  }

  // Pass the adapted covariance to the filter
  foot_step_measurement_std_ = sigma_r_adapted; 

  if (verbose) std::cout << "\033[1mcontact_forces_prev_ \033[0m " << std::endl << contact_forces_prev_ << std::endl;
  if (verbose) std::cout << "\033[1mdelta_f_z \033[0m " << std::endl << delta_f_z << std::endl;
  if (verbose)
    std::cout << "\033[1mstddev_foot_positions \033[0m " << std::endl << stddev_foot_positions << std::endl;
  if (verbose) std::cout << "\033[1mfoot_step_measurement_std_ \033[0m " << std::endl << sigma_0 << std::endl;
  if (verbose) std::cout << "\033[1msigma_r_adapted\033[0m " << std::endl << sigma_r_adapted << std::endl;

  // Save current contact forces as previous contact forces
  contact_forces_prev_ << contact_forces_map[0][2], contact_forces_map[1][2], contact_forces_map[2][2],
      contact_forces_map[3][2];
}

void KalmanFilter::Reset() {
  filter_.clear();
  filter_.setState(init_state_);
}

void KalmanFilter::ResetCovariances() {
  std::cout << "Reseting covariances and bias to pior values" << std::endl;
  auto current_state = filter_.getState();
  Eigen::MatrixXd P_new;
  P_new.resize(current_state.dimP(), current_state.dimP());
  P_new.block<15, 15>(0, 0) = init_state_.getP();
  P_new.block(14, 14, current_state.dimP() - 15, current_state.dimP() - 15)
      .diagonal()
      .fill(pow(params_.foot_step_measurement_std_, 2));
  current_state.setTheta(init_state_.getTheta());
}

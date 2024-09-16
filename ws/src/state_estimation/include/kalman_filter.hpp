#pragma once

#include <InEKF.h>

#include <common/model_interface.hpp>
#include <common/sequence_containers.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define INEKF_USE_MUTEX false

#define DT_MIN 1e-6
#define DT_MAX 1

class KalmanFilter {
 private:
  std::shared_ptr<const ModelInterface> quad_model_;
  inekf::InEKF filter_;
  inekf::RobotState init_state_;
  sensor_msgs::msg::Imu last_imu_msg_;
  bool first_imu_msg_received_;
  Eigen::Vector4d contact_forces_prev_;
  double foot_step_measurement_std_;

 public:
  const struct Params {
    // From ROS Node
    Eigen::Vector3d gyroscope_std_;
    Eigen::Vector3d accelerometer_std_;
    double gyroscope_bias_std_;
    double accelerometer_bias_std_;
    double contact_std_;
    double foot_on_plane_std_;

    Eigen::Vector3d prior_gyroscope_bias_;
    Eigen::Vector3d prior_accelerometer_bias_;

    double prior_base_orientation_std_;
    double prior_base_velocity_std_;
    double prior_base_position_std_;
    double prior_gyroscope_bias_std_;
    double prior_accelerometer_bias_std_;

    double foot_step_measurement_std_;
    double belly_contact_point_measurement_std_;

    bool imu_measures_g_;
    bool planar_ground_;
    bool adapt_covariances_;

  } params_;

  explicit KalmanFilter(const std::shared_ptr<const ModelInterface> &quadModel,
                        const Eigen::Vector3d &init_robot_pose,
                        const Eigen::Quaterniond &initial_orientation,
                        Params params);

  void Predict(const sensor_msgs::msg::Imu &imu_msg);
  void Update(const SequenceView<bool, ModelInterface::N_LEGS> &foot_contacts,
              const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &foot_positions,
              bool belly_contact,
              const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &belly_contact_point);
  void GetState(Eigen::Vector3d &position, Eigen::Quaterniond &orientation, Eigen::Vector3d &linear_velocity) const;
  void GetOrientation(Eigen::Quaterniond &orientation) const;
  void GetCovariances(Eigen::Matrix<double, 6, 6> &pose_covariance, Eigen::Matrix3d &linear_velocity_covariance);
  void AdaptCovariances(const SequenceView<bool, ModelInterface::N_LEGS> &foot_contacts,
                        const SequenceView<const Eigen::Vector3d, ModelInterface::N_LEGS> &foot_positions,
                        const std::array<Eigen::Ref<Eigen::Vector3d>, ModelInterface::N_LEGS> &contact_forces_map);
  void Reset();
  void ResetCovariances();
};

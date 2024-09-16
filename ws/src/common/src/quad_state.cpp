#include "quad_state.hpp"

const Eigen::Vector3d &QuadState::GetPositionInWorld() const { return position_; }

const Eigen::Quaterniond &QuadState::GetOrientationInWorld() const { return orientation_; }

const Eigen::Vector3d &QuadState::GetLinearVelInWorld() const { return linear_velocity_; }

const Eigen::Vector3d &QuadState::GetAngularVelInWorld() const { return angular_velocity_; }

const Eigen::Vector3d &QuadState::GetLinearAccInWorld() const { return linear_acceleration_; }

const Eigen::Vector3d &QuadState::GetAngularAccInWorld() const { return angular_acceleration_; }

const StateInterface::TimePoint &QuadState::GetTime() const { return time_point_; }

const std::array<bool, StateInterface::NUM_FEET> &QuadState::GetFeetContacts() const { return feet_contacts_; }

const std::array<Eigen::Vector3d, StateInterface::NUM_FEET> &QuadState::GetContactForces() const {
  return contact_forces_;
}

const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> &
QuadState::GetJointPositions() const {
  return joint_positions_;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> &
QuadState::GetJointVelocities() const {
  return joint_velocities_;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> &
QuadState::GetJointAccelerations() const {
  return joint_accelerations_;
}
const std::array<std::array<double, StateInterface::NUM_JOINT_PER_FOOT>, StateInterface::NUM_FEET> &
QuadState::GetJointTorques() const {
  return joint_torques_;
}

StateInterface &QuadState::operator=(const StateInterface &other) {
  return QuadState::operator=(dynamic_cast<const QuadState &>(other));
}

void QuadState::UpdateFromMsg(const interfaces::msg::QuadState &quad_state_msg) {
  position_.x() = quad_state_msg.pose.pose.position.x;
  position_.y() = quad_state_msg.pose.pose.position.y;
  position_.z() = quad_state_msg.pose.pose.position.z;
  orientation_.w() = quad_state_msg.pose.pose.orientation.w;
  orientation_.x() = quad_state_msg.pose.pose.orientation.x;
  orientation_.y() = quad_state_msg.pose.pose.orientation.y;
  orientation_.z() = quad_state_msg.pose.pose.orientation.z;
  linear_velocity_.x() = quad_state_msg.twist.twist.linear.x;
  linear_velocity_.y() = quad_state_msg.twist.twist.linear.y;
  linear_velocity_.z() = quad_state_msg.twist.twist.linear.z;
  angular_velocity_.x() = quad_state_msg.twist.twist.angular.x;
  angular_velocity_.y() = quad_state_msg.twist.twist.angular.y;
  angular_velocity_.z() = quad_state_msg.twist.twist.angular.z;
  linear_acceleration_.x() = quad_state_msg.acceleration.linear.x;
  linear_acceleration_.y() = quad_state_msg.acceleration.linear.y;
  linear_acceleration_.z() = quad_state_msg.acceleration.linear.z;
  angular_acceleration_.x() = quad_state_msg.acceleration.angular.x;
  angular_acceleration_.y() = quad_state_msg.acceleration.angular.y;
  angular_acceleration_.z() = quad_state_msg.acceleration.angular.z;
  time_point_ = TimePoint(
      TimePoint::duration(quad_state_msg.header.stamp.nanosec)
      + std::chrono::duration_cast<TimePoint::duration>(std::chrono::seconds(quad_state_msg.header.stamp.sec)));
  assert(quad_state_msg.foot_contact.size() == NUM_FEET);
  assert(quad_state_msg.joint_state.position.size() == (NUM_FEET * NUM_JOINT_PER_FOOT));
  for (unsigned int foot_idx = 0; foot_idx < NUM_FEET; foot_idx++) {
    feet_contacts_[foot_idx] = quad_state_msg.foot_contact[foot_idx];
    contact_forces_[foot_idx].x() = quad_state_msg.ground_contact_force[foot_idx * 3 + 0];
    contact_forces_[foot_idx].y() = quad_state_msg.ground_contact_force[foot_idx * 3 + 1];
    contact_forces_[foot_idx].z() = quad_state_msg.ground_contact_force[foot_idx * 3 + 2];
    for (unsigned int joint_idx = 0; joint_idx < NUM_JOINT_PER_FOOT; joint_idx++) {
      joint_positions_[foot_idx][joint_idx] =
          quad_state_msg.joint_state.position[foot_idx * NUM_JOINT_PER_FOOT + joint_idx];
      joint_velocities_[foot_idx][joint_idx] =
          quad_state_msg.joint_state.velocity[foot_idx * NUM_JOINT_PER_FOOT + joint_idx];
      joint_accelerations_[foot_idx][joint_idx] =
          quad_state_msg.joint_state.acceleration[foot_idx * NUM_JOINT_PER_FOOT + joint_idx];
      joint_torques_[foot_idx][joint_idx] =
          quad_state_msg.joint_state.effort[foot_idx * NUM_JOINT_PER_FOOT + joint_idx];
    }
  }
}

QuadState::QuadState(const interfaces::msg::QuadState &msg) { this->UpdateFromMsg(msg); }

QuadState::QuadState(){};

StateInterface &QuadState::operator=(const interfaces::msg::QuadState &msg) {
  this->UpdateFromMsg(msg);
  return *this;
}

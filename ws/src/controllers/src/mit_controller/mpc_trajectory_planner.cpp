#include "mpc_trajectory_planner.hpp"

#include <common/model_interface.hpp>
#include <common/state_interface.hpp>

#include "common/quaternion_operations.hpp"
#include "gait_sequence.hpp"
#include "mit_controller_params.hpp"
#include "target.hpp"

MPCTrajectoryPlanner::MPCTrajectoryPlanner(double dt,
                                           const StateInterface& quad_state,
                                           const ModelInterface& quad_model,
                                           bool fix_standing_position,
                                           double distance_threshold,
                                           double angular_threshold,
                                           double velocity_threshold)
    : dt_(dt),
      quad_state_(quad_state),
      quad_model_(quad_model),
      fix_standing_position_(fix_standing_position),
      sequence_initialized_(false),
      distance_threshold_(distance_threshold),
      angular_threshold_(angular_threshold),
      velocity_threshold_(velocity_threshold),
      foot_heights_(std::array<double, 4>{}){};

void MPCTrajectoryPlanner::plan_trajectory(GaitSequence& sequence, const Target& target) {
  Eigen::Vector3d position = quad_state_.GetPositionInWorld();
  Eigen::Quaterniond orientation = quad_state_.GetOrientationInWorld();
  Eigen::Vector3d velocity = quad_state_.GetLinearVelInWorld();
  Eigen::Vector3d twist = quad_state_.GetAngularVelInWorld();
  Eigen::Vector3d euler_orientation = quaternion_to_euler(orientation);
  auto contacts = quad_state_.GetFeetContacts();
  for (int leg = 0; leg < N_LEGS; ++leg) {
    if (contacts[leg]) {
      foot_heights_[leg] = quad_model_.GetFootPositionInWorld(leg, quad_state_).z();
    }
  }

  sequence.reference_trajectory_position[0] = position;
  sequence.reference_trajectory_orientation[0] = orientation;
  sequence.reference_trajectory_velocity[0] = velocity;
  sequence.reference_trajectory_twist[0] = twist;

  for (unsigned int i = 1; i < GAIT_SEQUENCE_SIZE; i++) {
    // angular twist
    if (target.active.wx) {
      twist(0) = target.wx;
    }
    if (target.active.wy) {
      twist(1) = target.wy;
    }
    if (target.active.wz) {
      twist(2) = target.wz;
    }

    // update orientation from twist
    // Eigen::Quaterniond delta_w(1.0, 0.5 * dt_ * twist(0), 0.5 * dt_ * twist(1), 0.5 * dt_ * twist(2));
    // orientation *= delta_w;
    // orientation.normalize();
    euler_orientation(0) += dt_ * twist(0);
    euler_orientation(1) += dt_ * twist(1);
    euler_orientation(2) += dt_ * twist(2);

    // full orientation has higher priority than roll, pitch, yaw
    if (target.active.full_orientation) {
      orientation = target.full_orientation;
      // } else if (target.active.roll || target.active.pitch || target.active.yaw) {
    } else {
      // set orientation due to twist as default

      // overwrite orientation (higher priority than angular twist)
      if (target.active.yaw) {
        euler_orientation(2) = target.yaw;
      }
      if (target.active.pitch) {
        euler_orientation(1) = target.pitch;
      }
      if (target.active.roll) {
        euler_orientation(0) = target.roll;
      }
      orientation = euler_to_quaternion<double>(euler_orientation);
    }

    // velocities in hybrid frame
    if (target.active.hybrid_x_dot || target.active.hybrid_y_dot) {
      Eigen::Vector3d hybrid_velocity(Eigen::Vector3d::Zero());
      if (target.active.hybrid_x_dot) {
        hybrid_velocity(0) = target.hybrid_x_dot;
      }
      if (target.active.hybrid_y_dot) {
        hybrid_velocity(1) = target.hybrid_y_dot;
      }
      velocity = orientation * hybrid_velocity;
      // position update with velocity in direction between current and last yaw
      // orientation
      position += yaw_quaternion_from_quaternion<double>(sequence.reference_trajectory_orientation[i - 1])
                      .slerp(0.5, yaw_quaternion_from_quaternion<double>(orientation))
                  * (dt_ * hybrid_velocity);
    } else {
      // velocities in world frame (lower priority than hybrid frame)
      if (target.active.x_dot) {
        velocity(0) = target.x_dot;
        position(0) += dt_ * target.x_dot;
      }
      if (target.active.y_dot) {
        velocity(1) = target.y_dot;
        position(1) += dt_ * target.y_dot;
      }
    }
    // velocity in z direction (same for both frames)
    if (target.active.z_dot) {
      velocity(2) = target.z_dot;
      position(2) += dt_ * dt_ * target.z_dot;
    } else if (target.active.hybrid_z_dot) {
      velocity(2) = target.hybrid_z_dot;
      position(2) += dt_ * velocity(2);
    }

    // positions (higher priority than velocities)
    if (target.active.x) {
      position(0) = target.x;
    }
    if (target.active.y) {
      position(1) = target.y;
    }
    if (target.active.z) {
      double min_foot_height = *std::min_element(foot_heights_.begin(), foot_heights_.end());
      position(2) = target.z + min_foot_height;
    }
    sequence.reference_trajectory_position[i] = position;
    sequence.reference_trajectory_orientation[i] = orientation;
    sequence.reference_trajectory_velocity[i] = velocity;
    sequence.reference_trajectory_twist[i] = twist;
  }
  plan_desired_trajectory(sequence, target);
  set_sequence_mode(sequence, target);
  set_current_target(sequence, target);
}

void MPCTrajectoryPlanner::plan_desired_trajectory(GaitSequence& sequence, const Target& target) {
  if (!fix_standing_position_) {
    sequence.desired_reference_trajectory_position = sequence.reference_trajectory_position;
    sequence.desired_reference_trajectory_orientation = sequence.reference_trajectory_orientation;
    return;
  }
  Eigen::Vector3d position = sequence.reference_trajectory_position[0];
  Eigen::Quaterniond orientation = sequence.reference_trajectory_orientation[0];
  Eigen::Vector3d velocity = sequence.reference_trajectory_velocity[0];
  Eigen::Vector3d twist = sequence.reference_trajectory_twist[0];
  Eigen::Vector3d euler_orientation = quaternion_to_euler(orientation);

  sequence.desired_reference_trajectory_position[0] = position;
  sequence.desired_reference_trajectory_orientation[0] = orientation;

  // set last target position as current target position when no velocity is commanded
  if (sequence_initialized_) {
    if (!(target.active.x_dot && std::abs(target.x_dot) >= std::numeric_limits<double>::epsilon())
        && !(target.active.hybrid_x_dot && std::abs(target.hybrid_x_dot) >= std::numeric_limits<double>::epsilon())
        && !(target.active.y_dot && std::abs(target.y_dot) >= std::numeric_limits<double>::epsilon())
        && !(target.active.hybrid_y_dot && std::abs(target.hybrid_y_dot) >= std::numeric_limits<double>::epsilon())
        && !(target.active.wz && std::abs(target.wz) >= std::numeric_limits<double>::epsilon())
        && (target.active.x_dot || target.active.hybrid_x_dot || target.active.y_dot || target.active.hybrid_y_dot)
        && ((position.segment(0, 2) - sequence.desired_reference_trajectory_position[1].segment(0, 2)).norm()
            < distance_threshold_)
        && (velocity.norm() < velocity_threshold_)) {
      position.x() = sequence.desired_reference_trajectory_position[1][0];
      position.y() = sequence.desired_reference_trajectory_position[1][1];
    }
    if (!(target.active.z_dot && std::abs(target.z_dot) >= std::numeric_limits<double>::epsilon())
        && !(target.active.hybrid_z_dot && std::abs(target.hybrid_z_dot) >= std::numeric_limits<double>::epsilon())
        && (target.active.z_dot || target.active.hybrid_z_dot)) {
      position.z() = sequence.desired_reference_trajectory_position[1][2];
    }

    Eigen::Vector3d old_euler_orientation = quaternion_to_euler(sequence.desired_reference_trajectory_orientation[1]);
    if (target.active.wx && std::abs(target.wx) <= std::numeric_limits<double>::epsilon()) {
      euler_orientation.x() = old_euler_orientation.x();
    }
    if (target.active.wy && std::abs(target.wy) <= std::numeric_limits<double>::epsilon()) {
      euler_orientation.y() = old_euler_orientation.y();
    }
    if (target.active.wz && std::abs(target.wz) <= std::numeric_limits<double>::epsilon()
        && (std::abs(euler_orientation.z() - old_euler_orientation.z()) < angular_threshold_)) {
      euler_orientation.z() = old_euler_orientation.z();
    }
  }

  for (unsigned int i = 1; i < GAIT_SEQUENCE_SIZE; i++) {
    twist = sequence.reference_trajectory_twist[i];

    // update orientation from twist
    euler_orientation(0) += dt_ * twist(0);
    euler_orientation(1) += dt_ * twist(1);
    euler_orientation(2) += dt_ * twist(2);

    // full orientation has higher priority than roll, pitch, yaw
    if (target.active.full_orientation) {
      orientation = target.full_orientation;
      // } else if (target.active.roll || target.active.pitch || target.active.yaw) {
    } else {
      // set orientation due to twist as default

      // overwrite orientation (higher priority than angular twist)
      if (target.active.yaw) {
        euler_orientation(2) = target.yaw;
      }
      if (target.active.pitch) {
        euler_orientation(1) = target.pitch;
      }
      if (target.active.roll) {
        euler_orientation(0) = target.roll;
      }
      orientation = euler_to_quaternion<double>(euler_orientation);
    }

    // velocities in hybrid frame
    velocity = sequence.reference_trajectory_velocity[i];
    if (target.active.hybrid_x_dot || target.active.hybrid_y_dot) {
      Eigen::Vector3d hybrid_velocity(Eigen::Vector3d::Zero());
      if (target.active.hybrid_x_dot) {
        hybrid_velocity(0) = target.hybrid_x_dot;
      }
      if (target.active.hybrid_y_dot) {
        hybrid_velocity(1) = target.hybrid_y_dot;
      }
      // position update with velocity in direction between current and last yaw
      // orientation
      position += yaw_quaternion_from_quaternion<double>(sequence.desired_reference_trajectory_orientation[i - 1])
                      .slerp(0.5, yaw_quaternion_from_quaternion<double>(orientation))
                  * (dt_ * hybrid_velocity);
    } else {
      // velocities in world frame (lower priority than hybrid frame)
      if (target.active.x_dot) {
        position(0) += dt_ * target.x_dot;
      }
      if (target.active.y_dot) {
        position(1) += dt_ * target.y_dot;
      }
    }
    // velocity in z direction (same for both frames)
    if (target.active.z_dot) {
      position(2) += dt_ * dt_ * target.z_dot;
    } else if (target.active.hybrid_z_dot) {
      position(2) += dt_ * velocity(2);
    }

    // positions (higher priority than velocities)
    if (target.active.x) {
      position(0) = target.x;
    }
    if (target.active.y) {
      position(1) = target.y;
    }
    if (target.active.z) {
      double min_foot_height = *std::min_element(foot_heights_.begin(), foot_heights_.end());
      position(2) = target.z + min_foot_height;
    }
    sequence.desired_reference_trajectory_position[i] = position;
    sequence.desired_reference_trajectory_orientation[i] = orientation;
  }
  sequence_initialized_ = true;
}

void MPCTrajectoryPlanner::set_sequence_mode(GaitSequence& sequence, const Target& target) {
  // MOVE if active non zero x/y velocity or z rotational twist
  if ((target.active.x_dot && std::abs(target.x_dot) >= std::numeric_limits<double>::epsilon())
      || (target.active.hybrid_x_dot && std::abs(target.hybrid_x_dot) >= std::numeric_limits<double>::epsilon())
      || (target.active.y_dot && std::abs(target.y_dot) >= std::numeric_limits<double>::epsilon())
      || (target.active.hybrid_y_dot && std::abs(target.hybrid_y_dot) >= std::numeric_limits<double>::epsilon())
      || (target.active.wz && std::abs(target.wz) >= std::numeric_limits<double>::epsilon())) {
    sequence.sequence_mode = GaitSequence::MOVE;
  } else {
    sequence.sequence_mode = GaitSequence::KEEP;
  }
}

void MPCTrajectoryPlanner::set_current_target(GaitSequence& sequence, const Target& target) {
  Eigen::Vector3d position = sequence.reference_trajectory_position[0];
  Eigen::Quaterniond orientation = sequence.reference_trajectory_orientation[0];
  Eigen::Vector3d velocity = sequence.reference_trajectory_velocity[0];
  Eigen::Vector3d twist = sequence.reference_trajectory_twist[0];
  Eigen::Vector3d euler_orientation = quaternion_to_euler(orientation);

  // angular twist
  if (target.active.wx) {
    twist(0) = target.wx;
  }
  if (target.active.wy) {
    twist(1) = target.wy;
  }
  if (target.active.wz) {
    twist(2) = target.wz;
  }

  // full orientation has higher priority than roll, pitch, yaw
  if (target.active.full_orientation) {
    orientation = target.full_orientation;
    // } else if (target.active.roll || target.active.pitch || target.active.yaw) {
  } else {
    // set orientation due to twist as default
    // overwrite orientation (higher priority than angular twist)
    if (target.active.yaw) {
      euler_orientation(2) = target.yaw;
    }
    if (target.active.pitch) {
      euler_orientation(1) = target.pitch;
    }
    if (target.active.roll) {
      euler_orientation(0) = target.roll;
    }
    orientation = euler_to_quaternion<double>(euler_orientation);
  }

  // velocities in hybrid frame
  if (target.active.hybrid_x_dot || target.active.hybrid_y_dot) {
    Eigen::Vector3d hybrid_velocity(Eigen::Vector3d::Zero());
    if (target.active.hybrid_x_dot) {
      hybrid_velocity(0) = target.hybrid_x_dot;
    }
    if (target.active.hybrid_y_dot) {
      hybrid_velocity(1) = target.hybrid_y_dot;
    }
    velocity = orientation * hybrid_velocity;
  } else {
    // velocities in world frame (lower priority than hybrid frame)
    if (target.active.x_dot) {
      velocity(0) = target.x_dot;
    }
    if (target.active.y_dot) {
      velocity(1) = target.y_dot;
    }
  }
  // velocity in z direction (same for both frames)
  if (target.active.z_dot) {
    velocity(2) = target.z_dot;
  } else if (target.active.hybrid_z_dot) {
    velocity(2) = target.hybrid_z_dot;
  }

  // positions (higher priority than velocities)
  if (target.active.x) {
    position(0) = target.x;
  }
  if (target.active.y) {
    position(1) = target.y;
  }
  double min_foot_height = *std::min_element(foot_heights_.begin(), foot_heights_.end());
  if (target.active.z) {
    position(2) = target.z + min_foot_height;
  }
  sequence.target_position = position;
  sequence.target_orientation = orientation;
  sequence.target_velocity = velocity;
  sequence.target_twist = twist;
  sequence.target_height = position(2) - min_foot_height;
  sequence.current_height = sequence.reference_trajectory_position[0].z() - min_foot_height;
}
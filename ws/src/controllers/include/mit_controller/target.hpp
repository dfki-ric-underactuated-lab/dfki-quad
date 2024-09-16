#pragma once

#include <Eigen/Dense>

struct Target {
  double x = 0.0;      // world frame
  double y = 0.0;      // world frame
  double z = 0.0;      // world frame
  double roll = 0.0;   // hybrid frame where z axis is aligned with world and x axis
                       // with body
  double pitch = 0.0;  // hybrid frame where z axis is aligned with world and x axis
                       // with body
  double yaw = 0.0;    // world frame and hybrid frame (same)
  Eigen::Quaterniond full_orientation{1.0, 0.0, 0.0, 0.0};  // world frame
  double x_dot = 0.0;                                       // world frame
  double y_dot = 0.0;                                       // world frame
  double z_dot = 0.0;                                       // world frame
  double hybrid_x_dot = 0.0;                                // hybrid frame
  double hybrid_y_dot = 0.0;                                // hybrid frame
  double hybrid_z_dot = 0.0;                                // hybrid frame
  double wx = 0.0;                                          // hybrid frame where z axis is aligned with world and x
                                                            // axis with body
  double wy = 0.0;                                          // hybrid frame where z axis is aligned with world and x
                                                            // axis with body
  double wz = 0.0;                                          // world frame
  struct activateTarget {
    bool x = false;
    bool y = false;
    bool z = false;
    bool roll = false;
    bool pitch = false;
    bool yaw = false;
    bool full_orientation = false;
    bool x_dot = false;
    bool y_dot = false;
    bool z_dot = false;
    bool hybrid_x_dot = false;
    bool hybrid_y_dot = false;
    bool hybrid_z_dot = false;
    bool wx = false;
    bool wy = false;
    bool wz = false;
  } active;
};

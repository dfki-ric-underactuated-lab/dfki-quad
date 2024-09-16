#ifndef SYSIDTRAJECTORY_H
#define SYSIDTRAJECTORY_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "common/Interpolate.hpp"

class SysIdTrajectory {
 private:
  const double PI = 3.141592653589793238463;  // pi
  Eigen::Vector2d p0_, pf_;
  double segment_time_, a_, b_, c_;
  Eigen::Vector3d position_, pos_, vel_;
  Eigen::Vector3d SphericalPos2Cartesian(const Eigen::Ref<const Eigen::Vector2d> spherical) const;
  Eigen::Vector3d SphericalVel2Cartesian(const Eigen::Ref<const Eigen::Vector2d> spherical_pos,
                                         const Eigen::Ref<const Eigen::Vector2d> spherical_vel) const;

 public:
  SysIdTrajectory(const Eigen::Ref<const Eigen::Vector3d> position, const double a, const double b, const double c);
  void setInitialPosition(const Eigen::Ref<const Eigen::Vector2d> p0);
  void setFinalPosition(const Eigen::Ref<const Eigen::Vector2d> pf);
  void setEllipsoidParameters(const Eigen::Ref<const Eigen::Vector3d> position,
                              const double a,
                              const double b,
                              const double c);
  void setSegmentTime(const double segment_time);
  std::array<double, 3> getEllipsoidParams() const;
  Eigen::Vector3d getEllipsoidPosition() const;
  Eigen::Vector3d getInitialEEPositionCartesian() const;
  Eigen::Vector3d getFinalEEPositionCartesian() const;
  Eigen::Vector3d getCurrentEEPositionCartesian() const;
  Eigen::Vector3d getCurrentEEVelocityCartesian() const;
  double getSegmentTime() const;
  void computeEllipsoidalTrajectory(const double timer);
  void sampleNextEEPosition();
};

#endif